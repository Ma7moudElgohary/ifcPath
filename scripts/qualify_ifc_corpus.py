from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path
from typing import Any

from ifcpath.corpus_qualification import classify_result, verify_download


CATEGORY_LABELS = {
    "A_IFC_PARSING_SCHEMA": "A. IFC parsing/schema problem",
    "B_GEOMETRY_EXTRACTION": "B. geometry extraction problem",
    "C_FLOOR_POLYGON_CDT": "C. floor polygon/CDT problem",
    "D_SPACE_SEMANTICS": "D. space semantics problem",
    "E_DOOR_PORTAL_BINDING": "E. door/portal binding problem",
    "F_OPEN_BOUNDARY_RECOVERY": "F. open-boundary recovery problem",
    "G_STAIR_RAMP_RECONSTRUCTION": "G. stair/ramp reconstruction problem",
    "H_ELEVATOR_SEMANTICS": "H. elevator semantic problem",
    "I_SURFACE_SEAM_STITCH": "I. surface seam/stitch problem",
    "J_ROUTE_FUNNEL": "J. route/funnel problem",
    "K_READINESS_CLASSIFICATION": "K. readiness classification problem",
    "L_VIEWER_WEBIFC": "L. viewer/WebIFC issue",
    "M_PERFORMANCE_SCALABILITY": "M. performance/scalability issue",
    "N_UNSUPPORTED_OR_UNKNOWN": "N. unsupported but expected/unknown model",
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Download and qualify the pinned public IFC corpus")
    parser.add_argument("--manifest", type=Path, default=Path("qualification/ifc_corpus.json"))
    parser.add_argument("--tier", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--case", action="append", dest="case_ids", default=[])
    parser.add_argument("--cache-dir", type=Path, default=Path(".cache/ifc-corpus"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/ifc-corpus"))
    parser.add_argument("--no-fail", action="store_true", help="Write reports without failing the process")
    args = parser.parse_args()

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    defaults = manifest.get("defaults", {})
    cases = [case for case in manifest["cases"] if args.tier in case.get("tiers", [])]
    if args.case_ids:
        wanted = set(args.case_ids)
        cases = [case for case in cases if case["id"] in wanted]
        missing = wanted - {case["id"] for case in cases}
        if missing:
            raise SystemExit(f"requested cases are not present in tier {args.tier}: {sorted(missing)}")
    if not cases:
        raise SystemExit(f"no corpus cases selected for tier {args.tier}")

    args.cache_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    case_dir = args.output_dir / "cases"
    case_dir.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    results: list[dict[str, Any]] = []
    with tempfile.TemporaryDirectory(prefix="ifcpath-corpus-") as temp_name:
        temp = Path(temp_name)
        defaults_path = temp / "defaults.json"
        defaults_path.write_text(json.dumps(defaults), encoding="utf-8")
        for index, case in enumerate(cases, 1):
            print(f"[{index}/{len(cases)}] {case['id']}: {case.get('name', case['id'])}", flush=True)
            result = run_case(case, defaults, defaults_path, args.cache_dir, case_dir, temp)
            results.append(result)
            print(
                f"  -> {result.get('status')} stage={result.get('stage')} "
                f"categories={','.join(result.get('categories', [])) or '-'}",
                flush=True,
            )

    summary = build_summary(args.tier, manifest, results, time.perf_counter() - started)
    json_path = args.output_dir / f"tier-{args.tier}-report.json"
    md_path = args.output_dir / f"tier-{args.tier}-failure-matrix.md"
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    md_path.write_text(render_markdown(summary), encoding="utf-8")
    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    print(
        f"corpus tier {args.tier}: total={summary['total']} passed={summary['passed']} "
        f"failed={summary['failed']} errors={summary['errors']} time={summary['duration_seconds']:.1f}s"
    )
    if not args.no_fail and (summary["failed"] or summary["errors"]):
        raise SystemExit(2)


def run_case(
    case: dict[str, Any],
    defaults: dict[str, Any],
    defaults_path: Path,
    cache_dir: Path,
    case_dir: Path,
    temp: Path,
) -> dict[str, Any]:
    case_id = case["id"]
    output_path = case_dir / f"{case_id}.json"
    try:
        ifc_path = download_case(case, cache_dir)
    except Exception as exc:
        result = {
            "case_id": case_id,
            "name": case.get("name", case_id),
            "expected_outcome": case.get("expected_outcome", "PASS-PARTIAL"),
            "status": "error",
            "stage": "download",
            "categories": ["A_IFC_PARSING_SCHEMA"],
            "expectation_failures": [f"source acquisition failed: {type(exc).__name__}: {exc}"],
            "exception_type": type(exc).__name__,
            "exception_message": str(exc),
        }
        output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return result

    case_path = temp / f"{case_id}.json"
    case_path.write_text(json.dumps(case), encoding="utf-8")
    timeout = int(case.get("timeout_seconds", defaults.get("timeout_seconds", 180)))
    command = [
        sys.executable,
        "scripts/qualify_ifc_case.py",
        "--case-json", str(case_path),
        "--ifc", str(ifc_path),
        "--output", str(output_path),
        "--defaults-json", str(defaults_path),
    ]
    try:
        completed = subprocess.run(command, text=True, capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        result = {
            "case_id": case_id,
            "name": case.get("name", case_id),
            "expected_outcome": case.get("expected_outcome", "PASS-PARTIAL"),
            "status": "error",
            "stage": "timeout",
            "categories": ["M_PERFORMANCE_SCALABILITY"],
            "expectation_failures": [f"qualification exceeded {timeout}s timeout"],
            "exception_type": "TimeoutExpired",
            "exception_message": str(exc),
        }
        output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return result

    if output_path.exists():
        result = json.loads(output_path.read_text(encoding="utf-8"))
    else:
        result = {
            "case_id": case_id,
            "name": case.get("name", case_id),
            "expected_outcome": case.get("expected_outcome", "PASS-PARTIAL"),
            "status": "error",
            "stage": "worker",
            "categories": ["N_UNSUPPORTED_OR_UNKNOWN"],
            "expectation_failures": [f"worker exited {completed.returncode} without a result file"],
        }
    result["worker_exit_code"] = completed.returncode
    result["worker_stdout"] = completed.stdout[-8000:]
    result["worker_stderr"] = completed.stderr[-8000:]
    if completed.returncode != 0 and result.get("status") == "pass":
        result["status"] = "error"
        result.setdefault("expectation_failures", []).append(
            f"worker exited {completed.returncode} despite pass result"
        )
        result["categories"] = classify_result_dict(result)
    output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def download_case(case: dict[str, Any], cache_dir: Path) -> Path:
    source = case["source"]
    suffix = Path(source.get("path", "model.ifc")).suffix or ".ifc"
    destination = cache_dir / f"{case['id']}{suffix}"
    if destination.exists():
        data = destination.read_bytes()
        failures = verify_download(data, source)
        if not failures:
            return destination
        destination.unlink()

    request = urllib.request.Request(
        source["url"],
        headers={"User-Agent": "IfcPath-qualification/0.1", "Accept": "application/octet-stream"},
    )
    with urllib.request.urlopen(request, timeout=120) as response:
        data = response.read()
    failures = verify_download(data, source)
    if failures:
        raise ValueError("; ".join(failures))
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_bytes(data)
    temporary.replace(destination)
    return destination


def build_summary(tier: int, manifest: dict[str, Any], results: list[dict[str, Any]], duration: float) -> dict[str, Any]:
    category_counts: dict[str, int] = {}
    for result in results:
        for category in result.get("categories", []):
            category_counts[category] = category_counts.get(category, 0) + 1
    return {
        "schema_version": 1,
        "tier": tier,
        "corpus_schema_version": manifest.get("schema_version"),
        "duration_seconds": duration,
        "total": len(results),
        "passed": sum(result.get("status") == "pass" for result in results),
        "failed": sum(result.get("status") == "fail" for result in results),
        "errors": sum(result.get("status") == "error" for result in results),
        "category_counts": category_counts,
        "results": results,
    }


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        f"# IFC corpus tier {summary['tier']} qualification",
        "",
        f"Total: **{summary['total']}** · Passed: **{summary['passed']}** · Failed: **{summary['failed']}** · Errors: **{summary['errors']}** · Duration: **{summary['duration_seconds']:.1f}s**",
        "",
        "| Case | Expected | Status | Schema | Levels | Spaces | Cells | Ready | Build s | Route | Categories |",
        "|---|---|---|---|---:|---:|---:|---|---:|---|---|",
    ]
    for result in summary["results"]:
        counts = result.get("counts", {})
        routing = result.get("routing", {})
        categories = ", ".join(result.get("categories", [])) or "—"
        route = (
            "multi✓" if routing.get("multilevel_route_exists") else
            "local✓" if routing.get("representative_route_exists") else
            "—"
        )
        build_seconds = result.get("build_seconds")
        lines.append(
            "| {case} | {expected} | **{status}** | {schema} | {levels} | {spaces} | {cells} | {ready} | {build} | {route} | {categories} |".format(
                case=result.get("case_id", "?"),
                expected=result.get("expected_outcome", "?"),
                status=result.get("status", "?"),
                schema=result.get("schema_actual") or "—",
                levels=counts.get("levels", "—"),
                spaces=counts.get("spaces", "—"),
                cells=counts.get("cells", "—"),
                ready=counts.get("navigation_ready", "—"),
                build=f"{build_seconds:.2f}" if isinstance(build_seconds, (int, float)) else "—",
                route=route,
                categories=categories,
            )
        )
    lines += ["", "## Failures and limitations", ""]
    problem_results = [result for result in summary["results"] if result.get("status") != "pass"]
    if not problem_results:
        lines.append("No expectation failures in this tier.")
    else:
        for result in problem_results:
            lines.append(f"### {result.get('case_id')}")
            for failure in result.get("expectation_failures", []):
                lines.append(f"- {failure}")
            for category in result.get("categories", []):
                lines.append(f"- Category: {CATEGORY_LABELS.get(category, category)}")
            for issue in result.get("issues", [])[:20]:
                lines.append(f"- {issue.get('severity', '').upper()} `{issue.get('code', '')}`: {issue.get('message', '')}")
            lines.append("")
    lines += ["## Failure categories", ""]
    if summary["category_counts"]:
        for category, count in sorted(summary["category_counts"].items()):
            lines.append(f"- {CATEGORY_LABELS.get(category, category)}: {count}")
    else:
        lines.append("- None")
    lines.append("")
    return "\n".join(lines)


def classify_result_dict(result: dict[str, Any]) -> list[str]:
    # Keep parent-process failures conservatively classified. Rich semantic
    # classification happens inside the worker where a typed result is available.
    categories = set(result.get("categories", []))
    if result.get("stage") == "timeout":
        categories.add("M_PERFORMANCE_SCALABILITY")
    elif not categories:
        categories.add("N_UNSUPPORTED_OR_UNKNOWN")
    return sorted(categories)


if __name__ == "__main__":
    main()
