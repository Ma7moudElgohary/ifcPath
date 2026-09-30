from __future__ import annotations

import hashlib
import json
from pathlib import Path

from ifcpath.corpus_qualification import (
    CorpusCaseResult,
    evaluate_expectations,
    git_blob_sha1,
    read_ifc_schema,
    verify_download,
)


def test_git_blob_sha1_matches_git_object_format() -> None:
    data = b"hello\n"
    expected = hashlib.sha1(b"blob 6\0hello\n").hexdigest()
    assert git_blob_sha1(data) == expected


def test_verify_download_checks_size_sha256_and_git_blob() -> None:
    data = b"IFC corpus fixture"
    source = {
        "size_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "git_blob_sha1": git_blob_sha1(data),
    }
    assert verify_download(data, source) == []
    assert verify_download(data + b"!", source)


def test_read_ifc_schema_does_not_need_ifcopenshell(tmp_path: Path) -> None:
    path = tmp_path / "sample.ifc"
    path.write_bytes(
        b"ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('IFC4X3_ADD2'));\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;"
    )
    assert read_ifc_schema(path) == "IFC4X3_ADD2"


def test_pass_partial_does_not_require_navigation_ready() -> None:
    result = CorpusCaseResult(
        case_id="geometry-only",
        name="geometry only",
        expected_outcome="PASS-PARTIAL",
        schema_expected="IFC4",
        schema_actual="IFC4",
        status="pass",
        validation_stats={"surface_navigation_ready": False},
    )
    assert evaluate_expectations({"expected_outcome": "PASS-PARTIAL"}, result) == []


def test_pass_nav_requires_surface_readiness() -> None:
    result = CorpusCaseResult(
        case_id="nav",
        name="nav",
        expected_outcome="PASS-NAV",
        schema_expected="IFC4",
        schema_actual="IFC4",
        validation_stats={"surface_navigation_ready": False},
    )
    failures = evaluate_expectations({"expected_outcome": "PASS-NAV"}, result)
    assert failures == ["PASS-NAV case is not surface navigation ready"]


def _all_manifest_cases() -> tuple[list[Path], list[dict]]:
    paths = sorted(Path("qualification").glob("ifc_corpus*.json"))
    cases: list[dict] = []
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        cases.extend(document["cases"])
    return paths, cases


def test_manifests_have_unique_cases_pinned_sources_and_diversity() -> None:
    paths, cases = _all_manifest_cases()
    ids = [case["id"] for case in cases]
    assert len(paths) >= 2
    assert len(ids) == len(set(ids))
    assert len(ids) >= 15
    assert {case["schema"] for case in cases} >= {"IFC2X3", "IFC4", "IFC4X3_ADD2"}

    repositories = {case["source"]["repository"] for case in cases}
    assert len(repositories) >= 5

    authoring_sources = " ".join(str(case.get("authoring_source", "")) for case in cases).lower()
    for expected in ("revit", "archicad", "ifcopenshell", "blenderbim"):
        assert expected in authoring_sources

    for case in cases:
        source = case["source"]
        assert source["commit"]
        assert source["license"]
        assert source["redistributable"] is True
        assert source.get("sha256") or source.get("git_blob_sha1")
        assert case["expected_outcome"] in {"PASS-NAV", "PASS-PARTIAL", "EXPECTED-REJECT"}
        assert case["tiers"]
