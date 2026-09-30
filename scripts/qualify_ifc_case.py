from __future__ import annotations

import argparse
import json
import traceback
from pathlib import Path

from ifcpath.corpus_qualification import qualify_case
from ifcpath.ifc_loader import BuildOptions, build_from_ifc


def main() -> None:
    parser = argparse.ArgumentParser(description="Qualify one IFC corpus case in an isolated process")
    parser.add_argument("--case-json", required=True, type=Path)
    parser.add_argument("--ifc", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--defaults-json", type=Path)
    args = parser.parse_args()

    case = json.loads(args.case_json.read_text(encoding="utf-8"))
    defaults = (
        json.loads(args.defaults_json.read_text(encoding="utf-8"))
        if args.defaults_json
        else {}
    )
    result = qualify_case(case, args.ifc, defaults)
    payload = result.to_dict()

    # The normal qualification result intentionally catches case-local failures so
    # one malformed IFC cannot abort the corpus. Re-run only failed builds to
    # capture a complete traceback for architectural diagnosis in CI artifacts.
    if result.status == "error" and result.stage == "build":
        options = dict(defaults.get("build_options", {}))
        options.update(case.get("build_options", {}))
        try:
            build_from_ifc(args.ifc, BuildOptions(**options))
        except Exception:
            payload["traceback"] = traceback.format_exc()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps({
        "case_id": result.case_id,
        "status": result.status,
        "stage": result.stage,
        "categories": result.categories,
        "expectation_failures": result.expectation_failures,
    }))
    if result.status != "pass":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
