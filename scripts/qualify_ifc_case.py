from __future__ import annotations

import argparse
import json
from pathlib import Path

from ifcpath.corpus_qualification import qualify_case


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
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
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
