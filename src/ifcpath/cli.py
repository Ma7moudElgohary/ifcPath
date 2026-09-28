from __future__ import annotations

import argparse

from .exporter import save_inav
from .ifc_loader import BuildOptions, build_from_ifc


def main() -> None:
    parser = argparse.ArgumentParser(prog="ifcpath", description="Generate portable indoor navigation from IFC")
    sub = parser.add_subparsers(dest="command", required=True)

    build = sub.add_parser("build", help="Build an .inav file from IFC")
    build.add_argument("ifc")
    build.add_argument("-o", "--output", required=True)
    build.add_argument("--floor-spacing", type=float, default=0.8)
    build.add_argument("--stair-spacing", type=float, default=0.25)
    build.add_argument("--connect-distance", type=float, default=1.25)

    args = parser.parse_args()
    if args.command == "build":
        model = build_from_ifc(
            args.ifc,
            BuildOptions(
                floor_spacing_m=args.floor_spacing,
                stair_spacing_m=args.stair_spacing,
                connect_distance_m=args.connect_distance,
            ),
        )
        output = save_inav(model, args.output)
        print(f"Wrote {output}")
        print(f"nodes={len(model.nodes)} edges={len(model.edges)} spaces={len(model.spaces)} portals={len(model.portals)}")


if __name__ == "__main__":
    main()
