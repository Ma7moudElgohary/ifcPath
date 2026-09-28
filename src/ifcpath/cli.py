from __future__ import annotations

import json
from pathlib import Path

import typer

from .builder import build_navigation
from .package import read_inav, write_inav
from .routing import RouteState, find_route
from .validation import validate_navigation

app = typer.Typer(help="IFCPath: BIM/IFC to portable indoor navigation")


@app.command()
def build(
    ifc: Path,
    output: Path = typer.Option(..., "--output", "-o"),
    floor_spacing: float = 0.75,
    stair_spacing: float = 0.25,
):
    model = build_navigation(ifc, floor_spacing_m=floor_spacing, stair_spacing_m=stair_spacing)
    model.warnings = validate_navigation(model)
    path = write_inav(model, output, source_name=ifc.name)
    typer.echo(f"Wrote {path}")
    typer.echo(
        f"levels={len(model.levels)} spaces={len(model.spaces)} portals={len(model.portals)} "
        f"nodes={len(model.nodes)} edges={len(model.edges)} warnings={len(model.warnings)}"
    )


@app.command()
def inspect(inav: Path):
    model = read_inav(inav)
    typer.echo(model.model_dump_json(indent=2))


@app.command()
def route(
    inav: Path,
    start: str = typer.Option(..., help="x,y,z in metres"),
    end: str = typer.Option(..., help="x,y,z in metres"),
    block_portal: list[str] = typer.Option(None),
    block_space: list[str] = typer.Option(None),
):
    def vec(text: str):
        x, y, z = (float(v.strip()) for v in text.split(","))
        return (x, y, z)

    model = read_inav(inav)
    state = RouteState(
        blocked_portal_ids=set(block_portal or []),
        blocked_space_ids=set(block_space or []),
    )
    result = find_route(model, vec(start), vec(end), state)
    if not result:
        raise typer.Exit(code=2)
    typer.echo(json.dumps({
        "nodeIds": result.node_ids,
        "pointsM": result.points_m,
        "totalCost": result.total_cost,
        "totalLengthM": result.total_length_m,
    }, indent=2))


if __name__ == "__main__":
    app()
