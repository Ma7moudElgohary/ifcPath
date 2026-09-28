from __future__ import annotations

import math
import sys

from ifcpath.exporter import load_inav


def fail(message: str) -> None:
    raise SystemExit(f"navmesh export qualification failed: {message}")


def triangle_area_xy(vertices) -> float:
    a, b, c = vertices
    return abs(
        (b[0] - a[0]) * (c[1] - a[1])
        - (b[1] - a[1]) * (c[0] - a[0])
    ) * 0.5


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: assert_navmesh_export.py <model.inav>")

    model = load_inav(sys.argv[1])
    cells = {cell.id: cell for cell in model.cells}
    if not cells:
        fail("no portable navmesh cells were exported")
    if len(cells) != len(model.cells):
        fail("duplicate navmesh cell IDs")

    known_spaces = {space.id for space in model.spaces}
    spaces_with_cells: set[str] = set()

    for cell in model.cells:
        if cell.space_id not in known_spaces:
            fail(f"cell {cell.id} references unknown space {cell.space_id}")
        spaces_with_cells.add(cell.space_id)
        if len(cell.vertices_m) != 3:
            fail(f"cell {cell.id} is not triangular")
        area = triangle_area_xy(cell.vertices_m)
        if not math.isfinite(area) or area <= 1e-10:
            fail(f"cell {cell.id} is degenerate")

        for neighbour_id in cell.neighbor_ids:
            neighbour = cells.get(neighbour_id)
            if neighbour is None:
                fail(f"cell {cell.id} references missing neighbour {neighbour_id}")
            if cell.id not in neighbour.neighbor_ids:
                fail(f"cell adjacency is not reciprocal: {cell.id} -> {neighbour_id}")
            if neighbour.space_id != cell.space_id:
                fail(f"metric cell adjacency crosses spaces: {cell.id} -> {neighbour_id}")
            if neighbour.level_id != cell.level_id:
                fail(f"metric cell adjacency crosses levels: {cell.id} -> {neighbour_id}")

    expected_cdt_spaces = int(model.metadata.get("cdt_floor_space_count", 0))
    if expected_cdt_spaces and len(spaces_with_cells) != expected_cdt_spaces:
        fail(
            f"expected cells for {expected_cdt_spaces} CDT spaces, "
            f"found {len(spaces_with_cells)}"
        )

    nodes_with_cells = [node for node in model.nodes if node.cell_id]
    if not nodes_with_cells:
        fail("no metric nodes reference exported cells")
    for node in nodes_with_cells:
        if node.cell_id not in cells:
            fail(f"node {node.id} references missing cell {node.cell_id}")

    print(
        "navmesh export qualification passed: "
        f"cells={len(cells)} spaces={len(spaces_with_cells)} "
        f"cell_nodes={len(nodes_with_cells)}"
    )


if __name__ == "__main__":
    main()
