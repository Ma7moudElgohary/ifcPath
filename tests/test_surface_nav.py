from ifcpath.model import NavCell
from ifcpath.surface_nav import closest_cell, connect_cells_by_shared_edges, find_cell_corridor, find_surface_route, stitch_surface_seams, surface_components, walkable_surface_cells
from ifcpath.surface_reconstruction import _clear_surface_adjacency


def test_sloped_stair_surface_is_continuous():
    vertices = [(0,0,0),(1,0,0),(0,1,1),(1,1,1),(0,2,2),(1,2,2)]
    triangles = [(0,1,2),(1,3,2),(2,3,4),(3,5,4)]
    cells = walkable_surface_cells(vertices, triangles, id_prefix="stair:S1", terrain="stair", max_slope_deg=55.0)
    assert len(cells) == 4
    assert find_cell_corridor(cells, cells[0].id, cells[-1].id) == [cell.id for cell in cells]
    assert {cell.terrain for cell in cells} == {"stair"}


def test_vertical_riser_is_not_walkable_surface():
    cells = walkable_surface_cells([(0,0,0),(1,0,0),(0,0,1)], [(0,1,2)], id_prefix="stair:S", terrain="stair", max_slope_deg=55.0)
    assert cells == []


def test_seam_stitching_joins_floor_to_stair_without_radius_graph():
    floor = walkable_surface_cells(
        [(0,0,0),(1,0,0),(0,1,0)], [(0,1,2)],
        id_prefix="floor", terrain="open", max_slope_deg=55, space_id="S1"
    )
    stair = walkable_surface_cells(
        [(0,1.05,0.02),(1,1.05,0.02),(0,2,1)], [(0,1,2)],
        id_prefix="stair", terrain="stair", max_slope_deg=55
    )
    cells=floor+stair
    assert len(surface_components(cells)) == 2
    assert stitch_surface_seams(cells, max_gap_m=0.10, max_vertical_gap_m=0.10) >= 1
    assert len(surface_components(cells)) == 1


def test_xyz_route_projects_to_and_crosses_sloped_surface():
    vertices=[(0,0,0),(1,0,0),(0,1,1),(1,1,1),(0,2,2),(1,2,2)]
    cells=walkable_surface_cells(vertices,[(0,1,2),(1,3,2),(2,3,4),(3,5,4)],
        id_prefix="stair",terrain="stair",max_slope_deg=55)
    route=find_surface_route(cells,(0.2,0.1,0.4),(0.8,1.9,2.3),{"stair":1.25})
    assert len(route) >= 2
    assert route[0][2] < route[-1][2]
    assert closest_cell(cells,route[0])[2] < 1e-6
    assert closest_cell(cells,route[-1])[2] < 1e-6


def test_rebuild_after_space_labelling_drops_stale_cross_space_edge():
    left = NavCell(
        id="left",
        vertices_m=((0, 0, 0), (1, 0, 0), (1, 1, 0)),
        space_id="A",
        terrain="open",
        neighbor_ids=["right"],
        portals={"right": ((1, 0, 0), (1, 1, 0))},
    )
    right = NavCell(
        id="right",
        vertices_m=((1, 0, 0), (2, 1, 0), (1, 1, 0)),
        space_id="B",
        terrain="open",
        neighbor_ids=["left"],
        portals={"left": ((1, 0, 0), (1, 1, 0))},
    )
    cells = [left, right]

    _clear_surface_adjacency(cells)
    connect_cells_by_shared_edges(cells)

    assert left.neighbor_ids == []
    assert right.neighbor_ids == []
    assert left.portals == {}
    assert right.portals == {}
