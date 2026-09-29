from ifcpath.surface_nav import find_cell_corridor, stitch_surface_seams, surface_components, walkable_surface_cells


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
