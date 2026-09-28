from ifcpath.geometry import TriangleSoup, build_radius_edges, sample_walkable_triangles


def test_walkable_horizontal_triangle_is_sampled():
    soup = TriangleSoup(
        vertices=[0,0,0, 2,0,0, 0,2,0],
        indices=[0,1,2],
    )
    pts = sample_walkable_triangles(soup, spacing_m=0.5, max_slope_deg=10)
    assert len(pts) > 0


def test_radius_graph_connects_nearby_points():
    edges = build_radius_edges([(0,0,0), (0.5,0,0), (2,0,0)], 1.0)
    assert len(edges) == 1
    assert edges[0][0:2] == (0, 1)
