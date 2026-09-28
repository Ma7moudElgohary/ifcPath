from ifcpath.geometry import (
    build_radius_edges,
    sample_space_floor_triangles,
    sample_walkable_triangles,
)


def test_samples_horizontal_triangle():
    verts = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)]
    pts = sample_walkable_triangles(verts, [(0, 1, 2)], spacing_m=0.5)
    assert len(pts) > 3
    assert all(abs(p[2]) < 1e-9 for p in pts)


def test_samples_space_bottom_even_with_downward_winding():
    # Bottom triangle is wound downward, top triangle upward. Space-floor
    # sampling must choose the bottom by elevation rather than face winding.
    verts = [
        (0.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 3.0),
        (2.0, 0.0, 3.0),
        (0.0, 2.0, 3.0),
    ]
    pts = sample_space_floor_triangles(verts, [(0, 1, 2), (3, 4, 5)], spacing_m=0.5)
    assert len(pts) > 3
    assert all(abs(p[2]) < 1e-9 for p in pts)


def test_radius_edges_connect_near_points_only():
    pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
    edges = build_radius_edges(pts, 1.1)
    pairs = {(a, b) for a, b, _ in edges}
    assert (0, 1) in pairs
    assert (1, 2) not in pairs


def test_radius_edges_bound_dense_neighbourhoods():
    pts = [(i * 0.01, 0.0, 0.0) for i in range(30)]
    edges = build_radius_edges(pts, 1.0, max_neighbors=4)
    assert len(edges) < len(pts) * 4
    assert len(edges) < 435
