from shapely.geometry import LineString, Point, Polygon

from ifcpath.cdt import build_space_cdt_graph


def test_cdt_graph_respects_concave_space_boundary():
    vertices = [
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
        (1.0, 4.0, 0.0),
        (0.0, 4.0, 0.0),
        (0.0, 0.0, 3.0),
    ]
    triangles = [
        (0, 1, 3),
        (1, 2, 3),
        (0, 3, 5),
        (3, 4, 5),
    ]

    points, edges = build_space_cdt_graph(vertices, triangles)
    floor = Polygon([(0, 0), (4, 0), (4, 1), (1, 1), (1, 4), (0, 4)])

    assert len(points) >= 4
    assert edges
    for point in points:
        assert floor.covers(Point(point[0], point[1]))
    for a, b, _ in edges:
        line = LineString([(points[a][0], points[a][1]), (points[b][0], points[b][1])])
        assert floor.covers(line)


def test_cdt_clearance_can_remove_too_narrow_space():
    vertices = [
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.0),
        (0.4, 3.0, 0.0),
        (0.0, 3.0, 0.0),
    ]
    triangles = [(0, 1, 2), (0, 2, 3)]

    points, edges = build_space_cdt_graph(vertices, triangles, clearance_m=0.25)
    assert points == []
    assert edges == []
