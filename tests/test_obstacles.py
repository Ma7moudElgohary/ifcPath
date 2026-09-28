from ifcpath.obstacles import (
    edge_crosses_obstacle,
    mesh_obstacle_from_triangles,
    obstacle_intersects_pedestrian_volume,
    wall_obstacle_from_vertices,
)


def test_wall_obstacle_blocks_crossing_edge_at_same_height():
    wall = wall_obstacle_from_vertices([
        (0.9, -1.0, 0.0),
        (1.1, -1.0, 0.0),
        (1.1, 1.0, 3.0),
        (0.9, 1.0, 3.0),
    ])
    assert wall is not None
    assert edge_crosses_obstacle((0.0, 0.0, 1.0), (2.0, 0.0, 1.0), [wall])


def test_wall_obstacle_does_not_block_edge_on_other_level():
    wall = wall_obstacle_from_vertices([
        (0.9, -1.0, 0.0),
        (1.1, -1.0, 0.0),
        (1.1, 1.0, 3.0),
        (0.9, 1.0, 3.0),
    ])
    assert wall is not None
    assert not edge_crosses_obstacle((0.0, 0.0, 5.0), (2.0, 0.0, 5.0), [wall])


def test_non_crossing_edge_remains_available():
    wall = wall_obstacle_from_vertices([
        (0.9, -1.0, 0.0),
        (1.1, -1.0, 0.0),
        (1.1, 1.0, 3.0),
        (0.9, 1.0, 3.0),
    ])
    assert wall is not None
    assert not edge_crosses_obstacle((0.0, 2.0, 1.0), (2.0, 2.0, 1.0), [wall])


def test_mesh_obstacle_uses_projected_triangle_silhouette():
    vertices = [
        (1.0, 1.0, 0.0),
        (2.0, 1.0, 0.0),
        (2.0, 2.0, 0.0),
        (1.0, 2.0, 0.0),
        (1.0, 1.0, 3.0),
        (2.0, 1.0, 3.0),
        (2.0, 2.0, 3.0),
        (1.0, 2.0, 3.0),
    ]
    triangles = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ]
    obstacle = mesh_obstacle_from_triangles(vertices, triangles)
    assert obstacle is not None
    assert abs(obstacle.footprint.area - 1.0) < 1e-9
    assert obstacle.z_min == 0.0
    assert obstacle.z_max == 3.0


def test_overhead_obstacle_does_not_intersect_pedestrian_volume():
    obstacle = wall_obstacle_from_vertices([
        (0.0, 0.0, 2.5),
        (1.0, 0.0, 2.5),
        (1.0, 1.0, 3.0),
        (0.0, 1.0, 3.0),
    ])
    assert obstacle is not None
    assert not obstacle_intersects_pedestrian_volume(obstacle, floor_z=0.0, agent_height_m=1.8)
    assert obstacle_intersects_pedestrian_volume(obstacle, floor_z=0.0, agent_height_m=2.6)
