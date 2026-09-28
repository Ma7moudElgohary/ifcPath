from ifcpath.obstacles import edge_crosses_obstacle, wall_obstacle_from_vertices


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
