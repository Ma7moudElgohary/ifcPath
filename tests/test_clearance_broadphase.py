from __future__ import annotations

from ifcpath.raycast_surface import (
    SurfaceDetectionOptions,
    SurfaceDetectionStats,
    _body_clearance_blocked,
)


class _Entity:
    def __init__(self, entity_id: int, class_name: str, *, physical: bool = True):
        self._id = entity_id
        self._class_name = class_name
        self._physical = physical

    def id(self):
        return self._id

    def is_a(self, name=None):
        if name is None:
            return self._class_name
        if name == "IfcElement":
            return self._physical
        return self._class_name == name


class _IfcFile:
    def __init__(self, *entities):
        self.entities = {entity.id(): entity for entity in entities}

    def by_id(self, entity_id):
        return self.entities[int(entity_id)]


class _Tree:
    def __init__(self, *, broadphase=(), precise=()):
        self.broadphase = list(broadphase)
        self.precise = list(precise)
        self.box_calls = 0
        self.precise_calls = 0

    def select_box(self, center, *, extend):
        assert all(isinstance(value, float) for value in center)
        assert extend > 0.0
        self.box_calls += 1
        return self.broadphase

    def select(self, center, *, extend):
        assert all(isinstance(value, float) for value in center)
        assert extend > 0.0
        self.precise_calls += 1
        return self.precise


def _blocked(tree, ifc_file, *, ignored=frozenset()):
    stats = SurfaceDetectionStats()
    result = _body_clearance_blocked(
        tree,
        ifc_file,
        ignored_entity_ids=set(ignored),
        position=(0.0, 0.0, 0.0),
        opts=SurfaceDetectionOptions(agent_height_m=1.8, agent_radius_m=0.22),
        classify=lambda entity: entity.is_a(),
        stats=stats,
    )
    return result, stats


def test_empty_or_ignored_broadphase_skips_expensive_precise_queries():
    support = _Entity(1, "IfcSlab")
    ifc_file = _IfcFile(support)
    tree = _Tree(broadphase=[support], precise=[support])

    blocked, stats = _blocked(tree, ifc_file, ignored={1})

    assert blocked is False
    assert tree.box_calls == 3
    assert tree.precise_calls == 0
    assert stats.clearance_broadphase_queries == 3
    assert stats.clearance_precise_skips == 3
    assert stats.clearance_precise_queries == 0


def test_nonblocking_semantic_geometry_does_not_force_precise_query():
    door = _Entity(2, "IfcDoor")
    ifc_file = _IfcFile(door)
    tree = _Tree(broadphase=[door], precise=[door])

    blocked, stats = _blocked(tree, ifc_file)

    assert blocked is False
    assert tree.precise_calls == 0
    assert stats.clearance_precise_skips == 3


def test_broadphase_false_positive_is_refined_by_exact_sphere_query():
    wall = _Entity(3, "IfcWall")
    ifc_file = _IfcFile(wall)
    tree = _Tree(broadphase=[wall], precise=[])

    blocked, stats = _blocked(tree, ifc_file)

    assert blocked is False
    assert tree.box_calls == 3
    assert tree.precise_calls == 3
    assert stats.clearance_precise_queries == 3


def test_exact_blocking_geometry_still_rejects_walkable_sample():
    column = _Entity(4, "IfcColumn")
    ifc_file = _IfcFile(column)
    tree = _Tree(broadphase=[column], precise=[column])

    blocked, stats = _blocked(tree, ifc_file)

    assert blocked is True
    assert tree.box_calls == 1
    assert tree.precise_calls == 1
    assert stats.clearance_precise_queries == 1
