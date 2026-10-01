from __future__ import annotations

from types import SimpleNamespace

from shapely.geometry import Polygon

from ifcpath.exact_surface_seams import stitch_exact_verified_open_seams
from ifcpath.model import NavCell
from ifcpath.raycast_surface import SurfaceDetectionOptions
from ifcpath.semantic_heightfield_repair import SemanticRepairRegion


class _SupportEntity:
    PredefinedType = "FLOOR"

    def id(self):
        return 1

    def is_a(self, name=None):
        if name is None:
            return "IfcSlab"
        return name == "IfcElement" or name == "IfcSlab"


class _FakeIfc:
    def __init__(self):
        self.support = _SupportEntity()

    def by_id(self, entity_id):
        assert int(entity_id) == 1
        return self.support


class _Instance:
    def id(self):
        return 1


class _SupportTree:
    def __init__(self, *, has_support=True):
        self.has_support = has_support

    def select_ray(self, origin, _direction, *, length):
        del length
        if not self.has_support:
            return []
        return [
            SimpleNamespace(
                position=(origin[0], origin[1], 0.0),
                normal=(0.0, 0.0, 1.0),
                distance=origin[2],
                instance=_Instance(),
            )
        ]


def _split_room_cells():
    left = NavCell(
        id="left",
        vertices_m=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)),
        terrain="open",
        space_id="space:S",
        level_id="L1",
    )
    right = NavCell(
        id="right",
        vertices_m=((1.4, 0.0, 0.0), (2.4, 0.0, 0.0), (1.4, 1.0, 0.0)),
        terrain="open",
        space_id="space:S",
        level_id="L1",
    )
    return left, right


def _region():
    return SemanticRepairRegion(
        id="space:S",
        area_m2=3.0,
        min_z=-0.1,
        max_z=2.5,
        polygon=Polygon([(-0.2, -0.2), (2.6, -0.2), (2.6, 1.2), (-0.2, 1.2)]),
    )


def test_exact_verified_seam_repairs_clear_supported_raster_gap(monkeypatch):
    left, right = _split_room_cells()
    monkeypatch.setattr(
        "ifcpath.exact_surface_seams.build_semantic_repair_regions",
        lambda _: [_region()],
    )
    monkeypatch.setattr(
        "ifcpath.exact_surface_seams._native_geometry_tree",
        lambda _: _SupportTree(has_support=True),
    )
    monkeypatch.setattr(
        "ifcpath.exact_surface_seams._body_clearance_blocked",
        lambda *args, **kwargs: False,
    )

    stats = stitch_exact_verified_open_seams(
        _FakeIfc(),
        [left, right],
        options=SurfaceDetectionOptions(cell_size_m=0.2, agent_radius_m=0.22),
        max_gap_m=0.85,
    )

    assert stats.seams_added == 1
    assert stats.support_rays > 0
    assert stats.exact_body_checks > 0
    assert right.id in left.neighbor_ids
    assert left.id in right.neighbor_ids


def test_exact_verified_seam_refuses_gap_without_physical_support(monkeypatch):
    left, right = _split_room_cells()
    monkeypatch.setattr(
        "ifcpath.exact_surface_seams.build_semantic_repair_regions",
        lambda _: [_region()],
    )
    monkeypatch.setattr(
        "ifcpath.exact_surface_seams._native_geometry_tree",
        lambda _: _SupportTree(has_support=False),
    )

    stats = stitch_exact_verified_open_seams(
        _FakeIfc(),
        [left, right],
        options=SurfaceDetectionOptions(cell_size_m=0.2, agent_radius_m=0.22),
        max_gap_m=0.85,
    )

    assert stats.seams_added == 0
    assert stats.rejected_support > 0
    assert left.neighbor_ids == []
    assert right.neighbor_ids == []
