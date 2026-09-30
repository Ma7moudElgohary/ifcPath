from __future__ import annotations

from ifcpath.raycast_surface import (
    SurfaceDetectionOptions,
    SurfaceSamplingDomain,
    _SupportSample,
    _headroom_blocked,
    _samples_compatible,
    _triangulate_samples,
    ifc_walkable_support_role,
)


class _FakeIfcEntity:
    def __init__(self, class_name: str, predefined_type: str | None = None):
        self._class_name = class_name
        self.PredefinedType = predefined_type

    def is_a(self, class_name: str | None = None):
        if class_name is None:
            return self._class_name
        return self._class_name == class_name


def test_support_role_uses_ifc_semantics_not_flat_top_guessing():
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcSlab", "FLOOR")) == "open"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcCovering", "FLOORING")) == "open"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcRampFlight")) == "ramp"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcStairFlight")) == "stair"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcTransportElement", "ESCALATOR")) == "stair"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcTransportElement", "MOVINGWALKWAY")) == "open"

    # Flat furniture/columns/roofs are physical geometry but must not become
    # support merely because a ray can hit a horizontal top face.
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcFurniture")) is None
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcColumn")) is None
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcSlab", "ROOF")) is None


def test_headroom_filter_uses_nearest_distinct_surface_above_support():
    # Downward-ray order: ceiling top/bottom then floor support.
    positions = [(0.0, 0.0, 2.20), (0.0, 0.0, 2.10), (0.0, 0.0, 0.0)]
    assert not _headroom_blocked(2, positions, 0.0, 1.8)

    low_ceiling = [(0.0, 0.0, 1.70), (0.0, 0.0, 0.0)]
    assert _headroom_blocked(1, low_ceiling, 0.0, 1.8)

    # Coincident finish/shell faces are ignored rather than interpreted as a
    # zero-height ceiling.
    coincident = [(0.0, 0.0, 0.01), (0.0, 0.0, 0.0)]
    assert not _headroom_blocked(1, coincident, 0.0, 1.8)


def test_sample_connectivity_accepts_normal_stair_rise_but_rejects_large_jump():
    opts = SurfaceDetectionOptions(cell_size_m=0.30, max_climb_m=0.22, max_slope_deg=50.0)
    a = _SupportSample(0, 0, (0.0, 0.0, 0.0), 1, "stair")
    normal_step = _SupportSample(1, 0, (0.30, 0.0, 0.18), 1, "stair")
    high_jump = _SupportSample(1, 0, (0.30, 0.0, 0.60), 1, "stair")

    assert _samples_compatible(a, normal_step, opts)
    assert not _samples_compatible(a, high_jump, opts)


def test_multilayer_samples_become_continuous_triangle_surface():
    opts = SurfaceDetectionOptions(
        cell_size_m=0.30,
        max_climb_m=0.22,
        max_slope_deg=50.0,
        minimum_component_cells=1,
        minimum_component_area_m2=0.0,
    )
    domain = SurfaceSamplingDomain(
        id="stair-flight",
        bounds=(0.0, 0.0, 0.0, 0.3, 0.3, 0.2),
        support_entity_ids=frozenset({7}),
        terrain="stair",
    )
    samples = [
        _SupportSample(0, 0, (0.0, 0.0, 0.00), 7, "stair"),
        _SupportSample(1, 0, (0.3, 0.0, 0.18), 7, "stair"),
        _SupportSample(0, 1, (0.0, 0.3, 0.00), 7, "stair"),
        _SupportSample(1, 1, (0.3, 0.3, 0.18), 7, "stair"),
    ]

    cells = _triangulate_samples(domain, samples, opts)

    assert len(cells) == 2
    assert {cell.terrain for cell in cells} == {"stair"}
    assert cells[1].id in cells[0].neighbor_ids
    assert cells[0].id in cells[1].neighbor_ids
    assert min(v[2] for cell in cells for v in cell.vertices_m) == 0.0
    assert max(v[2] for cell in cells for v in cell.vertices_m) == 0.18
