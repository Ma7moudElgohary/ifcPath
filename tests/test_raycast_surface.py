from __future__ import annotations

from ifcpath.raycast_surface import (
    SurfaceDetectionOptions,
    SurfaceDetectionStats,
    SurfaceSamplingDomain,
    _SupportCandidate,
    _SupportSample,
    _WalkableSpan,
    _erode_walkable_spans,
    _field_spans_compatible,
    _group_support_candidates,
    _headroom_blocked,
    _intervals_overlap,
    _preferred_support_candidate,
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


def _span(ix, iy, z=0.0, *, blocked=False, terrain="open", owner_id=1):
    return _WalkableSpan(
        ix=ix,
        iy=iy,
        position=(ix * 0.15, iy * 0.15, z),
        owner_id=owner_id,
        terrain=terrain,
        ceiling_z=None,
        free_height_m=float("inf"),
        direct_blocked=blocked,
    )


def test_support_role_uses_ifc_semantics_not_flat_top_guessing():
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcSlab", "FLOOR")) == "open"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcCovering", "FLOORING")) == "open"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcRampFlight")) == "ramp"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcStairFlight")) == "stair"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcTransportElement", "ESCALATOR")) == "stair"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcTransportElement", "MOVINGWALKWAY")) == "open"
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcFurniture")) is None
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcColumn")) is None
    assert ifc_walkable_support_role(_FakeIfcEntity("IfcSlab", "ROOF")) is None


def test_headroom_filter_uses_nearest_distinct_surface_above_support():
    positions = [(0.0, 0.0, 2.20), (0.0, 0.0, 2.10), (0.0, 0.0, 0.0)]
    assert not _headroom_blocked(2, positions, 0.0, 1.8)

    low_ceiling = [(0.0, 0.0, 1.70), (0.0, 0.0, 0.0)]
    assert _headroom_blocked(1, low_ceiling, 0.0, 1.8)

    coincident = [(0.0, 0.0, 0.01), (0.0, 0.0, 0.0)]
    assert not _headroom_blocked(1, coincident, 0.0, 1.8)


def test_closed_solid_interval_overlap_matches_body_volume():
    assert _intervals_overlap(0.0, 2.5, 0.02, 1.8, 0.01)
    assert not _intervals_overlap(3.0, 3.2, 0.02, 1.8, 0.01)
    assert not _intervals_overlap(-0.4, 0.0, 0.02, 1.8, 0.01)


def test_coincident_support_representations_collapse_to_one_physical_layer():
    candidates = [
        _SupportCandidate(2, (1.0, 2.0, 0.020), 10, "open"),
        _SupportCandidate(3, (1.0, 2.0, 0.005), 11, "open"),
        _SupportCandidate(5, (1.0, 2.0, -0.40), 12, "open"),
    ]
    layers = _group_support_candidates(candidates, tolerance=0.025)
    assert [len(layer) for layer in layers] == [2, 1]
    assert {candidate.owner_id for candidate in layers[0]} == {10, 11}
    assert layers[1][0].owner_id == 12


def test_stair_support_wins_terrain_when_coincident_with_open_floor():
    open_floor = _SupportCandidate(3, (0.0, 0.0, 0.010), 20, "open")
    stair = _SupportCandidate(4, (0.0, 0.0, 0.000), 21, "stair")
    layers = _group_support_candidates([open_floor, stair], tolerance=0.025)
    chosen = _preferred_support_candidate(layers[0])
    assert chosen.owner_id == 21
    assert chosen.terrain == "stair"


def test_separate_vertical_supports_are_not_deduplicated():
    floor = _SupportCandidate(2, (0.0, 0.0, 0.0), 30, "open")
    upper_floor = _SupportCandidate(8, (0.0, 0.0, 3.1), 31, "open")
    layers = _group_support_candidates([floor, upper_floor], tolerance=0.025)
    assert len(layers) == 2
    assert {_preferred_support_candidate(layer).owner_id for layer in layers} == {30, 31}


def test_span_field_erosion_rejects_sample_beside_blocked_column():
    opts = SurfaceDetectionOptions(cell_size_m=0.15, agent_radius_m=0.22)
    stats = SurfaceDetectionStats()
    spans = {
        (0, 0): [_span(0, 0)],
        (1, 0): [_span(1, 0, blocked=True)],
        (-1, 0): [_span(-1, 0)],
        (0, 1): [_span(0, 1)],
        (0, -1): [_span(0, -1)],
        (1, 1): [_span(1, 1)],
        (1, -1): [_span(1, -1)],
        (-1, 1): [_span(-1, 1)],
        (-1, -1): [_span(-1, -1)],
    }
    result = _erode_walkable_spans(spans, opts, stats)
    assert not any(sample.ix == 0 and sample.iy == 0 for sample in result)
    assert stats.heightfield_radius_rejections > 0


def test_unsampled_domain_boundary_does_not_erode_walkable_span():
    opts = SurfaceDetectionOptions(cell_size_m=0.15, agent_radius_m=0.22)
    spans = {(0, 0): [_span(0, 0)]}

    result = _erode_walkable_spans(
        spans,
        opts,
        sampled_xy={(0, 0)},
    )

    assert [(sample.ix, sample.iy) for sample in result] == [(0, 0)]


def test_sampled_empty_column_erodes_adjacent_walkable_span():
    opts = SurfaceDetectionOptions(cell_size_m=0.15, agent_radius_m=0.22)
    spans = {(0, 0): [_span(0, 0)]}

    result = _erode_walkable_spans(
        spans,
        opts,
        sampled_xy={(0, 0), (1, 0)},
    )

    assert result == []


def test_span_field_erosion_does_not_mix_storeys():
    opts = SurfaceDetectionOptions(cell_size_m=0.15, agent_radius_m=0.22)
    spans = {}
    for ix in range(-2, 3):
        for iy in range(-2, 3):
            spans[(ix, iy)] = [_span(ix, iy, 3.0, owner_id=2)]
    spans[(1, 0)].append(_span(1, 0, 0.0, blocked=True, owner_id=3))

    result = _erode_walkable_spans(spans, opts)
    assert any(sample.ix == 0 and sample.iy == 0 and sample.position[2] == 3.0 for sample in result)


def test_span_field_preserves_normal_stair_rise():
    opts = SurfaceDetectionOptions(cell_size_m=0.15, agent_radius_m=0.10, max_climb_m=0.24)
    stair = _span(0, 0, 0.0, terrain="stair")
    next_tread = _span(1, 0, 0.18, terrain="stair")
    assert _field_spans_compatible(stair, next_tread, opts)


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


def test_three_clearance_samples_create_one_partial_quad_triangle():
    opts = SurfaceDetectionOptions(
        cell_size_m=0.30,
        max_climb_m=0.22,
        max_slope_deg=50.0,
        minimum_component_cells=1,
        minimum_component_area_m2=0.0,
    )
    domain = SurfaceSamplingDomain(
        id="door-neck",
        bounds=(0.0, 0.0, 0.0, 0.3, 0.3, 0.0),
        terrain="open",
    )
    samples = [
        _SupportSample(0, 0, (0.0, 0.0, 0.0), 1, "open"),
        _SupportSample(1, 0, (0.3, 0.0, 0.0), 1, "open"),
        _SupportSample(0, 1, (0.0, 0.3, 0.0), 1, "open"),
    ]
    cells = _triangulate_samples(domain, samples, opts)
    assert len(cells) == 1
    assert set(cells[0].vertices_m) == {sample.position for sample in samples}


def test_three_corner_quad_does_not_bridge_incompatible_vertical_jump():
    opts = SurfaceDetectionOptions(
        cell_size_m=0.30,
        max_climb_m=0.20,
        max_slope_deg=20.0,
        minimum_component_cells=1,
        minimum_component_area_m2=0.0,
    )
    domain = SurfaceSamplingDomain(
        id="ledge",
        bounds=(0.0, 0.0, 0.0, 0.3, 0.3, 1.0),
        terrain="open",
    )
    samples = [
        _SupportSample(0, 0, (0.0, 0.0, 0.0), 1, "open"),
        _SupportSample(1, 0, (0.3, 0.0, 0.0), 1, "open"),
        _SupportSample(0, 1, (0.0, 0.3, 0.8), 2, "open"),
    ]
    assert _triangulate_samples(domain, samples, opts) == []


def test_two_corner_gap_is_never_filled():
    opts = SurfaceDetectionOptions(
        cell_size_m=0.30,
        minimum_component_cells=1,
        minimum_component_area_m2=0.0,
    )
    domain = SurfaceSamplingDomain(
        id="obstacle-strip",
        bounds=(0.0, 0.0, 0.0, 0.3, 0.3, 0.0),
        terrain="open",
    )
    samples = [
        _SupportSample(0, 0, (0.0, 0.0, 0.0), 1, "open"),
        _SupportSample(1, 0, (0.3, 0.0, 0.0), 1, "open"),
    ]
    assert _triangulate_samples(domain, samples, opts) == []
