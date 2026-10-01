from __future__ import annotations

import math

from ifcpath.model import NavCell
from ifcpath.surface_seams import _overlap_portal, stitch_clearance_aware_seams


def test_parallel_tread_edges_use_full_overlap_not_endpoint_contact():
    candidate = _overlap_portal(
        (0.0, 0.0, 0.0),
        (1.2, 0.0, 0.0),
        (0.0, 0.05, 0.18),
        (1.2, 0.05, 0.18),
        max_gap_m=0.25,
        max_vertical_gap_m=0.25,
        edge_clearance_m=0.20,
        parallel_tolerance_deg=12.0,
    )
    assert candidate is not None
    portal, separation = candidate

    assert math.isclose(separation, math.hypot(0.05, 0.18), abs_tol=1e-9)
    assert math.isclose(portal[0][0], 0.20, abs_tol=1e-9)
    assert math.isclose(portal[1][0], 1.00, abs_tol=1e-9)
    assert math.isclose(portal[0][1], 0.025, abs_tol=1e-9)
    assert math.isclose(portal[1][1], 0.025, abs_tol=1e-9)
    assert math.isclose(portal[0][2], 0.09, abs_tol=1e-9)
    assert math.isclose(portal[1][2], 0.09, abs_tol=1e-9)


def test_parallel_edges_without_common_width_do_not_form_fake_wide_portal():
    candidate = _overlap_portal(
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.0),
        (0.6, 0.05, 0.18),
        (1.0, 0.05, 0.18),
        max_gap_m=0.25,
        max_vertical_gap_m=0.25,
        edge_clearance_m=0.15,
        parallel_tolerance_deg=12.0,
    )
    # No projected overlap; the closest endpoints are still 0.27 m apart in 3D,
    # which is outside max_gap, so the cells must stay disconnected.
    assert candidate is None


def test_stitcher_prefers_widest_candidate_between_same_two_cells():
    # Two triangles have both a near corner and a long, almost-parallel boundary.
    # The long overlap must win even if the corner is encountered first.
    lower = NavCell(
        id="lower",
        vertices_m=((0.0, 0.0, 0.0), (1.2, 0.0, 0.0), (0.0, 0.4, 0.0)),
        terrain="stair",
    )
    upper = NavCell(
        id="upper",
        vertices_m=((0.0, 0.05, 0.18), (1.2, 0.05, 0.18), (0.0, 0.45, 0.18)),
        terrain="stair",
    )

    added = stitch_clearance_aware_seams(
        [lower, upper],
        max_gap_m=0.25,
        max_vertical_gap_m=0.25,
        edge_clearance_m=0.20,
    )

    assert added == 1
    assert upper.id in lower.neighbor_ids
    portal = lower.portals[upper.id]
    assert math.dist(portal[0], portal[1]) > 0.70
    # Portal is centred between the two tread edges rather than sitting at x=0.
    assert portal[0][0] >= 0.19
    assert portal[1][0] <= 1.01


def test_vertical_only_stitching_preserves_flat_obstacle_gap():
    left = NavCell(
        id="left",
        vertices_m=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        terrain="open",
        space_id="S",
    )
    right = NavCell(
        id="right",
        vertices_m=((0.0, 1.20, 0.0), (1.0, 1.20, 0.0), (0.0, 2.20, 0.0)),
        terrain="open",
        space_id="S",
    )

    added = stitch_clearance_aware_seams(
        [left, right],
        max_gap_m=0.25,
        max_vertical_gap_m=0.25,
        edge_clearance_m=0.05,
        vertical_only=True,
    )

    assert added == 0
    assert left.neighbor_ids == []
    assert right.neighbor_ids == []


def test_vertical_only_stitching_can_bridge_stair_to_landing_gap():
    landing = NavCell(
        id="landing",
        vertices_m=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        terrain="open",
        space_id="S",
    )
    stair = NavCell(
        id="stair",
        vertices_m=((0.0, 1.20, 0.18), (1.0, 1.20, 0.18), (0.0, 2.20, 0.40)),
        terrain="stair",
    )

    added = stitch_clearance_aware_seams(
        [landing, stair],
        max_gap_m=0.30,
        max_vertical_gap_m=0.25,
        edge_clearance_m=0.05,
        vertical_only=True,
    )

    assert added == 1
    assert stair.id in landing.neighbor_ids
    assert landing.id in stair.neighbor_ids
