from __future__ import annotations

import hashlib
import math
import re
import sys
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .exporter import save_inav
from .hierarchical_routing import find_hierarchical_path
from .ifc_loader import BuildOptions, build_from_ifc
from .surface_funnel import find_surface_funnel_route
from .validation import validate_model
from .vertical_surface import find_surface_vertical_transfer

try:  # resource is unavailable on Windows, where qualification may still be run manually.
    import resource
except ImportError:  # pragma: no cover - exercised on Windows
    resource = None


_SCHEMA_RE = re.compile(rb"FILE_SCHEMA\s*\(\s*\(\s*'([^']+)'", re.IGNORECASE)
_VERTICAL_KINDS = {"stair", "ramp", "elevator", "escalator"}
_SURFACE_VERTICAL_KINDS = {"stair", "ramp", "escalator", "vertical"}


@dataclass(slots=True)
class CorpusCaseResult:
    case_id: str
    name: str
    expected_outcome: str
    status: str = "error"
    stage: str = "initializing"
    schema_expected: str | None = None
    schema_actual: str | None = None
    source_size_bytes: int = 0
    build_seconds: float | None = None
    validate_seconds: float | None = None
    route_seconds: float | None = None
    peak_memory_mb: float | None = None
    counts: dict[str, int | float | bool] = field(default_factory=dict)
    validation_stats: dict[str, int | float | bool] = field(default_factory=dict)
    issues: list[dict[str, Any]] = field(default_factory=list)
    routing: dict[str, Any] = field(default_factory=dict)
    categories: list[str] = field(default_factory=list)
    expectation_failures: list[str] = field(default_factory=list)
    exception_type: str | None = None
    exception_message: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def read_ifc_schema(path: str | Path) -> str | None:
    with Path(path).open("rb") as stream:
        header = stream.read(512 * 1024)
    match = _SCHEMA_RE.search(header)
    return match.group(1).decode("ascii", errors="replace") if match else None


def git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def verify_download(data: bytes, source: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    expected_size = source.get("size_bytes")
    if expected_size is not None and len(data) != int(expected_size):
        failures.append(f"size {len(data)} != expected {expected_size}")
    expected_sha256 = source.get("sha256")
    if expected_sha256:
        actual = hashlib.sha256(data).hexdigest()
        if actual.lower() != str(expected_sha256).lower():
            failures.append(f"sha256 {actual} != expected {expected_sha256}")
    expected_blob = source.get("git_blob_sha1")
    if expected_blob:
        actual = git_blob_sha1(data)
        if actual.lower() != str(expected_blob).lower():
            failures.append(f"git blob sha1 {actual} != expected {expected_blob}")
    return failures


def qualify_case(case: dict[str, Any], ifc_path: str | Path, defaults: dict[str, Any] | None = None) -> CorpusCaseResult:
    defaults = defaults or {}
    path = Path(ifc_path)
    result = CorpusCaseResult(
        case_id=case["id"],
        name=case.get("name", case["id"]),
        expected_outcome=case.get("expected_outcome", "PASS-PARTIAL"),
        schema_expected=case.get("schema"),
        source_size_bytes=path.stat().st_size,
    )
    try:
        result.stage = "schema"
        result.schema_actual = read_ifc_schema(path)

        # Qualification must use the same portable-semantic preparation as the
        # CLI/browser IFC->INAV path. save_inav() repairs door sides, recovers
        # conservative open boundaries, derives surface vertical transitions,
        # binds surface portals and classifies egress domains before validation.
        result.stage = "build"
        options = dict(defaults.get("build_options", {}))
        options.update(case.get("build_options", {}))
        started = time.perf_counter()
        model = build_from_ifc(path, BuildOptions(**options))
        with tempfile.TemporaryDirectory(prefix="ifcpath-case-") as temp_name:
            save_inav(model, Path(temp_name) / "qualified.inav")
        result.build_seconds = time.perf_counter() - started

        result.stage = "validate"
        started = time.perf_counter()
        report = validate_model(model)
        result.validate_seconds = time.perf_counter() - started
        result.validation_stats = dict(report.stats)
        result.issues = [asdict(issue) for issue in report.issues]
        result.counts = _model_counts(model, report.stats, report.issues)

        result.stage = "route"
        started = time.perf_counter()
        result.routing = _route_metrics(model)
        result.route_seconds = time.perf_counter() - started

        result.stage = "expectations"
        result.expectation_failures = evaluate_expectations(case, result)
        result.categories = classify_result(result)
        result.status = "pass" if not result.expectation_failures else "fail"
        result.stage = "complete"
    except Exception as exc:  # case isolation happens in the parent process
        result.exception_type = type(exc).__name__
        result.exception_message = str(exc)
        result.categories = classify_result(result)
        if result.expected_outcome == "EXPECTED-REJECT":
            result.status = "pass"
            result.stage = "complete"
        else:
            result.status = "error"
    finally:
        result.peak_memory_mb = _peak_memory_mb()
    return result


def evaluate_expectations(case: dict[str, Any], result: CorpusCaseResult) -> list[str]:
    failures: list[str] = []
    expected = case.get("expected_outcome", "PASS-PARTIAL")
    if expected == "EXPECTED-REJECT":
        if result.exception_type is None:
            failures.append("expected deterministic rejection but build completed")
        return failures

    if result.exception_type:
        failures.append(f"unexpected {result.stage} exception: {result.exception_type}: {result.exception_message}")
        return failures

    if result.schema_expected and result.schema_actual:
        if _normalize_schema(result.schema_expected) != _normalize_schema(result.schema_actual):
            failures.append(f"schema {result.schema_actual} != expected {result.schema_expected}")

    expect = case.get("expect", {})
    counts = result.counts
    for key in ("levels", "spaces", "portals", "exits", "cells", "stair_cells", "ramp_cells", "elevators"):
        minimum = expect.get(f"min_{key}")
        if minimum is not None and int(counts.get(key, 0)) < int(minimum):
            failures.append(f"{key}={counts.get(key, 0)} < expected minimum {minimum}")

    navigation_ready = bool(result.validation_stats.get("surface_navigation_ready", False))
    if expected == "PASS-NAV" and not navigation_ready:
        failures.append("PASS-NAV case is not surface navigation ready")
    if "navigation_ready" in expect and navigation_ready != bool(expect["navigation_ready"]):
        failures.append(f"navigation_ready={navigation_ready} != expected {bool(expect['navigation_ready'])}")

    if expect.get("require_multilevel_route") and not result.routing.get("multilevel_route_exists", False):
        failures.append("expected a representative multilevel route")

    return failures


def classify_result(result: CorpusCaseResult) -> list[str]:
    categories: set[str] = set()
    message = (result.exception_message or "").lower()
    stage = result.stage
    if result.exception_type:
        if stage == "schema" or any(token in message for token in ("schema", "parse", "step")):
            categories.add("A_IFC_PARSING_SCHEMA")
        elif any(token in message for token in ("triang", "polygon", "cdt", "self-intersect")):
            categories.add("C_FLOOR_POLYGON_CDT")
        elif any(token in message for token in ("shape", "geometry", "brep", "mesh")):
            categories.add("B_GEOMETRY_EXTRACTION")
        else:
            categories.add("B_GEOMETRY_EXTRACTION" if stage == "build" else "N_UNSUPPORTED_OR_UNKNOWN")

    codes = {str(issue.get("code", "")) for issue in result.issues}
    if any("PORTAL" in code or "DOOR" in code for code in codes):
        categories.add("E_DOOR_PORTAL_BINDING")
    if any("UNAUTHORIZED_SPACE_CROSSING" in code for code in codes):
        categories.add("I_SURFACE_SEAM_STITCH")
    if any("SPACE" in code and "SURFACE" in code for code in codes):
        categories.add("D_SPACE_SEMANTICS")
    if any("VERTICAL" in code for code in codes):
        categories.add("G_STAIR_RAMP_RECONSTRUCTION")
    if any("EXIT" in code or "CROSS_LEVEL" in code for code in codes):
        categories.add("K_READINESS_CLASSIFICATION")
    if result.routing.get("representative_route_attempted") and not result.routing.get("representative_route_exists"):
        categories.add("J_ROUTE_FUNNEL")
    if result.routing.get("multilevel_route_attempted") and not result.routing.get("multilevel_route_exists"):
        categories.add("G_STAIR_RAMP_RECONSTRUCTION")
    if result.expectation_failures:
        categories.add("K_READINESS_CLASSIFICATION")
    return sorted(categories)


def _model_counts(model, stats: dict[str, Any], issues) -> dict[str, int | float | bool]:
    transition_kinds = [transition.kind for transition in model.transitions]
    return {
        "levels": len(model.levels),
        "spaces": len(model.spaces),
        "portals": len(model.portals),
        "exits": sum(portal.is_exit for portal in model.portals),
        "cells": len(model.cells),
        "stair_cells": sum(cell.terrain == "stair" for cell in model.cells),
        "ramp_cells": sum(cell.terrain == "ramp" for cell in model.cells),
        "elevators": sum(kind == "elevator" for kind in transition_kinds),
        "semantic_transitions": len(model.transitions),
        "vertical_transitions": sum(kind in _VERTICAL_KINDS for kind in transition_kinds),
        "open_boundaries": sum(portal.kind == "open_boundary" for portal in model.portals),
        "surface_components": int(stats.get("surface_component_count", 0)),
        "surfaced_spaces": int(stats.get("surface_space_count", 0)),
        "reachable_spaces": int(stats.get("surface_exit_reachable_spaces", 0)),
        "missing_portal_sides": int(stats.get("surface_portal_side_failures", 0)),
        "unauthorized_crossings": sum(issue.code == "SURFACE_UNAUTHORIZED_SPACE_CROSSING" for issue in issues),
        "vertical_component_count": int(stats.get("surface_semantic_component_count", 0)),
        "navigation_ready": bool(stats.get("surface_navigation_ready", False)),
    }


def _route_metrics(model) -> dict[str, Any]:
    metrics: dict[str, Any] = {
        "representative_route_attempted": False,
        "representative_route_exists": False,
        "multilevel_route_attempted": False,
        "multilevel_route_exists": False,
        "surface_vertical_transfer_exists": False,
    }

    by_space: dict[str, list[Any]] = {}
    for cell in model.cells:
        if cell.space_id and cell.terrain == "open":
            by_space.setdefault(cell.space_id, []).append(cell)
    candidates = sorted(by_space.items(), key=lambda item: len(item[1]), reverse=True)
    for space_id, cells in candidates:
        if len(cells) < 2:
            continue
        centroids = [_centroid(cell) for cell in cells]
        start, goal = _approximate_diameter_pair(centroids)
        direct = math.dist(start, goal)
        if direct <= 1e-6:
            continue
        metrics["representative_route_attempted"] = True
        route = find_surface_funnel_route(cells, start, goal)
        if route is not None:
            metrics.update({
                "representative_route_exists": True,
                "representative_space_id": space_id,
                "representative_route_length_m": route.length_m,
                "representative_direct_distance_m": direct,
                "representative_stretch_ratio": route.length_m / direct,
                "representative_waypoint_count": len(route.points),
            })
        break

    # Prefer authored/recovered semantic vertical transfers over arbitrary
    # lowest/highest-level endpoints. A building may contain foundation, roof,
    # service or disconnected site levels that are not part of an occupant
    # circulation domain; choosing those extremes creates a false failure.
    surface_transitions = [
        transition
        for transition in model.transitions
        if transition.source == "surface_vertical_touch"
        and transition.kind in _SURFACE_VERTICAL_KINDS
    ]
    if surface_transitions:
        metrics["multilevel_route_attempted"] = True
    for transition in surface_transitions:
        transfer = find_surface_vertical_transfer(model, transition)
        if transfer is None or len(transfer.points) < 2:
            continue
        rise = abs(transfer.points[-1][2] - transfer.points[0][2])
        metrics.update({
            "surface_vertical_transfer_exists": True,
            "surface_vertical_transition_id": transition.id,
            "surface_vertical_kind": transition.kind,
            "surface_vertical_length_m": transfer.length_m,
            "surface_vertical_rise_m": rise,
            "surface_vertical_cell_count": len(transfer.cell_ids),
        })
        route = find_hierarchical_path(model, transfer.points[0], transfer.points[-1])
        if route is not None:
            _record_multilevel_route(metrics, route, transfer.points[0], transfer.points[-1], "surface_vertical_touch")
            return metrics

    # Explicit resources such as elevators may not have a walkable vertical
    # surface. Try endpoints from the transition's actual from/to spaces before
    # falling back to generic level pairs.
    cross_level = [
        transition for transition in model.transitions
        if transition.from_level_id
        and transition.to_level_id
        and transition.from_level_id != transition.to_level_id
        and transition.from_space_id
        and transition.to_space_id
    ]
    if cross_level:
        metrics["multilevel_route_attempted"] = True
    for transition in cross_level:
        start_cell = _representative_space_cell(by_space, transition.from_space_id)
        goal_cell = _representative_space_cell(by_space, transition.to_space_id)
        if start_cell is None or goal_cell is None:
            continue
        start = _centroid(start_cell)
        goal = _centroid(goal_cell)
        route = find_hierarchical_path(model, start, goal)
        if route is not None:
            _record_multilevel_route(metrics, route, start, goal, "semantic_transition")
            return metrics

    # Last-resort diagnostic for models whose vertical semantics have not yet
    # been recovered. Keep this bounded: one representative cell per surfaced
    # level and at most 12 cross-level attempts.
    level_cells: dict[str, list[Any]] = {}
    for cell in model.cells:
        if cell.level_id and cell.space_id and cell.terrain == "open":
            level_cells.setdefault(cell.level_id, []).append(cell)
    levels = [level for level in model.levels if level.id in level_cells]
    levels.sort(key=lambda level: level.elevation_m)
    attempts = 0
    for left_index, low in enumerate(levels):
        for high in levels[left_index + 1:]:
            metrics["multilevel_route_attempted"] = True
            start = _centroid(max(level_cells[low.id], key=lambda cell: _triangle_area_xy(cell.vertices_m)))
            goal = _centroid(max(level_cells[high.id], key=lambda cell: _triangle_area_xy(cell.vertices_m)))
            route = find_hierarchical_path(model, start, goal)
            attempts += 1
            if route is not None:
                _record_multilevel_route(metrics, route, start, goal, "level_pair_fallback")
                return metrics
            if attempts >= 12:
                return metrics
    return metrics


def _record_multilevel_route(metrics: dict[str, Any], route, start, goal, source: str) -> None:
    metrics.update({
        "multilevel_route_exists": True,
        "multilevel_route_source": source,
        "multilevel_route_length_m": route.length_m,
        "multilevel_weighted_cost": route.weighted_cost,
        "multilevel_transition_ids": list(route.transition_ids),
        "multilevel_transition_count": len(route.transition_ids),
        "multilevel_segment_kinds": [segment.kind for segment in route.segments],
        "multilevel_vertical_rise_m": abs(goal[2] - start[2]),
    })


def _representative_space_cell(by_space: dict[str, list[Any]], space_id: str):
    cells = by_space.get(space_id, ())
    if not cells:
        return None
    return max(cells, key=lambda cell: _triangle_area_xy(cell.vertices_m))


def _centroid(cell) -> tuple[float, float, float]:
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )


def _approximate_diameter_pair(points: list[tuple[float, float, float]]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    # Two linear farthest-point sweeps avoid an O(n^2) corpus benchmark on
    # large spaces while still choosing a useful long representative route.
    seed = points[0]
    first = max(points, key=lambda point: math.dist(seed, point))
    second = max(points, key=lambda point: math.dist(first, point))
    return first, second


def _triangle_area_xy(vertices) -> float:
    a, b, c = vertices
    return abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) * 0.5


def _peak_memory_mb() -> float | None:
    if resource is None:
        return None
    usage = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    # Linux reports KiB; macOS reports bytes.
    return usage / (1024.0 * 1024.0) if sys.platform == "darwin" else usage / 1024.0


def _normalize_schema(value: str) -> str:
    return re.sub(r"[^A-Z0-9]", "", value.upper())