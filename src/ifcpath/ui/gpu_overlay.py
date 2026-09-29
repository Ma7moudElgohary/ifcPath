from __future__ import annotations

from array import array
from dataclasses import dataclass, field
import math

from ..model import InavModel, Vec3
from .gpu_scene import GpuSceneData, visible_cells
from .person_mesh import build_person_mesh

Color = tuple[float, float, float, float]


@dataclass(slots=True)
class GpuOverlayData:
    triangle_vertices: array = field(default_factory=lambda: array("f"))
    line_vertices: array = field(default_factory=lambda: array("f"))

    @property
    def triangle_vertex_count(self) -> int:
        return len(self.triangle_vertices) // 7

    @property
    def line_vertex_count(self) -> int:
        return len(self.line_vertices) // 7

    def extend(self, other: "GpuOverlayData") -> None:
        self.triangle_vertices.extend(other.triangle_vertices)
        self.line_vertices.extend(other.line_vertices)


def build_navigation_overlay(
    model: InavModel | None,
    scene: GpuSceneData | None,
    *,
    level_id: str | None,
    show_graph: bool,
    show_portals: bool,
    show_exits: bool,
    start: Vec3 | None,
    goal: Vec3 | None,
) -> GpuOverlayData:
    out = GpuOverlayData()
    if model is None or scene is None:
        return out

    if show_graph:
        nodes = {node.id: node for node in model.nodes}
        for edge in model.edges:
            a = nodes.get(edge.a)
            b = nodes.get(edge.b)
            if a is None or b is None:
                continue
            if level_id and not (a.level_id == level_id and b.level_id == level_id):
                continue
            _add_line(out, scene, a.position_m, b.position_m, (0.52, 0.60, 0.68, 0.75), z_offset=0.02)

    radius = _scene_marker_radius(scene)
    for portal in model.portals:
        if level_id and portal.level_id and portal.level_id != level_id:
            continue
        if portal.is_exit:
            if not show_exits:
                continue
            _add_octahedron(out, scene, portal.position_m, radius * 1.25, (0.20, 0.92, 0.56, 1.0))
        elif show_portals:
            _add_octahedron(out, scene, portal.position_m, radius, (1.0, 0.58, 0.24, 1.0))

    if start is not None:
        _add_octahedron(out, scene, start, radius * 1.45, (0.22, 0.95, 0.46, 1.0))
    if goal is not None:
        _add_octahedron(out, scene, goal, radius * 1.45, (1.0, 0.24, 0.34, 1.0))
    return out


def build_scenario_overlay(
    model: InavModel | None,
    scene: GpuSceneData | None,
    *,
    level_id: str | None,
    blocked_portals: set[str],
    blocked_spaces: set[str],
    space_cost_multipliers: dict[str, float],
    hazard_kinds: dict[str, str],
    show_person: bool,
    agent_position: Vec3 | None,
    agent_forward: tuple[float, float],
    evacuation_poses: dict[str, tuple[Vec3, tuple[float, float], str]],
    max_full_people: int = 36,
) -> GpuOverlayData:
    out = GpuOverlayData()
    if model is None or scene is None:
        return out

    for cell in visible_cells(model, level_id):
        space_id = cell.space_id
        if not space_id:
            continue
        blocked = space_id in blocked_spaces
        multiplier = space_cost_multipliers.get(space_id, 1.0)
        if not blocked and multiplier <= 1.0:
            continue
        if blocked:
            color: Color = (0.92, 0.16, 0.16, 0.48)
        else:
            kind = hazard_kinds.get(space_id, "hazard")
            alpha = min(0.66, 0.25 + 0.08 * math.log2(max(1.0, multiplier)))
            colors: dict[str, Color] = {
                "smoke": (0.62, 0.66, 0.70, alpha),
                "fire": (1.0, 0.24, 0.06, alpha),
                "crowd": (1.0, 0.62, 0.10, alpha),
            }
            color = colors.get(kind, (1.0, 0.40, 0.10, alpha))
        _add_triangle(out, scene, cell.vertices_m, color, z_offset=0.025)

    portal_by_id = {portal.id: portal for portal in model.portals}
    cross_radius = _scene_marker_radius(scene) * 1.8
    for portal_id in blocked_portals:
        portal = portal_by_id.get(portal_id)
        if portal is None:
            continue
        if level_id and portal.level_id and portal.level_id != level_id:
            continue
        x, y, z = portal.position_m
        _add_line(
            out,
            scene,
            (x - cross_radius, y - cross_radius, z + 0.05),
            (x + cross_radius, y + cross_radius, z + 0.05),
            (1.0, 0.10, 0.14, 1.0),
        )
        _add_line(
            out,
            scene,
            (x - cross_radius, y + cross_radius, z + 0.05),
            (x + cross_radius, y - cross_radius, z + 0.05),
            (1.0, 0.10, 0.14, 1.0),
        )

    if show_person and agent_position is not None and _point_visible_on_level(model, agent_position, level_id):
        _add_person(out, scene, agent_position, agent_forward, (0.20, 0.67, 0.96, 1.0))

    visible_people = [
        (agent_id, pose)
        for agent_id, pose in sorted(evacuation_poses.items())
        if pose[2] != "evacuated" and _point_visible_on_level(model, pose[0], level_id)
    ]
    marker_radius = _scene_marker_radius(scene) * 0.75
    for index, (_agent_id, (position, forward, status)) in enumerate(visible_people):
        if status == "waiting":
            color = (1.0, 0.58, 0.12, 1.0)
        elif status == "trapped":
            color = (0.95, 0.15, 0.15, 1.0)
        else:
            color = (0.18, 0.82, 0.50, 1.0)
        if index < max_full_people:
            _add_person(out, scene, position, forward, color)
        else:
            _add_octahedron(out, scene, position, marker_radius, color)
    return out


def _point_visible_on_level(model: InavModel, point: Vec3, level_id: str | None) -> bool:
    if not level_id:
        return True
    level = next((item for item in model.levels if item.id == level_id), None)
    if level is None:
        return True
    elevations = sorted(item.elevation_m for item in model.levels)
    if len(elevations) <= 1:
        return True
    index = elevations.index(level.elevation_m) if level.elevation_m in elevations else 0
    lower = -math.inf if index == 0 else (elevations[index - 1] + level.elevation_m) * 0.5
    upper = math.inf if index == len(elevations) - 1 else (level.elevation_m + elevations[index + 1]) * 0.5
    return lower <= point[2] <= upper


def _scene_marker_radius(scene: GpuSceneData) -> float:
    dx = scene.bounds_max[0] - scene.bounds_min[0]
    dy = scene.bounds_max[1] - scene.bounds_min[1]
    diagonal = math.hypot(dx, dy)
    return min(max(diagonal / 180.0, 0.07), 0.30)


def _add_person(out: GpuOverlayData, scene: GpuSceneData, position: Vec3, forward: tuple[float, float], color: Color) -> None:
    for triangle in build_person_mesh(position, forward, height_m=1.72):
        _add_triangle(out, scene, triangle, color)


def _add_octahedron(out: GpuOverlayData, scene: GpuSceneData, center: Vec3, radius: float, color: Color) -> None:
    x, y, z = center
    top = (x, y, z + radius)
    bottom = (x, y, z - radius)
    ring = (
        (x + radius, y, z),
        (x, y + radius, z),
        (x - radius, y, z),
        (x, y - radius, z),
    )
    for i in range(4):
        a = ring[i]
        b = ring[(i + 1) % 4]
        _add_triangle(out, scene, (top, a, b), color)
        _add_triangle(out, scene, (bottom, b, a), color)


def _add_triangle(
    out: GpuOverlayData,
    scene: GpuSceneData,
    triangle: tuple[Vec3, Vec3, Vec3],
    color: Color,
    *,
    z_offset: float = 0.0,
) -> None:
    for world in triangle:
        local = scene.world_to_local((world[0], world[1], world[2] + z_offset))
        out.triangle_vertices.extend((local[0], local[1], local[2], *color))


def _add_line(
    out: GpuOverlayData,
    scene: GpuSceneData,
    a: Vec3,
    b: Vec3,
    color: Color,
    *,
    z_offset: float = 0.0,
) -> None:
    for world in (a, b):
        local = scene.world_to_local((world[0], world[1], world[2] + z_offset))
        out.line_vertices.extend((local[0], local[1], local[2], *color))
