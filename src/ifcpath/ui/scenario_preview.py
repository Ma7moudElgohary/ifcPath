from __future__ import annotations

import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QPen, QPolygonF

from ..model import InavModel, Vec3
from .person_mesh import build_person_mesh
from .preview_3d import Projected3DPreview


class Scenario3DPreview(Projected3DPreview):
    """Projected 3D preview with Digital Twin scenario overlays and an agent mesh."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._blocked_portals: set[str] = set()
        self._blocked_spaces: set[str] = set()
        self._space_cost_multipliers: dict[str, float] = {}
        self._hazard_kinds: dict[str, str] = {}
        self._show_person = True
        self._person_height_m = 1.72
        self._person_triangle_count = 0

    def set_scenario(
        self,
        *,
        blocked_portals: set[str] | None = None,
        blocked_spaces: set[str] | None = None,
        space_cost_multipliers: dict[str, float] | None = None,
        hazard_kinds: dict[str, str] | None = None,
    ) -> None:
        self._blocked_portals = set(blocked_portals or ())
        self._blocked_spaces = set(blocked_spaces or ())
        self._space_cost_multipliers = dict(space_cost_multipliers or {})
        self._hazard_kinds = dict(hazard_kinds or {})
        self.redraw(fit=False)

    def set_person_visible(self, visible: bool) -> None:
        self._show_person = bool(visible)
        self.redraw(fit=False)

    @property
    def blocked_portals(self) -> set[str]:
        return set(self._blocked_portals)

    @property
    def blocked_spaces(self) -> set[str]:
        return set(self._blocked_spaces)

    @property
    def person_triangle_count(self) -> int:
        return self._person_triangle_count

    def _draw_navmesh(self, model: InavModel) -> None:
        super()._draw_navmesh(model)
        outline = QPen(QColor(255, 109, 95, 220), 0.0)
        outline.setCosmetic(True)
        for cell in model.cells:
            if self._level_id and cell.level_id != self._level_id:
                continue
            space_id = cell.space_id
            if not space_id:
                continue
            blocked = space_id in self._blocked_spaces
            multiplier = self._space_cost_multipliers.get(space_id, 1.0)
            if not blocked and multiplier <= 1.0:
                continue

            points_depth = [self._project(vertex) for vertex in cell.vertices_m]
            polygon = QPolygonF([item[0] for item in points_depth])
            depth = sum(item[1] for item in points_depth) / 3.0
            if blocked:
                fill = QColor(225, 62, 62, 118)
            else:
                # Higher dynamic route cost becomes more visually intense.
                alpha = min(150, 55 + int(math.log2(max(1.0, multiplier)) * 24))
                kind = self._hazard_kinds.get(space_id, "hazard")
                if kind == "smoke":
                    fill = QColor(175, 184, 194, alpha)
                elif kind == "fire":
                    fill = QColor(255, 91, 45, alpha)
                elif kind == "crowd":
                    fill = QColor(255, 187, 66, alpha)
                else:
                    fill = QColor(255, 130, 62, alpha)
            item = self._scene.addPolygon(polygon, outline, QBrush(fill))
            item.setZValue(900.0 - depth)

    def _draw_portals(self, model: InavModel) -> None:
        super()._draw_portals(model)
        radius = max(0.12, self._marker_radius(model) * 1.75)
        pen = QPen(QColor("#ff4f5e"), 0.0)
        pen.setCosmetic(True)
        for portal in model.portals:
            if portal.id not in self._blocked_portals:
                continue
            if self._level_id and portal.level_id and portal.level_id != self._level_id:
                continue
            point, depth = self._project(portal.position_m)
            for x1, y1, x2, y2 in (
                (point.x() - radius, point.y() - radius, point.x() + radius, point.y() + radius),
                (point.x() - radius, point.y() + radius, point.x() + radius, point.y() - radius),
            ):
                item = self._scene.addLine(x1, y1, x2, y2, pen)
                item.setZValue(6500.0 - depth)

    def _draw_route(self) -> None:
        super()._draw_route()
        self._draw_person_agent()

    def _draw_person_agent(self) -> None:
        self._person_triangle_count = 0
        if not self._show_person or self._start_point is None:
            return

        forward = (0.0, 1.0)
        for point in self._route_points[1:]:
            dx = point[0] - self._start_point[0]
            dy = point[1] - self._start_point[1]
            if math.hypot(dx, dy) > 1e-6:
                forward = (dx, dy)
                break

        triangles = build_person_mesh(
            self._start_point,
            forward,
            height_m=self._person_height_m,
        )
        self._person_triangle_count = len(triangles)
        projected = []
        for triangle in triangles:
            points_depth = [self._project(vertex) for vertex in triangle]
            polygon = QPolygonF([item[0] for item in points_depth])
            depth = sum(item[1] for item in points_depth) / 3.0
            projected.append((depth, polygon))

        pen = QPen(QColor(117, 224, 255, 230), 0.0)
        pen.setCosmetic(True)
        fill = QBrush(QColor(53, 161, 223, 215))
        for depth, polygon in sorted(projected, key=lambda item: item[0], reverse=True):
            item = self._scene.addPolygon(polygon, pen, fill)
            item.setZValue(7000.0 - depth)
