from __future__ import annotations

from PySide6.QtGui import QBrush, QColor, QPen, QPolygonF

from ..model import Vec3
from .person_mesh import build_person_mesh
from .scenario_preview import Scenario3DPreview


class Evacuation3DPreview(Scenario3DPreview):
    """Scenario preview extended with many live evacuation agents."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._evacuation_poses: dict[str, tuple[Vec3, tuple[float, float], str]] = {}
        self._max_person_meshes = 36
        self._interactive_person_meshes = 8

    @property
    def evacuation_agent_count(self) -> int:
        return len(self._evacuation_poses)

    def set_evacuation_agents(
        self,
        poses: dict[str, tuple[Vec3, tuple[float, float], str]] | None,
    ) -> None:
        self._evacuation_poses = dict(poses or {})
        self.redraw(fit=False, interactive=self._live_interaction_quality())

    def _draw_route(self) -> None:
        super()._draw_route()
        self._draw_evacuation_agents()

    def _draw_evacuation_agents(self) -> None:
        visible = [
            (agent_id, pose)
            for agent_id, pose in sorted(self._evacuation_poses.items())
            if pose[2] != "evacuated"
        ]
        mesh_limit = (
            self._interactive_person_meshes
            if self._interactive_lod
            else self._max_person_meshes
        )
        for index, (_agent_id, (position, forward, status)) in enumerate(visible):
            if index < mesh_limit:
                self._draw_person_mesh(position, forward, status)
            else:
                self._draw_agent_marker(position, status)

    def _draw_person_mesh(
        self,
        position: Vec3,
        forward: tuple[float, float],
        status: str,
    ) -> None:
        triangles = build_person_mesh(position, forward, height_m=1.72)
        projected = []
        for triangle in triangles:
            points_depth = [self._project(vertex) for vertex in triangle]
            polygon = QPolygonF([item[0] for item in points_depth])
            depth = sum(item[1] for item in points_depth) / 3.0
            projected.append((depth, polygon))

        if status == "waiting":
            fill = QColor(255, 184, 77, 205)
            edge = QColor(255, 216, 146, 230)
        elif status == "trapped":
            fill = QColor(238, 78, 78, 215)
            edge = QColor(255, 145, 145, 235)
        else:
            fill = QColor(84, 205, 147, 205)
            edge = QColor(160, 245, 200, 230)

        pen = QPen(edge, 0.0)
        pen.setCosmetic(True)
        brush = QBrush(fill)
        for depth, polygon in sorted(projected, key=lambda item: item[0], reverse=True):
            item = self._scene.addPolygon(polygon, pen, brush)
            item.setZValue(7200.0 - depth)

    def _draw_agent_marker(self, position: Vec3, status: str) -> None:
        point, depth = self._project(position)
        radius = 0.12
        if status == "waiting":
            color = QColor(255, 184, 77, 225)
        elif status == "trapped":
            color = QColor(238, 78, 78, 230)
        else:
            color = QColor(84, 205, 147, 225)
        item = self._scene.addEllipse(
            point.x() - radius,
            point.y() - radius,
            radius * 2.0,
            radius * 2.0,
            QPen(color, 0.0),
            QBrush(color),
        )
        item.setZValue(7200.0 - depth)
