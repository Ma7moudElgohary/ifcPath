from __future__ import annotations

import math

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen, QBrush
from PySide6.QtWidgets import QFrame, QGraphicsScene, QGraphicsView

from ..model import InavModel


class NavigationPreview(QGraphicsView):
    """Lightweight interactive top-down preview of generated INAV data."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QColor("#10151c"))
        self.setFrameShape(QFrame.NoFrame)
        self._model: InavModel | None = None
        self._level_id: str | None = None
        self._show_navmesh = True
        self._show_graph = False
        self._show_portals = True
        self._show_exits = True
        self._empty_message()

    def set_model(self, model: InavModel | None) -> None:
        self._model = model
        self.redraw(fit=True)

    def set_level(self, level_id: str | None) -> None:
        self._level_id = level_id
        self.redraw(fit=True)

    def set_layers(
        self,
        *,
        navmesh: bool,
        graph: bool,
        portals: bool,
        exits: bool,
    ) -> None:
        self._show_navmesh = navmesh
        self._show_graph = graph
        self._show_portals = portals
        self._show_exits = exits
        self.redraw(fit=False)

    def fit_content(self) -> None:
        bounds = self._scene.itemsBoundingRect()
        if not bounds.isNull() and bounds.width() > 0 and bounds.height() > 0:
            self.fitInView(bounds.adjusted(-0.5, -0.5, 0.5, 0.5), Qt.KeepAspectRatio)

    def redraw(self, *, fit: bool) -> None:
        self._scene.clear()
        model = self._model
        if model is None:
            self._empty_message()
            return

        node_by_id = {node.id: node for node in model.nodes}

        if self._show_navmesh:
            nav_pen = QPen(QColor(64, 156, 255, 185), 0.0)
            nav_pen.setCosmetic(True)
            for cell in model.cells:
                if self._level_id and cell.level_id != self._level_id:
                    continue
                points = [QPointF(v[0], -v[1]) for v in cell.vertices_m]
                if len(points) != 3:
                    continue
                path = QPainterPath(points[0])
                path.lineTo(points[1])
                path.lineTo(points[2])
                path.closeSubpath()
                hue = abs(hash(cell.space_id or cell.id)) % 360
                fill = QColor.fromHsv(hue, 90, 210, 48)
                self._scene.addPath(path, nav_pen, QBrush(fill))

        if self._show_graph:
            graph_pen = QPen(QColor(135, 148, 166, 145), 0.0)
            graph_pen.setCosmetic(True)
            for edge in model.edges:
                a = node_by_id.get(edge.a)
                b = node_by_id.get(edge.b)
                if a is None or b is None:
                    continue
                if self._level_id:
                    a_on_level = a.level_id == self._level_id
                    b_on_level = b.level_id == self._level_id
                    if not (a_on_level and b_on_level):
                        continue
                self._scene.addLine(
                    a.position_m[0],
                    -a.position_m[1],
                    b.position_m[0],
                    -b.position_m[1],
                    graph_pen,
                )

        portal_radius = self._marker_radius(model)
        for portal in model.portals:
            if self._level_id and portal.level_id and portal.level_id != self._level_id:
                continue
            if portal.is_exit and not self._show_exits:
                continue
            if not portal.is_exit and not self._show_portals:
                continue

            x, y, _ = portal.position_m
            color = QColor("#48d597") if portal.is_exit else QColor("#ffad57")
            pen = QPen(color, 0.0)
            pen.setCosmetic(True)
            self._scene.addEllipse(
                x - portal_radius,
                -y - portal_radius,
                portal_radius * 2,
                portal_radius * 2,
                pen,
                QBrush(color),
            )

        if not self._scene.items():
            self._empty_message("No navigation geometry on this level")
            return

        if fit:
            self.fit_content()

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt override
        factor = 1.18 if event.angleDelta().y() > 0 else 1 / 1.18
        self.scale(factor, factor)
        event.accept()

    def _marker_radius(self, model: InavModel) -> float:
        positions = [node.position_m for node in model.nodes]
        if not positions:
            return 0.12
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        diagonal = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return min(max(diagonal / 180.0, 0.06), 0.30)

    def _empty_message(self, text: str = "Open an IFC or INAV model to begin") -> None:
        item = self._scene.addText(text)
        item.setDefaultTextColor(QColor("#7f8ea3"))
        item.setPos(-item.boundingRect().width() / 2.0, -item.boundingRect().height() / 2.0)
