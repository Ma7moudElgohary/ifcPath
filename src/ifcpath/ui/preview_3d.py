from __future__ import annotations

import math
from dataclasses import dataclass

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QFrame, QGraphicsScene, QGraphicsView

from ..model import InavModel, NavCell, Vec3
from .preview_geometry import PreviewGeometry


@dataclass(slots=True)
class _PickRecord:
    cell: NavCell
    polygon: QPolygonF
    depth: float
    elevation: float


class Projected3DPreview(QGraphicsView):
    """Qt-native projected 3D viewport for BIM context and INAV QA.

    The widget deliberately avoids an additional 3D-engine dependency. IFC and
    INAV triangles are projected through a small orthographic camera, depth
    sorted, and drawn into a QGraphicsScene. Picking is performed against the
    projected CDT triangle and converted back to an exact world point by
    barycentric interpolation.
    """

    pointPicked = Signal(str, object, object, object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setFrameShape(QFrame.NoFrame)
        self.setBackgroundBrush(QColor("#0d1218"))
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)

        self._model: InavModel | None = None
        self._bim: PreviewGeometry | None = None
        self._level_id: str | None = None
        self._show_bim = True
        self._show_navmesh = True
        self._show_graph = False
        self._show_portals = True
        self._show_exits = True
        self._view_mode = "3d"

        self._yaw_deg = 35.0
        self._pitch_deg = 55.0
        self._center: Vec3 = (0.0, 0.0, 0.0)
        self._orbit_last: QPointF | None = None
        self._pick_mode: str | None = None
        self._pick_records: list[_PickRecord] = []
        self._start_point: Vec3 | None = None
        self._goal_point: Vec3 | None = None
        self._route_points: list[Vec3] = []
        self._empty_message()

    # ---------- public API compatible with the original preview ----------

    def set_model(self, model: InavModel | None) -> None:
        self._model = model
        self._start_point = None
        self._goal_point = None
        self._route_points = []
        self.redraw(fit=True)

    def set_bim_geometry(self, geometry: PreviewGeometry | None) -> None:
        self._bim = geometry
        self.redraw(fit=True)

    def set_bim_visible(self, visible: bool) -> None:
        self._show_bim = visible
        self.redraw(fit=False)

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

    def set_view_mode(self, mode: str) -> None:
        if mode not in {"3d", "top"}:
            raise ValueError(f"Unsupported preview mode: {mode}")
        self._view_mode = mode
        self.redraw(fit=True)

    def set_pick_mode(self, mode: str | None) -> None:
        if mode not in {None, "start", "goal"}:
            raise ValueError(f"Unsupported pick mode: {mode}")
        self._pick_mode = mode
        self.viewport().setCursor(Qt.CrossCursor if mode else Qt.ArrowCursor)

    def set_route(
        self,
        start: Vec3 | None,
        goal: Vec3 | None,
        points: list[Vec3] | None,
    ) -> None:
        self._start_point = start
        self._goal_point = goal
        self._route_points = list(points or [])
        self.redraw(fit=False)

    @property
    def route_points(self) -> list[Vec3]:
        return list(self._route_points)

    def fit_content(self) -> None:
        bounds = self._scene.itemsBoundingRect()
        if not bounds.isNull() and bounds.width() > 0 and bounds.height() > 0:
            self.fitInView(bounds.adjusted(-0.8, -0.8, 0.8, 0.8), Qt.KeepAspectRatio)

    def project_world(self, point: Vec3) -> tuple[QPointF, float]:
        return self._project(point)

    def pick_scene_point(self, scene_point: QPointF) -> tuple[Vec3, str | None, str | None] | None:
        candidates = [
            record
            for record in self._pick_records
            if record.polygon.containsPoint(scene_point, Qt.OddEvenFill)
        ]
        if not candidates:
            return None

        # With all storeys visible, favour the highest physical surface. On a
        # filtered level this naturally reduces to the visible CDT cell.
        record = max(candidates, key=lambda item: (item.elevation, -item.depth))
        vertices = record.cell.vertices_m
        screen = [self._project(vertex)[0] for vertex in vertices]
        weights = _barycentric_2d(scene_point, screen[0], screen[1], screen[2])
        if weights is None:
            world = tuple(sum(vertex[i] for vertex in vertices) / 3.0 for i in range(3))
        else:
            world = tuple(
                weights[0] * vertices[0][i]
                + weights[1] * vertices[1][i]
                + weights[2] * vertices[2][i]
                for i in range(3)
            )
        return (world[0], world[1], world[2]), record.cell.space_id, record.cell.level_id

    # ---------- rendering ----------

    def redraw(self, *, fit: bool) -> None:
        self._scene.clear()
        self._pick_records.clear()
        model = self._model
        if model is None:
            self._empty_message()
            return

        self._center = self._compute_center(model)

        if self._show_bim and self._bim is not None:
            self._draw_bim(self._bim)
        if self._show_navmesh:
            self._draw_navmesh(model)
        if self._show_graph:
            self._draw_graph(model)
        if self._show_portals or self._show_exits:
            self._draw_portals(model)
        self._draw_route()

        if not self._scene.items():
            self._empty_message("No geometry on this level")
            return
        if fit:
            self.fit_content()

    def _draw_bim(self, geometry: PreviewGeometry) -> None:
        palette = {
            "wall": QColor(115, 130, 146, 72),
            "slab": QColor(88, 101, 116, 62),
            "column": QColor(145, 154, 166, 92),
            "stair": QColor(128, 112, 164, 92),
            "ramp": QColor(113, 126, 164, 92),
            "door": QColor(165, 128, 82, 100),
        }
        pen = QPen(QColor(145, 158, 173, 72), 0.0)
        pen.setCosmetic(True)

        projected: list[tuple[float, QPolygonF, QColor]] = []
        for triangle in geometry.triangles:
            if self._level_id and triangle.level_id and triangle.level_id != self._level_id:
                continue
            points_depth = [self._project(vertex) for vertex in triangle.vertices_m]
            polygon = QPolygonF([item[0] for item in points_depth])
            depth = sum(item[1] for item in points_depth) / 3.0
            projected.append((depth, polygon, palette.get(triangle.category, QColor(120, 132, 145, 65))))

        for depth, polygon, color in sorted(projected, key=lambda item: item[0], reverse=True):
            item = self._scene.addPolygon(polygon, pen, QBrush(color))
            item.setZValue(-1000.0 - depth)

    def _draw_navmesh(self, model: InavModel) -> None:
        pen = QPen(QColor(73, 164, 255, 190), 0.0)
        pen.setCosmetic(True)
        for cell in model.cells:
            if self._level_id and cell.level_id != self._level_id:
                continue
            points_depth = [self._project(vertex) for vertex in cell.vertices_m]
            polygon = QPolygonF([item[0] for item in points_depth])
            depth = sum(item[1] for item in points_depth) / 3.0
            hue = abs(hash(cell.space_id or cell.id)) % 360
            fill = QColor.fromHsv(hue, 95, 225, 72)
            item = self._scene.addPolygon(polygon, pen, QBrush(fill))
            item.setZValue(100.0 - depth)
            elevation = sum(vertex[2] for vertex in cell.vertices_m) / 3.0
            self._pick_records.append(_PickRecord(cell, polygon, depth, elevation))

    def _draw_graph(self, model: InavModel) -> None:
        nodes = {node.id: node for node in model.nodes}
        pen = QPen(QColor(145, 157, 171, 150), 0.0)
        pen.setCosmetic(True)
        for edge in model.edges:
            a = nodes.get(edge.a)
            b = nodes.get(edge.b)
            if a is None or b is None:
                continue
            if self._level_id and not (a.level_id == self._level_id and b.level_id == self._level_id):
                continue
            pa, da = self._project(a.position_m)
            pb, db = self._project(b.position_m)
            item = self._scene.addLine(pa.x(), pa.y(), pb.x(), pb.y(), pen)
            item.setZValue(300.0 - (da + db) * 0.5)

    def _draw_portals(self, model: InavModel) -> None:
        radius = self._marker_radius(model)
        for portal in model.portals:
            if self._level_id and portal.level_id and portal.level_id != self._level_id:
                continue
            if portal.is_exit and not self._show_exits:
                continue
            if not portal.is_exit and not self._show_portals:
                continue
            point, depth = self._project(portal.position_m)
            color = QColor("#4be0a4") if portal.is_exit else QColor("#ffad57")
            pen = QPen(color, 0.0)
            pen.setCosmetic(True)
            item = self._scene.addEllipse(
                point.x() - radius,
                point.y() - radius,
                radius * 2.0,
                radius * 2.0,
                pen,
                QBrush(color),
            )
            item.setZValue(1500.0 - depth)

    def _draw_route(self) -> None:
        if len(self._route_points) >= 2:
            path = QPainterPath()
            first, _ = self._project(self._route_points[0])
            path.moveTo(first)
            for point in self._route_points[1:]:
                projected, _ = self._project(point)
                path.lineTo(projected)
            pen = QPen(QColor("#57f0ff"), 0.10)
            pen.setCosmetic(True)
            item = self._scene.addPath(path, pen)
            item.setZValue(5000.0)

        radius = self._route_marker_radius()
        if self._start_point is not None:
            p, _ = self._project(self._start_point)
            item = self._scene.addEllipse(
                p.x() - radius,
                p.y() - radius,
                radius * 2.0,
                radius * 2.0,
                QPen(QColor("#62e38c"), 0.0),
                QBrush(QColor("#62e38c")),
            )
            item.setZValue(6000.0)
        if self._goal_point is not None:
            p, _ = self._project(self._goal_point)
            item = self._scene.addEllipse(
                p.x() - radius,
                p.y() - radius,
                radius * 2.0,
                radius * 2.0,
                QPen(QColor("#ff6b7a"), 0.0),
                QBrush(QColor("#ff6b7a")),
            )
            item.setZValue(6000.0)

    def _compute_center(self, model: InavModel) -> Vec3:
        points: list[Vec3] = [vertex for cell in model.cells for vertex in cell.vertices_m]
        if not points:
            points = [node.position_m for node in model.nodes]
        if self._bim is not None:
            # Sampling the triangle list avoids a second huge temporary array.
            for triangle in self._bim.triangles[:: max(1, len(self._bim.triangles) // 2000 or 1)]:
                points.extend(triangle.vertices_m)
        if not points:
            return (0.0, 0.0, 0.0)
        return tuple(sum(point[i] for point in points) / len(points) for i in range(3))  # type: ignore[return-value]

    def _project(self, point: Vec3) -> tuple[QPointF, float]:
        dx = point[0] - self._center[0]
        dy = point[1] - self._center[1]
        dz = point[2] - self._center[2]

        yaw = math.radians(0.0 if self._view_mode == "top" else self._yaw_deg)
        pitch = math.radians(90.0 if self._view_mode == "top" else self._pitch_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)

        x1 = cy * dx - sy * dy
        y1 = sy * dx + cy * dy
        depth = cp * y1 - sp * dz
        vertical = sp * y1 + cp * dz
        return QPointF(x1, -vertical), depth

    def _marker_radius(self, model: InavModel) -> float:
        points = [node.position_m for node in model.nodes]
        if not points:
            return 0.12
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        diagonal = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return min(max(diagonal / 180.0, 0.07), 0.30)

    def _route_marker_radius(self) -> float:
        if self._model is None:
            return 0.15
        return max(0.12, self._marker_radius(self._model) * 1.5)

    # ---------- interaction ----------

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt override
        if event.button() == Qt.RightButton and self._view_mode == "3d":
            self._orbit_last = event.position()
            event.accept()
            return
        if event.button() == Qt.LeftButton and self._pick_mode:
            scene_point = self.mapToScene(event.position().toPoint())
            picked = self.pick_scene_point(scene_point)
            if picked is not None:
                world, space_id, level_id = picked
                mode = self._pick_mode
                self.set_pick_mode(None)
                self.pointPicked.emit(mode, world, space_id, level_id)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt override
        if self._orbit_last is not None and self._view_mode == "3d":
            delta = event.position() - self._orbit_last
            self._orbit_last = event.position()
            self._yaw_deg = (self._yaw_deg + delta.x() * 0.45) % 360.0
            self._pitch_deg = min(88.0, max(15.0, self._pitch_deg - delta.y() * 0.35))
            self.redraw(fit=False)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt override
        if event.button() == Qt.RightButton and self._orbit_last is not None:
            self._orbit_last = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802 - Qt override
        self.fit_content()
        event.accept()

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt override
        factor = 1.18 if event.angleDelta().y() > 0 else 1 / 1.18
        self.scale(factor, factor)
        event.accept()

    def _empty_message(self, text: str = "Open an IFC or INAV model to begin") -> None:
        item = self._scene.addText(text)
        item.setDefaultTextColor(QColor("#7f8ea3"))
        item.setPos(-item.boundingRect().width() / 2.0, -item.boundingRect().height() / 2.0)


def _barycentric_2d(
    point: QPointF,
    a: QPointF,
    b: QPointF,
    c: QPointF,
) -> tuple[float, float, float] | None:
    denominator = (b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y())
    if abs(denominator) <= 1e-12:
        return None
    w0 = ((b.y() - c.y()) * (point.x() - c.x()) + (c.x() - b.x()) * (point.y() - c.y())) / denominator
    w1 = ((c.y() - a.y()) * (point.x() - c.x()) + (a.x() - c.x()) * (point.y() - c.y())) / denominator
    w2 = 1.0 - w0 - w1
    return w0, w1, w2
