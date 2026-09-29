from __future__ import annotations

import math
from dataclasses import dataclass

from PySide6.QtCore import QPointF, Qt, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QFrame, QGraphicsScene, QGraphicsView

from ..model import InavModel, NavCell, Vec3
from .preview_geometry import PreviewGeometry, PreviewTriangle
from .viewport_navigation import (
    ViewportNavigationConfig,
    clamp_zoom_scale,
    evenly_sample,
    orbit_angles,
    wheel_zoom_factor,
)


@dataclass(slots=True)
class _PickRecord:
    cell: NavCell
    polygon: QPolygonF
    depth: float
    elevation: float


class Projected3DPreview(QGraphicsView):
    """Qt-native projected 3D viewport for BIM context and INAV QA.

    The preview remains engine-independent, but camera interaction is treated like
    a real BIM viewport: pan and zoom only transform the already-built scene,
    orbit redraws are coalesced to a bounded frame rate, and a deterministic LOD
    is used while the camera is moving. Full geometry is rebuilt only after the
    camera settles. This avoids the previous behaviour where every mouse move
    destroyed and recreated thousands of QGraphicsItems.
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
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)
        self.setOptimizationFlag(QGraphicsView.DontSavePainterState, True)
        self.setOptimizationFlag(QGraphicsView.DontAdjustForAntialiasing, True)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)

        self._navigation = ViewportNavigationConfig()
        self._interaction_redraw_timer = QTimer(self)
        self._interaction_redraw_timer.setSingleShot(True)
        self._interaction_redraw_timer.setInterval(
            self._navigation.interaction_frame_interval_ms
        )
        self._interaction_redraw_timer.timeout.connect(self._flush_interaction_redraw)
        self._settle_timer = QTimer(self)
        self._settle_timer.setSingleShot(True)
        self._settle_timer.setInterval(self._navigation.settle_delay_ms)
        self._settle_timer.timeout.connect(self._restore_full_quality)

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
        self._fit_scale = 1.0
        self._model_diagonal_xy = 20.0
        self._orbit_last: QPointF | None = None
        self._pan_last: QPointF | None = None
        self._interactive_lod = False
        self._interaction_redraw_pending = False

        self._cached_bim_triangles: list[PreviewTriangle] = []
        self._cached_cells: list[NavCell] = []
        self._cached_portals = []

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
        self._refresh_visibility_cache()
        self._reset_orbit_target()
        self.redraw(fit=True)

    def set_bim_geometry(self, geometry: PreviewGeometry | None) -> None:
        self._bim = geometry
        self._refresh_visibility_cache()
        self._reset_orbit_target()
        self.redraw(fit=True)

    def set_bim_visible(self, visible: bool) -> None:
        self._show_bim = visible
        self.redraw(fit=False)

    def set_level(self, level_id: str | None) -> None:
        self._level_id = level_id
        self._refresh_visibility_cache()
        self._reset_orbit_target()
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
        self._restore_cursor()

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

    @property
    def orbit_angles(self) -> tuple[float, float]:
        return self._yaw_deg, self._pitch_deg

    @property
    def orbit_target(self) -> Vec3:
        return self._center

    def fit_content(self) -> None:
        bounds = self._scene.itemsBoundingRect()
        if bounds.isNull() or bounds.width() <= 0 or bounds.height() <= 0:
            return

        span = max(bounds.width(), bounds.height(), 1.0)
        pan_margin = span * 1.5
        self._scene.setSceneRect(bounds.adjusted(-pan_margin, -pan_margin, pan_margin, pan_margin))
        padding = max(0.8, span * 0.025)
        self.fitInView(
            bounds.adjusted(-padding, -padding, padding, padding),
            Qt.KeepAspectRatio,
        )
        self._fit_scale = max(1e-12, abs(self.transform().m11()))

    def reset_view(self) -> None:
        self._reset_orbit_target()
        self.redraw(fit=True)

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

    # ---------- visibility/cache ----------

    def _refresh_visibility_cache(self) -> None:
        model = self._model
        if model is None:
            self._cached_cells = []
            self._cached_portals = []
        else:
            self._cached_cells = [
                cell
                for cell in model.cells
                if not self._level_id or cell.level_id == self._level_id
            ]
            self._cached_portals = [
                portal
                for portal in model.portals
                if not self._level_id
                or not portal.level_id
                or portal.level_id == self._level_id
            ]

        if self._bim is None:
            self._cached_bim_triangles = []
        else:
            self._cached_bim_triangles = [
                triangle
                for triangle in self._bim.triangles
                if not self._level_id
                or not triangle.level_id
                or triangle.level_id == self._level_id
            ]

        self._update_model_size()

    def _bim_triangles_for_render(self):
        if not self._interactive_lod:
            return self._cached_bim_triangles
        return evenly_sample(
            self._cached_bim_triangles,
            self._navigation.interactive_bim_triangle_limit,
        )

    def _cells_for_render(self):
        if not self._interactive_lod:
            return self._cached_cells
        return evenly_sample(
            self._cached_cells,
            self._navigation.interactive_nav_cell_limit,
        )

    def _portals_for_render(self):
        if not self._interactive_lod:
            return self._cached_portals
        return evenly_sample(
            self._cached_portals,
            self._navigation.interactive_portal_limit,
        )

    # ---------- rendering ----------

    def redraw(self, *, fit: bool, interactive: bool = False) -> None:
        self._interactive_lod = bool(interactive)
        self.setRenderHint(QPainter.Antialiasing, not self._interactive_lod)
        self._scene.clear()
        self._pick_records.clear()
        model = self._model
        if model is None:
            self._empty_message()
            return

        if self._show_bim and self._bim is not None:
            self._draw_bim(self._bim)
        if self._show_navmesh:
            self._draw_navmesh(model)
        if self._show_graph and not self._interactive_lod:
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
        del geometry  # visible triangles are cached by level
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

        # QGraphicsScene already depth-sorts by Z value, so sorting thousands of
        # polygons in Python before insertion was redundant and expensive.
        for triangle in self._bim_triangles_for_render():
            points_depth = [self._project(vertex) for vertex in triangle.vertices_m]
            polygon = QPolygonF([item[0] for item in points_depth])
            depth = sum(item[1] for item in points_depth) / 3.0
            color = palette.get(triangle.category, QColor(120, 132, 145, 65))
            item = self._scene.addPolygon(polygon, pen, QBrush(color))
            item.setZValue(-1000.0 - depth)

    def _draw_navmesh(self, model: InavModel) -> None:
        del model
        pen = QPen(QColor(73, 164, 255, 190), 0.0)
        pen.setCosmetic(True)
        for cell in self._cells_for_render():
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
        for portal in self._portals_for_render():
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

    # ---------- camera math ----------

    def _iter_visible_world_points(self):
        for cell in self._cached_cells:
            yield from cell.vertices_m
        for triangle in self._cached_bim_triangles:
            yield from triangle.vertices_m
        if not self._cached_cells and not self._cached_bim_triangles and self._model is not None:
            for node in self._model.nodes:
                if not self._level_id or node.level_id == self._level_id:
                    yield node.position_m

    def _reset_orbit_target(self) -> None:
        points = self._iter_visible_world_points()
        first = next(points, None)
        if first is None:
            self._center = (0.0, 0.0, 0.0)
            return

        mins = [first[0], first[1], first[2]]
        maxs = [first[0], first[1], first[2]]
        for point in points:
            for axis in range(3):
                mins[axis] = min(mins[axis], point[axis])
                maxs[axis] = max(maxs[axis], point[axis])
        # Bounding-box centre is stable regardless of mesh tessellation density.
        self._center = tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))  # type: ignore[assignment]

    def _update_model_size(self) -> None:
        xs: list[float] = []
        ys: list[float] = []
        for cell in self._cached_cells:
            for point in cell.vertices_m:
                xs.append(point[0])
                ys.append(point[1])
        if not xs and self._model is not None:
            for node in self._model.nodes:
                if self._level_id and node.level_id != self._level_id:
                    continue
                xs.append(node.position_m[0])
                ys.append(node.position_m[1])
        if not xs:
            self._model_diagonal_xy = 20.0
            return
        self._model_diagonal_xy = max(
            1.0,
            math.hypot(max(xs) - min(xs), max(ys) - min(ys)),
        )

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
        del model
        return min(max(self._model_diagonal_xy / 180.0, 0.07), 0.30)

    def _route_marker_radius(self) -> float:
        if self._model is None:
            return 0.15
        return max(0.12, self._marker_radius(self._model) * 1.5)

    # ---------- interaction ----------

    def _restore_cursor(self) -> None:
        if self._pick_mode:
            self.viewport().setCursor(Qt.CrossCursor)
        else:
            self.viewport().setCursor(Qt.ArrowCursor)

    def _begin_orbit(self, position: QPointF) -> None:
        self._settle_timer.stop()
        self._orbit_last = position
        self._interactive_lod = True
        self.viewport().setCursor(Qt.ClosedHandCursor)

    def _end_orbit(self) -> None:
        self._orbit_last = None
        self._restore_cursor()
        self._settle_timer.start()

    def _begin_pan(self, position: QPointF) -> None:
        self._pan_last = position
        self.setRenderHint(QPainter.Antialiasing, False)
        self.viewport().setCursor(Qt.ClosedHandCursor)

    def _end_pan(self) -> None:
        self._pan_last = None
        self.setRenderHint(QPainter.Antialiasing, True)
        self._restore_cursor()
        self.viewport().update()

    def _request_interaction_redraw(self) -> None:
        self._interaction_redraw_pending = True
        if not self._interaction_redraw_timer.isActive():
            self._interaction_redraw_timer.start()

    def _flush_interaction_redraw(self) -> None:
        if not self._interaction_redraw_pending:
            return
        self._interaction_redraw_pending = False
        self.redraw(fit=False, interactive=True)

    def _restore_full_quality(self) -> None:
        self._interaction_redraw_pending = False
        self._interaction_redraw_timer.stop()
        if self._model is not None:
            self.redraw(fit=False, interactive=False)

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt override
        position = event.position()
        modifiers = event.modifiers()

        if event.button() == Qt.MiddleButton:
            self._begin_pan(position)
            event.accept()
            return

        if event.button() == Qt.RightButton:
            if self._view_mode == "3d" and not (modifiers & Qt.ShiftModifier):
                self._begin_orbit(position)
            else:
                self._begin_pan(position)
            event.accept()
            return

        if (
            event.button() == Qt.LeftButton
            and self._view_mode == "3d"
            and (modifiers & Qt.AltModifier)
        ):
            self._begin_orbit(position)
            event.accept()
            return

        if event.button() == Qt.LeftButton and self._pick_mode:
            scene_point = self.mapToScene(position.toPoint())
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
        position = event.position()

        if self._pan_last is not None:
            delta = position - self._pan_last
            self._pan_last = position
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - round(delta.x())
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - round(delta.y())
            )
            event.accept()
            return

        if self._orbit_last is not None and self._view_mode == "3d":
            delta = position - self._orbit_last
            self._orbit_last = position
            self._yaw_deg, self._pitch_deg = orbit_angles(
                self._yaw_deg,
                self._pitch_deg,
                delta.x(),
                delta.y(),
                self._navigation,
            )
            self._request_interaction_redraw()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt override
        if self._pan_last is not None and event.button() in {Qt.MiddleButton, Qt.RightButton}:
            self._end_pan()
            event.accept()
            return
        if self._orbit_last is not None and event.button() in {Qt.RightButton, Qt.LeftButton}:
            self._end_orbit()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802 - Qt override
        # BIM viewers are much easier to orbit when the target follows the area
        # the user is inspecting. Navmesh picking gives us an exact world pivot.
        if event.button() == Qt.LeftButton:
            scene_point = self.mapToScene(event.position().toPoint())
            picked = self.pick_scene_point(scene_point)
            if picked is not None:
                world, _space_id, _level_id = picked
                self._center = world
                self.redraw(fit=False)
                self.centerOn(0.0, 0.0)
                current = abs(self.transform().m11())
                desired = max(current, self._fit_scale * 2.2)
                factor, _ = clamp_zoom_scale(
                    current,
                    desired / max(current, 1e-12),
                    self._fit_scale,
                    self._navigation,
                )
                self.scale(factor, factor)
                event.accept()
                return

        self.reset_view()
        event.accept()

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt override
        precision = 0.35 if event.modifiers() & Qt.ControlModifier else 1.0
        requested = wheel_zoom_factor(
            angle_delta_y=event.angleDelta().y(),
            pixel_delta_y=event.pixelDelta().y(),
            config=self._navigation,
            precision_scale=precision,
        )
        current = abs(self.transform().m11())
        factor, _target = clamp_zoom_scale(
            current,
            requested,
            self._fit_scale,
            self._navigation,
        )
        if abs(factor - 1.0) > 1e-9:
            self.scale(factor, factor)
        event.accept()

    def keyPressEvent(self, event) -> None:  # noqa: N802 - Qt override
        if event.key() in {Qt.Key_F, Qt.Key_Home}:
            self.reset_view()
            event.accept()
            return
        if event.key() == Qt.Key_Escape and self._pick_mode:
            self.set_pick_mode(None)
            event.accept()
            return
        super().keyPressEvent(event)

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
