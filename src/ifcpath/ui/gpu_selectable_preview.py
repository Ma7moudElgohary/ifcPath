from __future__ import annotations

from array import array
import math

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QMouseEvent

from .gpu_selection import BimSelectionIndex, ElementSelectionHit
from .gpu_streaming_preview import GpuStreamingScenarioPreview

try:
    import wgpu
except Exception:  # pragma: no cover - parent backend already handles availability
    wgpu = None


class GpuSelectableScenarioPreview(GpuStreamingScenarioPreview):
    """Streaming WebGPU viewport with IFC element selection.

    Selection is intentionally separated from navigation picking. Start/goal
    mode keeps using authoritative CDT cells; a normal left click uses a BIM BVH
    and returns the IFC GlobalId + metadata. The selected object's bounds are
    drawn through the existing small dynamic line buffer while the static BIM
    resident set remains memory-budgeted.
    """

    elementSelected = Signal(object)
    selectionCleared = Signal()

    def __init__(self, parent=None) -> None:
        self._selection_index = BimSelectionIndex(None)
        self._selected_hit: ElementSelectionHit | None = None
        self._left_press: QPointF | None = None
        super().__init__(parent)

    @property
    def selected_guid(self) -> str | None:
        return self._selected_hit.guid if self._selected_hit else None

    @property
    def selection_element_count(self) -> int:
        return self._selection_index.element_count

    def set_bim_geometry(self, geometry) -> None:
        super().set_bim_geometry(geometry)
        self._rebuild_selection_index(clear=True)

    def set_level(self, level_id: str | None) -> None:
        super().set_level(level_id)
        self._rebuild_selection_index(clear=True)

    def clear_element_selection(self) -> None:
        if self._selected_hit is None:
            return
        self._selected_hit = None
        self._upload_route()
        self.request_draw()
        self.selectionCleared.emit()

    def select_element_at(self, x_px: float, y_px: float) -> ElementSelectionHit | None:
        scene = self._scene_data
        if scene is None or not self._show_bim:
            self.clear_element_selection()
            return None
        width, height = self._canvas_size()
        local_origin, direction = self._camera.screen_ray(x_px, y_px, width, height)
        world_origin = scene.local_to_world(local_origin)
        hit = self._selection_index.pick(world_origin, direction)
        self._selected_hit = hit
        self._upload_route()
        self.request_draw()
        if hit is None:
            self.selectionCleared.emit()
        else:
            self.elementSelected.emit(hit)
        return hit

    def _rebuild_selection_index(self, *, clear: bool) -> None:
        self._selection_index = BimSelectionIndex(self._bim, level_id=self._level_id)
        if clear:
            self._selected_hit = None
            self.selectionCleared.emit()
        self._upload_route()

    def _upload_route(self) -> None:
        """Upload route + selected-element bounds as one tiny line buffer."""
        scene = self._scene_data
        self._route_buffer = None
        self._route_vertex_count = 0
        if scene is None or wgpu is None:
            return

        vertices = array("f")
        route_color = (0.20, 0.95, 1.0, 1.0)
        selection_color = (1.0, 0.86, 0.12, 1.0)

        for a, b in zip(self._route_points, self._route_points[1:]):
            for world in (a, b):
                point = scene.world_to_local(world)
                vertices.extend((point[0], point[1], point[2] + 0.035, *route_color))

        bounds = self._selection_index.bounds_for_guid(self.selected_guid)
        if bounds is not None:
            bmin, bmax = bounds
            corners = [
                (bmin[0], bmin[1], bmin[2]),
                (bmax[0], bmin[1], bmin[2]),
                (bmax[0], bmax[1], bmin[2]),
                (bmin[0], bmax[1], bmin[2]),
                (bmin[0], bmin[1], bmax[2]),
                (bmax[0], bmin[1], bmax[2]),
                (bmax[0], bmax[1], bmax[2]),
                (bmin[0], bmax[1], bmax[2]),
            ]
            edges = (
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            )
            for a_idx, b_idx in edges:
                for world in (corners[a_idx], corners[b_idx]):
                    point = scene.world_to_local(world)
                    vertices.extend((point[0], point[1], point[2], *selection_color))

        if not vertices:
            return
        self._route_vertex_count = len(vertices) // 7
        self._route_buffer = self._device.create_buffer_with_data(
            data=vertices,
            usage=wgpu.BufferUsage.VERTEX,
        )

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        if (
            event.button() == Qt.LeftButton
            and self._pick_mode is None
            and not (event.modifiers() & Qt.AltModifier)
        ):
            self._left_press = event.position()
        else:
            self._left_press = None
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        if event.button() == Qt.LeftButton and self._left_press is not None and self._pick_mode is None:
            delta = event.position() - self._left_press
            self._left_press = None
            if math.hypot(delta.x(), delta.y()) <= 5.0:
                self.select_element_at(event.position().x(), event.position().y())
                event.accept()
                return
        self._left_press = None
        super().mouseReleaseEvent(event)
