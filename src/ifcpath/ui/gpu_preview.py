from __future__ import annotations

from array import array
import math
from typing import Any

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QKeyEvent, QMouseEvent, QWheelEvent

from ..model import InavModel, Vec3
from .gpu_camera import GpuCameraState, ray_triangle_distance
from .gpu_scene import GpuSceneData, build_gpu_scene, visible_cells
from .preview_geometry import PreviewGeometry

try:
    import wgpu
    from rendercanvas.qt import QRenderWidget
except Exception as exc:  # pragma: no cover - exercised through fallback factory
    wgpu = None
    QRenderWidget = object  # type: ignore[assignment,misc]
    _GPU_IMPORT_ERROR: Exception | None = exc
else:
    _GPU_IMPORT_ERROR = None


_SHADER = """
struct Camera {
    view_proj: mat4x4<f32>,
};
@group(0) @binding(0) var<uniform> camera: Camera;

struct VertexIn {
    @location(0) position: vec3f,
    @location(1) color: vec4f,
};
struct VertexOut {
    @builtin(position) position: vec4f,
    @location(0) color: vec4f,
};

@vertex
fn vs_main(input: VertexIn) -> VertexOut {
    var out: VertexOut;
    out.position = camera.view_proj * vec4f(input.position, 1.0);
    out.color = input.color;
    return out;
}

@fragment
fn fs_main(input: VertexOut) -> @location(0) vec4f {
    return input.color;
}
"""


class GpuBackendUnavailable(RuntimeError):
    pass


class GpuBimPreview(QRenderWidget):  # type: ignore[misc]
    """Experimental GPU route-QA viewport embedded directly in PySide6.

    The WebGPU renderer owns static BIM/navmesh geometry in GPU buffers. Camera
    motion only changes a 64-byte uniform buffer; IFC geometry is not rebuilt or
    reprojected on the CPU. The class intentionally mirrors the core API of
    ``Projected3DPreview`` so it can be introduced incrementally.
    """

    pointPicked = Signal(str, object, object, object)

    def __init__(self, parent=None) -> None:
        if _GPU_IMPORT_ERROR is not None or wgpu is None:
            raise GpuBackendUnavailable(str(_GPU_IMPORT_ERROR or "WebGPU backend unavailable"))
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)

        self._model: InavModel | None = None
        self._bim: PreviewGeometry | None = None
        self._scene_data: GpuSceneData | None = None
        self._level_id: str | None = None
        self._show_bim = True
        self._show_navmesh = True
        self._show_graph = False
        self._show_portals = True
        self._show_exits = True
        self._view_mode = "3d"
        self._pick_mode: str | None = None
        self._start_point: Vec3 | None = None
        self._goal_point: Vec3 | None = None
        self._route_points: list[Vec3] = []
        self._camera = GpuCameraState()
        self._drag_mode: str | None = None
        self._drag_last = QPointF()

        self._adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
        if self._adapter is None:
            raise GpuBackendUnavailable("No WebGPU adapter was found")
        self._device = self._adapter.request_device_sync()
        self._context = self.get_wgpu_context()
        self._format = self._context.get_preferred_format(self._adapter)
        self._context.configure(device=self._device, format=self._format)
        self._camera_buffer = self._device.create_buffer(
            size=64,
            usage=wgpu.BufferUsage.UNIFORM | wgpu.BufferUsage.COPY_DST,
        )
        self._shader = self._device.create_shader_module(code=_SHADER)
        self._bind_group_layout = self._device.create_bind_group_layout(
            entries=[
                {
                    "binding": 0,
                    "visibility": wgpu.ShaderStage.VERTEX,
                    "buffer": {"type": "uniform"},
                }
            ]
        )
        self._pipeline_layout = self._device.create_pipeline_layout(bind_group_layouts=[self._bind_group_layout])
        self._bind_group = self._device.create_bind_group(
            layout=self._bind_group_layout,
            entries=[{"binding": 0, "resource": {"buffer": self._camera_buffer}}],
        )
        self._triangle_pipeline = self._create_pipeline("triangle-list")
        self._line_pipeline = self._create_pipeline("line-list")
        self._gpu_batches: list[tuple[Any, Any, int, str]] = []
        self._route_buffer = None
        self._route_vertex_count = 0
        self._depth_texture = None
        self._depth_size = (0, 0)
        self.request_draw(self._draw_frame)

    # ---------- API parity ----------

    @property
    def renderer_name(self) -> str:
        info = getattr(self._adapter, "info", {}) or {}
        device = info.get("device") or info.get("description") or "WebGPU"
        backend = info.get("backend_type") or info.get("backend") or ""
        return f"WebGPU · {device}{f' · {backend}' if backend else ''}"

    @property
    def route_points(self) -> list[Vec3]:
        return list(self._route_points)

    def set_model(self, model: InavModel | None) -> None:
        self._model = model
        self._start_point = None
        self._goal_point = None
        self._route_points = []
        self._rebuild_scene(fit=True)

    def set_bim_geometry(self, geometry: PreviewGeometry | None) -> None:
        self._bim = geometry
        self._rebuild_scene(fit=True)

    def set_bim_visible(self, visible: bool) -> None:
        if self._show_bim != bool(visible):
            self._show_bim = bool(visible)
            self._rebuild_scene(fit=False)

    def set_level(self, level_id: str | None) -> None:
        self._level_id = level_id
        self._rebuild_scene(fit=True)

    def set_layers(self, *, navmesh: bool, graph: bool, portals: bool, exits: bool) -> None:
        rebuild = self._show_navmesh != bool(navmesh)
        self._show_navmesh = bool(navmesh)
        self._show_graph = bool(graph)
        self._show_portals = bool(portals)
        self._show_exits = bool(exits)
        if rebuild:
            self._rebuild_scene(fit=False)
        else:
            self.request_draw()

    def set_view_mode(self, mode: str) -> None:
        if mode not in {"3d", "top"}:
            raise ValueError(f"Unsupported preview mode: {mode}")
        self._view_mode = mode
        if mode == "top":
            self._camera.yaw_deg = 0.0
            self._camera.pitch_deg = 87.5
        else:
            self._camera.yaw_deg = 35.0
            self._camera.pitch_deg = 38.0
        self.request_draw()

    def set_pick_mode(self, mode: str | None) -> None:
        if mode not in {None, "start", "goal"}:
            raise ValueError(f"Unsupported pick mode: {mode}")
        self._pick_mode = mode
        self.setCursor(Qt.CrossCursor if mode else Qt.ArrowCursor)

    def set_route(self, start: Vec3 | None, goal: Vec3 | None, points: list[Vec3] | None) -> None:
        self._start_point = start
        self._goal_point = goal
        self._route_points = list(points or [])
        self._upload_route()
        self.request_draw()

    def fit_content(self) -> None:
        scene = self._scene_data
        if scene is None:
            return
        local_min = scene.world_to_local(scene.bounds_min)
        local_max = scene.world_to_local(scene.bounds_max)
        self._camera.fit_bounds(local_min, local_max)
        self.request_draw()

    def project_world(self, point: Vec3) -> tuple[QPointF, float]:
        # Compatibility helper used by tests/tools. This is a perspective projection.
        scene = self._scene_data
        if scene is None:
            return QPointF(), 0.0
        local = scene.world_to_local(point)
        width, height = self._canvas_size()
        m = self._camera.view_projection(width / max(1.0, height))
        x, y, z, w = _transform_point(m, local)
        if abs(w) <= 1e-9:
            return QPointF(), z
        ndc_x, ndc_y = x / w, y / w
        return QPointF((ndc_x + 1.0) * 0.5 * width, (1.0 - ndc_y) * 0.5 * height), z / w

    def pick_view_point(self, x_px: float, y_px: float) -> tuple[Vec3, str | None, str | None] | None:
        scene = self._scene_data
        if scene is None:
            return None
        width, height = self._canvas_size()
        origin, direction = self._camera.screen_ray(x_px, y_px, width, height)
        best: tuple[float, Any] | None = None
        for cell in visible_cells(self._model, self._level_id):
            local_tri = tuple(scene.world_to_local(point) for point in cell.vertices_m)
            distance = ray_triangle_distance(origin, direction, local_tri)  # type: ignore[arg-type]
            if distance is not None and (best is None or distance < best[0]):
                best = (distance, cell)
        if best is None:
            return None
        distance, cell = best
        local_hit = (
            origin[0] + direction[0] * distance,
            origin[1] + direction[1] * distance,
            origin[2] + direction[2] * distance,
        )
        return scene.local_to_world(local_hit), cell.space_id, cell.level_id

    # ---------- GPU lifecycle ----------

    def _create_pipeline(self, topology: str):
        return self._device.create_render_pipeline(
            layout=self._pipeline_layout,
            vertex={
                "module": self._shader,
                "entry_point": "vs_main",
                "buffers": [
                    {
                        "array_stride": 28,
                        "step_mode": "vertex",
                        "attributes": [
                            {"shader_location": 0, "offset": 0, "format": "float32x3"},
                            {"shader_location": 1, "offset": 12, "format": "float32x4"},
                        ],
                    }
                ],
            },
            primitive={"topology": topology, "front_face": "ccw", "cull_mode": "none"},
            depth_stencil={
                "format": "depth24plus",
                "depth_write_enabled": True,
                "depth_compare": "less",
            },
            multisample={"count": 1},
            fragment={
                "module": self._shader,
                "entry_point": "fs_main",
                "targets": [
                    {
                        "format": self._format,
                        "blend": {
                            "color": {"src_factor": "src-alpha", "dst_factor": "one-minus-src-alpha", "operation": "add"},
                            "alpha": {"src_factor": "one", "dst_factor": "one-minus-src-alpha", "operation": "add"},
                        },
                        "write_mask": wgpu.ColorWrite.ALL,
                    }
                ],
            },
        )

    def _rebuild_scene(self, *, fit: bool) -> None:
        self._scene_data = build_gpu_scene(
            self._model,
            self._bim,
            level_id=self._level_id,
            show_bim=self._show_bim,
            show_navmesh=self._show_navmesh,
        )
        self._gpu_batches.clear()
        for batch in self._scene_data.batches:
            vertex_buffer = self._device.create_buffer_with_data(
                data=batch.vertices,
                usage=wgpu.BufferUsage.VERTEX,
            )
            index_buffer = self._device.create_buffer_with_data(
                data=batch.indices,
                usage=wgpu.BufferUsage.INDEX,
            )
            self._gpu_batches.append((vertex_buffer, index_buffer, len(batch.indices), batch.category))
        self._upload_route()
        if fit:
            self.fit_content()
        self.request_draw()

    def _upload_route(self) -> None:
        scene = self._scene_data
        self._route_buffer = None
        self._route_vertex_count = 0
        if scene is None or len(self._route_points) < 2:
            return
        vertices = array("f")
        color = (0.20, 0.95, 1.0, 1.0)
        for a, b in zip(self._route_points, self._route_points[1:]):
            for world in (a, b):
                point = scene.world_to_local(world)
                vertices.extend((point[0], point[1], point[2] + 0.035, *color))
        self._route_vertex_count = len(vertices) // 7
        self._route_buffer = self._device.create_buffer_with_data(data=vertices, usage=wgpu.BufferUsage.VERTEX)

    def _ensure_depth(self, width: int, height: int) -> Any:
        size = (max(1, width), max(1, height))
        if self._depth_texture is None or self._depth_size != size:
            self._depth_texture = self._device.create_texture(
                size=(size[0], size[1], 1),
                format="depth24plus",
                usage=wgpu.TextureUsage.RENDER_ATTACHMENT,
            )
            self._depth_size = size
        return self._depth_texture.create_view()

    def _draw_frame(self) -> None:
        try:
            current_texture = self._context.get_current_texture()
        except getattr(wgpu, "DrawCancelled", RuntimeError):
            return
        width, height = self.get_physical_size()
        width, height = max(1, int(width)), max(1, int(height))
        aspect = width / max(1.0, float(height))
        matrix = array("f", self._camera.view_projection(aspect))
        self._device.queue.write_buffer(self._camera_buffer, 0, matrix)

        encoder = self._device.create_command_encoder()
        render_pass = encoder.begin_render_pass(
            color_attachments=[
                {
                    "view": current_texture.create_view(),
                    "resolve_target": None,
                    "clear_value": (0.035, 0.055, 0.075, 1.0),
                    "load_op": "clear",
                    "store_op": "store",
                }
            ],
            depth_stencil_attachment={
                "view": self._ensure_depth(width, height),
                "depth_clear_value": 1.0,
                "depth_load_op": "clear",
                "depth_store_op": "store",
            },
        )
        render_pass.set_bind_group(0, self._bind_group)
        render_pass.set_pipeline(self._triangle_pipeline)
        for vertex_buffer, index_buffer, count, _category in self._gpu_batches:
            render_pass.set_vertex_buffer(0, vertex_buffer)
            render_pass.set_index_buffer(index_buffer, "uint32")
            render_pass.draw_indexed(count, 1, 0, 0, 0)
        if self._route_buffer is not None and self._route_vertex_count:
            render_pass.set_pipeline(self._line_pipeline)
            render_pass.set_vertex_buffer(0, self._route_buffer)
            render_pass.draw(self._route_vertex_count, 1, 0, 0)
        render_pass.end()
        self._device.queue.submit([encoder.finish()])

    # ---------- Qt interaction ----------

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        mods = event.modifiers()
        if event.button() == Qt.RightButton and (mods & Qt.ShiftModifier):
            self._drag_mode = "pan"
        elif event.button() == Qt.MiddleButton:
            self._drag_mode = "pan"
        elif event.button() == Qt.RightButton or (event.button() == Qt.LeftButton and mods & Qt.AltModifier):
            self._drag_mode = "orbit"
        elif event.button() == Qt.LeftButton and self._pick_mode:
            picked = self.pick_view_point(event.position().x(), event.position().y())
            if picked is not None:
                world, space_id, level_id = picked
                mode = self._pick_mode
                self.set_pick_mode(None)
                self.pointPicked.emit(mode, world, space_id, level_id)
                event.accept()
                return
        self._drag_last = event.position()
        if self._drag_mode:
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        if self._drag_mode:
            delta = event.position() - self._drag_last
            self._drag_last = event.position()
            if self._drag_mode == "orbit" and self._view_mode == "3d":
                self._camera.orbit(delta.x(), delta.y())
            else:
                _w, h = self._canvas_size()
                self._camera.pan(delta.x(), delta.y(), h)
            self.request_draw()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        if self._drag_mode and event.button() in {Qt.LeftButton, Qt.RightButton, Qt.MiddleButton}:
            self._drag_mode = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        picked = self.pick_view_point(event.position().x(), event.position().y())
        if picked is not None and self._scene_data is not None:
            self._camera.target = self._scene_data.world_to_local(picked[0])
            self.request_draw()
        event.accept()

    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802
        steps = event.angleDelta().y() / 120.0
        if abs(steps) < 1e-6:
            steps = event.pixelDelta().y() / 120.0
        precision = 0.35 if event.modifiers() & Qt.ControlModifier else 1.0
        self._camera.dolly(steps * precision)
        self.request_draw()
        event.accept()

    def keyPressEvent(self, event: QKeyEvent) -> None:  # noqa: N802
        if event.key() in {Qt.Key_F, Qt.Key_Home}:
            self.fit_content()
            event.accept()
            return
        if event.key() == Qt.Key_Escape and self._pick_mode:
            self.set_pick_mode(None)
            event.accept()
            return
        super().keyPressEvent(event)

    def _canvas_size(self) -> tuple[float, float]:
        try:
            width, height = self.get_logical_size()
            return max(1.0, float(width)), max(1.0, float(height))
        except Exception:
            return max(1.0, float(self.width())), max(1.0, float(self.height()))


def gpu_backend_available() -> tuple[bool, str | None]:
    if _GPU_IMPORT_ERROR is not None or wgpu is None:
        return False, str(_GPU_IMPORT_ERROR or "wgpu is not installed")
    try:
        adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
    except Exception as exc:
        return False, str(exc)
    if adapter is None:
        return False, "No WebGPU adapter was found"
    return True, None


def _transform_point(matrix: tuple[float, ...], point: Vec3) -> tuple[float, float, float, float]:
    x, y, z = point
    v = (x, y, z, 1.0)
    return tuple(sum(matrix[col * 4 + row] * v[col] for col in range(4)) for row in range(4))  # type: ignore[return-value]
