from __future__ import annotations

from array import array
import os

import wgpu

from .gpu_culling import aabb_visible_in_clip, world_bounds_to_local
from .gpu_residency import GpuResidentSet, ResidentBatchInfo, batch_center, budget_bytes_from_mb
from .gpu_scenario_preview import GpuScenarioPreview
from .gpu_scene import build_gpu_scene


class GpuStreamingScenarioPreview(GpuScenarioPreview):
    """Scenario renderer with a memory-budgeted static GPU resident set.

    CPU batch data stays available for deterministic re-upload, selection and
    future disk caching. Only the most valuable spatial batches occupy GPU memory:
    visible chunks first, then a small camera-near prefetch ring. New uploads are
    throttled per frame to keep large camera jumps responsive.
    """

    def __init__(self, parent=None) -> None:
        budget_mb = _env_float("IFCPATH_GPU_BUDGET_MB", 512.0)
        upload_mb = _env_float("IFCPATH_GPU_UPLOAD_MB_PER_FRAME", 64.0)
        prefetch_batches = _env_int("IFCPATH_GPU_PREFETCH_BATCHES", 12)
        upload_batches = _env_int("IFCPATH_GPU_UPLOAD_BATCHES_PER_FRAME", 8)
        self._resident_set = GpuResidentSet(
            budget_bytes_from_mb(budget_mb),
            prefetch_batches=prefetch_batches,
            max_upload_batches=upload_batches,
            max_upload_bytes=budget_bytes_from_mb(upload_mb),
        )
        self._resident_buffers: dict[str, tuple[object, object, int, str]] = {}
        self._batch_by_key = {}
        self._last_resident_bytes = 0
        self._last_visible_requested = 0
        self._last_visible_dropped = 0
        self._uploaded_batches_total = 0
        self._evicted_batches_total = 0
        super().__init__(parent)

    @property
    def gpu_memory_budget_bytes(self) -> int:
        return self._resident_set.budget_bytes

    @property
    def resident_static_batch_count(self) -> int:
        return len(self._resident_buffers)

    @property
    def resident_static_bytes(self) -> int:
        return self._last_resident_bytes

    @property
    def visible_requested_batch_count(self) -> int:
        return self._last_visible_requested

    @property
    def visible_dropped_batch_count(self) -> int:
        return self._last_visible_dropped

    @property
    def uploaded_static_batches_total(self) -> int:
        return self._uploaded_batches_total

    @property
    def evicted_static_batches_total(self) -> int:
        return self._evicted_batches_total

    def set_gpu_memory_budget_mb(self, value: float) -> None:
        self._resident_set.set_budget(budget_bytes_from_mb(value))
        self.request_draw()

    # ---------- scene / residency lifecycle ----------

    def _rebuild_scene(self, *, fit: bool) -> None:
        self._destroy_all_resident_buffers()
        self._scene_data = build_gpu_scene(
            self._model,
            self._bim,
            level_id=self._level_id,
            show_bim=self._show_bim,
            show_navmesh=self._show_navmesh,
        )
        scene = self._scene_data
        self._batch_by_key = {batch.key: batch for batch in scene.batches}
        self._resident_set.reset(
            ResidentBatchInfo(
                key=batch.key,
                byte_size=batch.byte_size,
                center=scene.world_to_local(batch_center(batch.bounds_min, batch.bounds_max)),
            )
            for batch in scene.batches
        )
        self._last_total_static_batches = len(scene.batches)
        self._last_visible_static_batches = 0
        self._last_resident_bytes = 0
        self._last_visible_requested = 0
        self._last_visible_dropped = 0
        self._upload_route()
        if fit:
            self.fit_content()
        self.request_draw()

    def _sync_residency(self, matrix: tuple[float, ...]) -> set[str]:
        scene = self._scene_data
        if scene is None:
            return set()

        visible: list[str] = []
        for batch in scene.batches:
            local_min, local_max = world_bounds_to_local(batch.bounds_min, batch.bounds_max, scene.origin)
            if aabb_visible_in_clip(matrix, local_min, local_max):
                visible.append(batch.key)

        plan = self._resident_set.plan(
            visible_keys=visible,
            camera_position=self._camera.position(),
        )
        for key in plan.evict:
            buffers = self._resident_buffers.pop(key, None)
            if buffers is not None:
                _destroy_buffer(buffers[0])
                _destroy_buffer(buffers[1])
                self._evicted_batches_total += 1

        for key in plan.upload:
            batch = self._batch_by_key.get(key)
            if batch is None:
                continue
            vertex_buffer = self._device.create_buffer_with_data(
                data=batch.vertices,
                usage=wgpu.BufferUsage.VERTEX,
            )
            index_buffer = self._device.create_buffer_with_data(
                data=batch.indices,
                usage=wgpu.BufferUsage.INDEX,
            )
            self._resident_buffers[key] = (
                vertex_buffer,
                index_buffer,
                len(batch.indices),
                batch.category,
            )
            self._uploaded_batches_total += 1

        resident_keys = set(self._resident_buffers)
        visible_resident = set(visible) & resident_keys
        self._last_total_static_batches = len(scene.batches)
        self._last_visible_static_batches = len(visible_resident)
        self._last_resident_bytes = sum(
            self._batch_by_key[key].byte_size
            for key in resident_keys
            if key in self._batch_by_key
        )
        self._last_visible_requested = len(visible)
        self._last_visible_dropped = len(set(visible) - resident_keys)

        if plan.target_pending:
            # Rendercanvas coalesces draw requests; this progressively fills the
            # resident set without one giant synchronous upload burst.
            self.request_draw()
        return visible_resident

    def _destroy_all_resident_buffers(self) -> None:
        for vertex_buffer, index_buffer, _count, _category in self._resident_buffers.values():
            _destroy_buffer(vertex_buffer)
            _destroy_buffer(index_buffer)
        self._resident_buffers.clear()

    # ---------- draw static resident chunks + inherited dynamic overlays ----------

    def _draw_frame(self) -> None:
        try:
            current_texture = self._context.get_current_texture()
        except getattr(wgpu, "DrawCancelled", RuntimeError):
            return

        width, height = self.get_physical_size()
        width, height = max(1, int(width)), max(1, int(height))
        aspect = width / max(1.0, float(height))
        matrix_tuple = self._camera.view_projection(aspect)
        matrix = array("f", matrix_tuple)
        self._device.queue.write_buffer(self._camera_buffer, 0, matrix)
        visible_resident = self._sync_residency(matrix_tuple)

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
        scene = self._scene_data
        if scene is not None:
            for batch in scene.batches:
                if batch.key not in visible_resident:
                    continue
                resident = self._resident_buffers.get(batch.key)
                if resident is None:
                    continue
                vertex_buffer, index_buffer, count, _category = resident
                render_pass.set_vertex_buffer(0, vertex_buffer)
                render_pass.set_index_buffer(index_buffer, "uint32")
                render_pass.draw_indexed(count, 1, 0, 0, 0)

        if self._overlay_triangle_buffer is not None and self._overlay_triangle_count:
            render_pass.set_vertex_buffer(0, self._overlay_triangle_buffer)
            render_pass.draw(self._overlay_triangle_count, 1, 0, 0)

        render_pass.set_pipeline(self._line_pipeline)
        if self._route_buffer is not None and self._route_vertex_count:
            render_pass.set_vertex_buffer(0, self._route_buffer)
            render_pass.draw(self._route_vertex_count, 1, 0, 0)
        if self._overlay_line_buffer is not None and self._overlay_line_count:
            render_pass.set_vertex_buffer(0, self._overlay_line_buffer)
            render_pass.draw(self._overlay_line_count, 1, 0, 0)

        render_pass.end()
        self._device.queue.submit([encoder.finish()])


def _destroy_buffer(buffer: object) -> None:
    destroy = getattr(buffer, "destroy", None)
    if callable(destroy):
        try:
            destroy()
        except Exception:
            pass


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default
