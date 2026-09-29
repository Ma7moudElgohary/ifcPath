from __future__ import annotations

from array import array

import wgpu

from ..model import InavModel, Vec3
from .gpu_overlay import build_navigation_overlay, build_scenario_overlay
from .gpu_preview import GpuBimPreview
from .person_mesh import build_person_mesh
from .preview_geometry import PreviewGeometry


class GpuScenarioPreview(GpuBimPreview):
    """WebGPU preview for the real scenario/evacuation Builder shell.

    BIM/navmesh batches remain resident. Route, portal/graph/endpoint markers,
    hazards, blocked portals, the walking agent and evacuation occupants use
    separate grow-on-demand dynamic buffers. Once capacity is sufficient, a
    simulation tick only writes new bytes; it does not allocate GPU resources or
    rebuild static IFC geometry.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._blocked_portals: set[str] = set()
        self._blocked_spaces: set[str] = set()
        self._space_cost_multipliers: dict[str, float] = {}
        self._hazard_kinds: dict[str, str] = {}
        self._show_person = True
        self._agent_position: Vec3 | None = None
        self._agent_forward: tuple[float, float] = (0.0, 1.0)
        self._evacuation_poses: dict[str, tuple[Vec3, tuple[float, float], str]] = {}
        self._overlay_triangle_buffer = None
        self._overlay_triangle_capacity = 0
        self._overlay_triangle_count = 0
        self._overlay_line_buffer = None
        self._overlay_line_capacity = 0
        self._overlay_line_count = 0
        self._person_triangle_count = 0
        self._refresh_overlay_buffers()

    # ---------- preserve the preview contract while refreshing only overlays ----------

    def set_model(self, model: InavModel | None) -> None:
        super().set_model(model)
        self._refresh_overlay_buffers()

    def set_bim_geometry(self, geometry: PreviewGeometry | None) -> None:
        super().set_bim_geometry(geometry)
        self._refresh_overlay_buffers()

    def set_bim_visible(self, visible: bool) -> None:
        super().set_bim_visible(visible)
        self._refresh_overlay_buffers()

    def set_level(self, level_id: str | None) -> None:
        super().set_level(level_id)
        self._refresh_overlay_buffers()

    def set_layers(self, *, navmesh: bool, graph: bool, portals: bool, exits: bool) -> None:
        super().set_layers(navmesh=navmesh, graph=graph, portals=portals, exits=exits)
        self._refresh_overlay_buffers()

    def set_route(self, start: Vec3 | None, goal: Vec3 | None, points: list[Vec3] | None) -> None:
        super().set_route(start, goal, points)
        self._refresh_overlay_buffers()

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
        self._refresh_overlay_buffers()
        self.request_draw()

    def set_person_visible(self, visible: bool) -> None:
        self._show_person = bool(visible)
        self._refresh_overlay_buffers()
        self.request_draw()

    def set_agent_pose(
        self,
        position: Vec3 | None,
        forward: tuple[float, float] | None = None,
    ) -> None:
        self._agent_position = position
        if forward is not None:
            self._agent_forward = forward
        self._refresh_overlay_buffers()
        self.request_draw()

    def set_evacuation_agents(
        self,
        poses: dict[str, tuple[Vec3, tuple[float, float], str]] | None,
    ) -> None:
        self._evacuation_poses = dict(poses or {})
        self._refresh_overlay_buffers()
        self.request_draw()

    @property
    def blocked_portals(self) -> set[str]:
        return set(self._blocked_portals)

    @property
    def blocked_spaces(self) -> set[str]:
        return set(self._blocked_spaces)

    @property
    def agent_position(self) -> Vec3 | None:
        return self._agent_position

    @property
    def evacuation_agent_count(self) -> int:
        return len(self._evacuation_poses)

    @property
    def person_triangle_count(self) -> int:
        return self._person_triangle_count

    @property
    def overlay_triangle_count(self) -> int:
        return self._overlay_triangle_count // 3

    @property
    def overlay_line_count(self) -> int:
        return self._overlay_line_count // 2

    # ---------- dynamic buffers ----------

    def _refresh_overlay_buffers(self) -> None:
        scene = self._scene_data
        self._overlay_triangle_count = 0
        self._overlay_line_count = 0
        self._person_triangle_count = 0
        if scene is None:
            return

        overlay = build_navigation_overlay(
            self._model,
            scene,
            level_id=self._level_id,
            show_graph=self._show_graph,
            show_portals=self._show_portals,
            show_exits=self._show_exits,
            start=self._start_point,
            goal=self._goal_point,
        )
        scenario = build_scenario_overlay(
            self._model,
            scene,
            level_id=self._level_id,
            blocked_portals=self._blocked_portals,
            blocked_spaces=self._blocked_spaces,
            space_cost_multipliers=self._space_cost_multipliers,
            hazard_kinds=self._hazard_kinds,
            show_person=self._show_person,
            agent_position=self._agent_position,
            agent_forward=self._agent_forward,
            evacuation_poses=self._evacuation_poses,
        )
        overlay.extend(scenario)

        (
            self._overlay_triangle_buffer,
            self._overlay_triangle_capacity,
        ) = self._write_dynamic_buffer(
            self._overlay_triangle_buffer,
            self._overlay_triangle_capacity,
            overlay.triangle_vertices,
        )
        self._overlay_triangle_count = overlay.triangle_vertex_count
        (
            self._overlay_line_buffer,
            self._overlay_line_capacity,
        ) = self._write_dynamic_buffer(
            self._overlay_line_buffer,
            self._overlay_line_capacity,
            overlay.line_vertices,
        )
        self._overlay_line_count = overlay.line_vertex_count

        if self._show_person and self._agent_position is not None:
            self._person_triangle_count = len(
                build_person_mesh(self._agent_position, self._agent_forward, height_m=1.72)
            )

    def _write_dynamic_buffer(self, buffer, capacity: int, data: array):
        required = len(data) * data.itemsize
        if required <= 0:
            return buffer, capacity
        if buffer is None or capacity < required:
            new_capacity = 256
            while new_capacity < required:
                new_capacity *= 2
            buffer = self._device.create_buffer(
                size=new_capacity,
                usage=wgpu.BufferUsage.VERTEX | wgpu.BufferUsage.COPY_DST,
            )
            capacity = new_capacity
        self._device.queue.write_buffer(buffer, 0, data)
        return buffer, capacity

    # ---------- draw static batches + small dynamic overlays in one frame ----------

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
