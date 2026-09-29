from __future__ import annotations

import pytest

wgpu = pytest.importorskip("wgpu")
pytest.importorskip("rendercanvas")

from rendercanvas.offscreen import RenderCanvas


_SHADER = """
@vertex
fn vs_main(@builtin(vertex_index) index: u32) -> @builtin(position) vec4f {
    var positions = array<vec2f, 3>(
        vec2f(-0.75, -0.65),
        vec2f( 0.75, -0.65),
        vec2f( 0.00,  0.75),
    );
    return vec4f(positions[index], 0.0, 1.0);
}

@fragment
fn fs_main() -> @location(0) vec4f {
    return vec4f(0.15, 0.75, 0.95, 1.0);
}
"""


def test_webgpu_offscreen_device_pipeline_and_present() -> None:
    canvas = RenderCanvas(size=(96, 64), pixel_ratio=1)
    adapter = wgpu.gpu.request_adapter_sync(power_preference="low-power")
    assert adapter is not None
    device = adapter.request_device_sync()
    context = canvas.get_wgpu_context()
    texture_format = context.get_preferred_format(adapter)
    context.configure(device=device, format=texture_format)
    shader = device.create_shader_module(code=_SHADER)
    pipeline = device.create_render_pipeline(
        layout="auto",
        vertex={"module": shader, "entry_point": "vs_main"},
        primitive={"topology": "triangle-list"},
        fragment={
            "module": shader,
            "entry_point": "fs_main",
            "targets": [{"format": texture_format}],
        },
    )

    def draw() -> None:
        texture = context.get_current_texture()
        encoder = device.create_command_encoder()
        render_pass = encoder.begin_render_pass(
            color_attachments=[
                {
                    "view": texture.create_view(),
                    "resolve_target": None,
                    "clear_value": (0.01, 0.02, 0.03, 1.0),
                    "load_op": "clear",
                    "store_op": "store",
                }
            ]
        )
        render_pass.set_pipeline(pipeline)
        render_pass.draw(3, 1, 0, 0)
        render_pass.end()
        device.queue.submit([encoder.finish()])

    canvas.request_draw(draw)
    image = canvas.draw()
    assert image.shape == (64, 96, 4)
    assert image[..., 2].max() > image[..., 0].max()
