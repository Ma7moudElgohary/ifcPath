# IFCPath GPU Viewport

IFCPath is moving from a CPU-projected `QGraphicsScene` preview toward a real GPU rendering architecture suitable for large BIM / Digital Twin models.

The navigation model, route engines and evacuation simulators do **not** depend on the renderer.

## Why WebGPU

The GPU backend uses `wgpu-py` with `rendercanvas` embedded in PySide6.

Reasons:

- modern API over Vulkan / Metal / DX12 rather than adding more CPU projection work;
- direct Qt embedding through `QRenderWidget`;
- Windows, Linux and macOS support in wgpu-native;
- PyInstaller hooks are provided by `wgpu-py`;
- explicit GPU buffers and draw pipelines give IFCPath control over BIM batching, culling, picking and future streaming;
- permissive BSD-2-Clause dependencies.

The design was informed by That Open / Fragments, especially its separation between compact model data, GPU rendering, camera-driven culling/LOD and background/worker processing. IFCPath does not depend on Fragments or copy its implementation.

## Current milestone

Implemented:

- renderer-neutral BIM/navmesh GPU batches;
- batches grouped by level and semantic category;
- deterministic batch chunking so large models do not become one monolithic allocation;
- float32-safe origin rebasing for large IFC/geospatial coordinates;
- perspective orbit camera;
- camera-relative pan and exponential dolly/zoom;
- CPU ray/triangle picking against authoritative navmesh cells;
- persistent GPU vertex/index buffers for static BIM/navmesh geometry;
- camera motion updates only the camera uniform buffer;
- route polyline in a small dynamic GPU buffer;
- dynamic GPU overlays for graph edges, portals, exits and Start/Goal markers;
- dynamic GPU hazard / blocked-space tinting;
- dynamic blocked-door crosses;
- dynamic walking-person mesh;
- dynamic multi-person evacuation rendering;
- crowd visual LOD: first 36 active occupants use the procedural person mesh, overflow uses lightweight markers;
- grow-on-demand overlay buffers reused through simulation ticks via `queue.write_buffer()`;
- final Builder opt-in through `IFCPATH_VIEWPORT=gpu`;
- automatic fallback to the established Qt projected renderer if WebGPU initialization fails;
- an offscreen software-WebGPU CI job that actually creates a device, shader pipeline and rendered frame;
- WebGPU dependencies included in the packaged Windows build.

Still intentionally pending before GPU becomes the default renderer:

- hardware/Windows surface qualification on representative user machines;
- GPU ID-buffer IFC element picking / highlighting;
- frustum and later occlusion culling;
- geometry streaming / resident-set budgeting;
- large federated-model performance gates.

GPU mode is explicitly enabled with:

```powershell
$env:IFCPATH_VIEWPORT="gpu"
ifcpath-builder
```

or before launching the packaged executable:

```cmd
set IFCPATH_VIEWPORT=gpu
IFCPathBuilder.exe
```

If the GPU backend cannot initialize, IFCPath logs the reason and falls back to Qt.

## Static vs dynamic rendering

The viewport deliberately separates long-lived BIM geometry from simulation state:

```text
IFC BIM + INAV navmesh
        ↓
static GPU vertex/index batches
        ↓
remain resident while camera moves

route / graph / portals / hazards / people
        ↓
small dynamic GPU buffers
        ↓
write_buffer() on state changes / simulation ticks
```

Orbit, pan and zoom never rebuild IFC triangles.

## Precision model

IFC and INAV coordinates remain authoritative doubles in metres.

GPU vertex buffers use float32, so sending large projected/geospatial coordinates directly would eventually cause jitter. Each visible GPU scene therefore chooses a local origin at the physical bounds centre and stores:

```text
GPU position = IFC world position - scene origin
```

Picking and route points are translated back to the original world coordinates before they leave the viewport.

## Large-building roadmap

The target architecture is:

```text
IFC / future cached BIM geometry
            ↓
semantic element records
            ↓
spatial chunks / level chunks
            ↓
GPU resident-set manager
      ├─ visible high-detail chunks
      ├─ lower-detail distant chunks
      └─ unloaded / streamed chunks
            ↓
WebGPU renderer
            ↓
ID/depth picking + dynamic overlays
```

The next renderer milestones are:

1. qualify the embedded WebGPU surface on Windows hardware and then switch packaged `auto` mode to GPU;
2. add AABB frustum culling per batch;
3. introduce GPU ID-buffer element picking and selection/highlighting;
4. preserve IFC element/object identity through selection and rendering;
5. add GPU instancing/deduplication for repeated geometry;
6. add a memory-budgeted resident-set / tile streaming layer;
7. move IFC preview geometry extraction and GPU batch construction fully off the UI thread;
8. add performance qualification on intentionally large synthetic and real federated models;
9. add occlusion culling only after measured profiling shows it is valuable.

The portable INAV schema and routing engines remain independent of all of the above.
