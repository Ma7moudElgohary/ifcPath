# IFCPath GPU Viewport

IFCPath is moving from a CPU-projected `QGraphicsScene` preview toward a real GPU rendering architecture suitable for large BIM / Digital Twin models.

This is intentionally incremental. The navigation model, route engines and evacuation simulators do **not** depend on the renderer.

## Why WebGPU

The first GPU backend uses `wgpu-py` with `rendercanvas` embedded in PySide6.

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
- final Builder opt-in through `IFCPATH_VIEWPORT=gpu`;
- automatic fallback to the established Qt projected renderer if WebGPU initialization fails;
- an offscreen software-WebGPU CI job that actually creates a device, shader pipeline and rendered frame;
- WebGPU dependencies included in the packaged Windows build.

Not yet GPU-parity:

- hazard tint overlays;
- blocked-door markers;
- procedural person meshes / evacuation crowds;
- dense graph overlay;
- GPU ID-buffer element picking;
- frustum/occlusion culling;
- geometry streaming / resident-set budgeting.

Because of those remaining overlays, the normal Builder still defaults to the qualified Qt renderer. GPU mode is explicitly enabled with:

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
ID/depth picking + overlays
```

The next renderer milestones are:

1. move scenario/hazard/portal/person overlays to dynamic GPU buffers;
2. switch packaged `auto` mode to GPU after visual parity is qualified;
3. add AABB frustum culling per batch;
4. introduce element-ID picking and selection/highlighting;
5. preserve IFC element/object identity inside render batches;
6. add GPU instancing/deduplication for repeated geometry;
7. add a memory-budgeted resident-set / tile streaming layer;
8. move IFC preview geometry extraction and GPU batch construction fully off the UI thread;
9. add performance qualification on intentionally large synthetic and real federated models.

The portable INAV schema and routing engines remain independent of all of the above.
