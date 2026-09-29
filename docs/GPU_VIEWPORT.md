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
- explicit GPU buffers and draw pipelines give IFCPath control over BIM batching, culling, picking and streaming;
- permissive BSD-2-Clause dependencies.

The design was informed by That Open / Fragments, especially its separation between compact model data, GPU rendering, camera-driven culling/LOD, BIM identity, workers and streamed/tiled model data. IFCPath does not depend on Fragments or copy its implementation.

## Current milestone

Implemented:

- renderer-neutral BIM/navmesh GPU batches;
- batches grouped by level and semantic category;
- Morton/spatial ordering before deterministic chunking so batch AABBs represent local building regions;
- per-batch AABB frustum culling before draw submission;
- **memory-budgeted GPU residency** for static BIM/navmesh batches;
- visible chunks have first residency priority;
- nearby invisible chunks are prefetched when budget remains;
- irrelevant chunks are evicted and their GPU buffers destroyed;
- camera jumps use bounded uploads per frame instead of one huge synchronous upload burst;
- configurable GPU memory/upload/prefetch budgets through environment variables;
- CPU-side batch data stays authoritative for re-upload, selection and future disk caching;
- float32-safe origin rebasing for large IFC/geospatial coordinates;
- perspective orbit camera;
- camera-relative pan and exponential dolly/zoom;
- CPU ray/triangle picking against authoritative navmesh cells for Start/Goal;
- route polyline in a small dynamic GPU buffer;
- dynamic GPU overlays for graph edges, portals, exits and Start/Goal markers;
- dynamic GPU hazard / blocked-space tinting;
- dynamic blocked-door crosses;
- dynamic walking-person mesh;
- dynamic multi-person evacuation rendering;
- crowd visual LOD: first 36 active occupants use the procedural person mesh, overflow uses lightweight markers;
- grow-on-demand overlay buffers reused through simulation ticks via `queue.write_buffer()`;
- IFC GlobalId-preserving BIM selection;
- BVH-accelerated element picking instead of scanning all BIM triangles;
- selected-element GPU bounding highlight without rebuilding static BIM batches;
- docked IFC inspector with occurrence attributes and inherited property sets/quantities;
- final Builder opt-in through `IFCPATH_VIEWPORT=gpu`;
- automatic fallback to the established Qt projected renderer if WebGPU initialization fails;
- an offscreen software-WebGPU CI job that actually creates a device, shader pipeline and rendered frame;
- WebGPU dependencies included in the packaged Windows build.

Still intentionally pending before GPU becomes the default renderer:

- hardware/Windows surface qualification on representative user machines;
- CPU/disk tile streaming so nonresident chunks need not all stay expanded in RAM;
- repeated-geometry instancing/deduplication;
- large federated-model performance gates;
- optional GPU ID-buffer picking if profiling shows BVH picking is insufficient for heavily instanced scenes;
- occlusion culling only if profiling shows a benefit beyond frustum/residency culling.

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

## GPU residency budget

The static geometry memory policy is intentionally explicit and deterministic.

Defaults:

```text
GPU resident budget             512 MB
new static uploads / frame       64 MB
new batch count / frame           8
near-camera prefetch batches     12
```

They can be overridden before launch:

```powershell
$env:IFCPATH_GPU_BUDGET_MB="1024"
$env:IFCPATH_GPU_UPLOAD_MB_PER_FRAME="96"
$env:IFCPATH_GPU_UPLOAD_BATCHES_PER_FRAME="10"
$env:IFCPATH_GPU_PREFETCH_BATCHES="16"
```

For each frame:

```text
camera frustum
      ↓
visible spatial batch keys
      ↓
resident-set policy
      ├─ visible nearest first
      ├─ nearby prefetch next
      ├─ retain useful existing chunks to avoid churn
      └─ evict batches outside the target budget
      ↓
throttled create_buffer_with_data()
      ↓
render resident visible batches
```

The GPU budget is a hard target except when one required chunk is itself larger than the configured budget; that single chunk is allowed so rendering can still make forward progress.

This is **GPU-residency streaming**, not yet full disk streaming. CPU-side `GpuMeshBatch` data remains in memory, making selection and re-upload immediate. The next large-model storage milestone is to serialize those chunks to a compact cache and load/decode them on demand.

## Static vs dynamic rendering

The viewport separates long-lived BIM geometry from simulation state:

```text
IFC BIM + INAV navmesh
        ↓
CPU spatial batch catalog
        ↓
GPU resident-set manager
        ↓
only useful static vertex/index buffers stay resident

route / graph / portals / hazards / people
        ↓
small dynamic GPU buffers
        ↓
write_buffer() on state changes / simulation ticks
```

Orbit, pan and zoom never rebuild IFC triangles.

## IFC selection and inspection

A normal left click in GPU mode selects BIM geometry; Start/Goal pick mode remains independent and continues to target CDT navigation cells.

```text
screen click
    ↓
perspective camera ray
    ↓
BIM BVH / AABB traversal
    ↓
exact IFC preview triangle
    ↓
GlobalId
    ↓
selection highlight + IFC inspector
```

The inspector displays IFC identity, scalar occurrence attributes and property sets/quantities extracted with IfcOpenShell. The BVH avoids an O(N) triangle scan and avoids a synchronous GPU readback stall on each click. The selected element is highlighted through a tiny dynamic line buffer; static BIM buffers remain independently residency-managed.

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
IFC / cached BIM geometry
            ↓
semantic element records + GlobalId
            ↓
spatial chunks / level chunks
            ↓
CPU/disk chunk catalog
            ↓
GPU resident-set manager
      ├─ visible resident chunks
      ├─ nearby prefetched chunks
      └─ evicted / on-disk chunks
            ↓
WebGPU renderer
            ↓
BVH/ID picking + dynamic overlays + IFC inspector
```

The next renderer milestones are:

1. qualify the embedded WebGPU surface on Windows hardware and then switch packaged `auto` mode to GPU;
2. add an on-disk compact preview cache + background chunk decode/loading;
3. preserve per-element geometry ranges for isolation/hide/show and repeated-geometry instancing;
4. add GPU instancing/deduplication for repeated geometry;
5. move IFC preview geometry extraction and cache construction fully off the UI thread;
6. add performance qualification on intentionally large synthetic and real federated models;
7. add worker-style streaming/concurrency without one worker/process per federated model;
8. add occlusion culling only after measured profiling shows it is valuable.

The portable INAV schema and routing engines remain independent of all of the above.
