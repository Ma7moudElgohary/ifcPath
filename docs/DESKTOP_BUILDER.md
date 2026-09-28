# IFCPath Builder Desktop

The desktop Builder is a PySide6 application on top of the same engine used by the `ifcpath` CLI. It does not spawn the CLI as a subprocess; IFC generation, validation, route QA and export call the Python core directly.

## Install and run

```bash
python -m pip install -e ".[desktop]"
ifcpath-builder
```

The Builder supports:

- opening IFC models;
- opening existing `.inav`/JSON navigation models;
- selecting CDT or sampled floor generation;
- pedestrian clearance and height;
- configurable fixed IFC obstacle classes;
- floor/stair sampling and graph-connection parameters;
- non-blocking Build & Validate processing;
- projected 3D BIM context for IFC builds;
- navmesh, graph, portal and exit overlays;
- individual storeys or all-storey 3D display;
- exact click-to-pick Start/Goal points on CDT cells;
- hierarchical multi-space/multi-storey route calculation;
- stitched 3D path display through doors, stairs and ramps;
- live blocked-door and blocked-space scenarios;
- smoke, fire and crowd route-cost scenarios;
- automatic rerouting whenever scenario state changes;
- a procedural low-poly 3D person agent at the route start;
- validation diagnostics and summary metrics;
- `.inav` export;
- drag/drop for IFC and INAV files;
- persisted generation settings.

## 3D navigation preview

The viewport remains engine-independent and Qt-native. It uses a small orthographic 3D camera instead of introducing another rendering engine into the desktop product.

For IFC builds, the worker extracts a **preview-only** bounded triangle soup from walls, slabs, columns, stairs, ramps and doors. The triangle count is capped to keep the Builder responsive. BIM preview geometry is never written into `.inav` and never participates in route semantics.

The navigation overlay remains the source of truth:

- CDT cells grouped by semantic `IfcSpace`;
- metric graph edges;
- semantic portals/doors;
- classified exits;
- route start/goal markers;
- the stitched hierarchical route.

Interaction:

- **Right drag**: orbit the 3D view.
- **Mouse wheel**: zoom.
- **Double click**: fit content.
- **3D orbit / Top view**: switch projection mode.
- **Pick Start / Pick Goal**: click directly on a visible walkable CDT triangle.

Picking is not nearest-node snapping. The clicked screen point is located inside the projected CDT triangle and converted back to a world-space XYZ point with barycentric interpolation. That exact point is sent to `find_hierarchical_path()`.

After both endpoints are selected, the Builder calculates and displays the same hierarchical route used by the Python reference engine:

```text
clicked start XYZ
    ↓
local CDT + funnel route
    ↓
door / stair / ramp transfer
    ↓
next semantic space
    ↓
local CDT + funnel route
    ↓
...
    ↓
clicked goal XYZ
```

The route panel reports physical length, semantic-space count, transfer count and weighted dynamic cost.

When opening an existing INAV without its original IFC, all navigation 3D features remain available; only the optional BIM context mesh is absent.

## Live scenario editor

Scenario changes are runtime-only. They are passed to `HierarchicalRouteOptions` and never mutate the canonical INAV topology.

Supported controls:

- select a door/portal and **Block / Unblock** it;
- select a space and **Block / Unblock** it;
- apply **Smoke**, **Fire**, or **Crowd** to a space with a configurable route-cost multiplier;
- clear a selected hazard or reset the complete scenario.

If Start and Goal are already selected, every scenario edit recalculates the route immediately. The 3D preview overlays blocked doors with a red cross, blocked spaces in red, and hazard spaces with type-specific shading.

The person agent is generated procedurally from triangular boxes plus a faceted head. It is approximately 1.72 m tall by default, stands at the exact picked Start XYZ, and rotates toward the first non-zero route segment. It is display-only and introduces no external mesh asset or licensing dependency.

## Native desktop bundle

Install build extras:

```bash
python -m pip install -e ".[desktop-build]"
```

Then run:

```bash
python scripts/build_desktop.py
```

PyInstaller creates `IFCPathBuilder` under `dist/`. Build on the target operating system (for example, build the Windows executable on Windows).

The packaging helper explicitly collects IFCPath desktop submodules plus IfcOpenShell and Shapely runtime files in addition to the Qt runtime.

## CI

The normal test job compiles all Python sources. The `desktop-smoke` job installs PySide6, uses Qt's `offscreen` platform and qualifies:

- main-window construction;
- INAV model binding;
- projected 3D viewport rendering;
- screen-to-CDT world-point recovery;
- hierarchical route calculation between selected world points;
- procedural person-mesh geometry and rendering;
- live scenario state without mutating the INAV model.

A Windows packaging job also builds and uploads an `IFCPathBuilder-Windows` PyInstaller artifact.
