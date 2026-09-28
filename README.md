# IFCPath

IFCPath converts BIM/IFC models into a portable indoor-navigation model for Digital Twins.

## Architecture

```text
IFC
 ↓
IfcOpenShell semantics + world geometry
 ↓
┌──────────────────────────────┬──────────────────────────────┐
│ semantic connectivity       │ metric movement geometry      │
│ IfcSpace / boundaries       │ IfcSpace bottom polygon       │
│ doors / vertical transfers  │ fixed-obstacle subtraction    │
│ Space ↔ Transition ↔ Space  │ clearance erosion             │
│                              │ constrained Delaunay cells     │
└──────────────┬───────────────┴──────────────┬───────────────┘
               └───────────────┬──────────────┘
                               ↓
                         portable .inav
                               ↓
                     Unreal / other targets
```

The production core does **not** depend on Pathfinder or TopologicPy. The implementation is grounded in published IFC indoor-navigation work; see [`docs/RESEARCH.md`](docs/RESEARCH.md).

## Current capabilities

### BIM / generator

- IFC storeys, `IfcSpace`, doors, stairs and ramps via IfcOpenShell.
- `IfcRelSpaceBoundary*` preferred for door↔space topology; geometry inference is fallback.
- Actual bottom surfaces of `IfcSpace` are the primary walkable source.
- Fixed IFC obstacles are subtracted before navigation generation (`IfcColumn` by default).
- Configurable pedestrian clearance and height.
- Constrained Delaunay triangulation (CDT) is the default floor backend.
- Sampled graph backend remains available for malformed IFC geometry.
- Explicit vertical semantic transitions inferred from stair/ramp landing contacts.

### Portable INAV 0.3

INAV contains both semantic and metric representations:

- levels;
- spaces;
- portals/exits;
- semantic transitions (`Space ↔ Door/Stair/Ramp ↔ Space`);
- triangular navmesh cells with reciprocal neighbours;
- metric nodes/edges for backward-compatible graph routing;
- node→cell identity;
- metadata / generator settings.

This separation lets target software perform hierarchical routing rather than treating a building as one anonymous point cloud.

### Metric route accuracy

The Python core supports a navmesh route:

```text
start / goal
 ↓
find containing CDT cells
 ↓
cell corridor
 ↓
shared cell edges (portals)
 ↓
funnel / string pulling
 ↓
short geometric path
```

Quantitative tests require:

- an open rectangular room to reduce exactly to the straight Euclidean segment;
- known rectangular-obstacle examples to match their analytic shortest detour lengths.

The old centroid graph therefore remains a connectivity/search representation, not the final visible path geometry.

### Dynamic Digital Twin routing

Runtime state is kept separate from BIM preprocessing:

- blocked door / portal;
- blocked BIM space;
- per-space cost multiplier for smoke, crowd density, security or other risk;
- occupants already inside a newly blocked space can still escape; it is treated as **no-entry**, not a trap.

## Generate INAV

```bash
python -m pip install -e ".[test]"
ifcpath build Building.ifc -o Building.inav
```

Useful options:

```bash
ifcpath build Building.ifc -o Building.inav \
  --floor-backend cdt \
  --agent-clearance 0.30 \
  --agent-height 1.80 \
  --obstacle-class IfcColumn \
  --stair-spacing 0.25 \
  --connect-distance 1.25
```

`--obstacle-class` may be repeated. Furniture is deliberately not treated as fixed by default because IFC files often mix movable and built-in objects.

Validate a generated model:

```bash
ifcpath validate Building.inav
ifcpath validate Building.inav --json
```

## Unreal Engine

Copy `Unreal/IFCPath` into your Unreal project's `Plugins` directory and rebuild.

`UIFCPathSubsystem` exposes Blueprint-callable operations.

### Existing building-wide graph routing

- `LoadInav(FilePath)`
- `FindPath(StartNodeId, GoalNodeId)`
- `FindNearestNode(WorldPosition, MaxDistanceCm)`
- `FindPathFromWorldPositions(StartWorldPosition, GoalWorldPosition, MaxSnapDistanceCm)`

This pathfinder supports dynamic door/space state and remains the fallback for cross-space and non-CDT segments.

### Accurate CDT local routing

- `FindNavMeshPathFromWorldPositions(StartWorldPosition, GoalWorldPosition)`
- `GetCellCount()`
- `DrawDebugNavMesh(Color, Thickness, Duration)`

`FindNavMeshPathFromWorldPositions` is intentionally local to a single CDT-backed IFC space. It finds the triangle corridor and runs the same funnel/string-pulling algorithm as the quantitatively tested Python reference instead of returning a centroid zig-zag.

The importer compensates for the IFC→Unreal Y-axis mirror when evaluating 2D orientation, so the funnel winding remains consistent after converting from right-handed IFC coordinates to Unreal centimetres.

### Dynamic state

- `SetPortalBlocked(PortalId, Blocked)`
- `SetSpaceBlocked(SpaceId, Blocked)`
- `SetSpaceCostMultiplier(SpaceId, CostMultiplier)`
- `ClearDynamicState()`
- `IsSpaceBlocked(SpaceId)`
- `GetSpaceCostMultiplier(SpaceId)`

### Debug visualization

- `DrawDebugPath(Points, Color, Thickness, Duration)`
- `DrawDebugGraph(Color, Thickness, Duration)`
- `DrawDebugNavMesh(Color, Thickness, Duration)`

Graph debug colors use red for blocked state and yellow for penalized spaces.

## Real IFC qualification

CI downloads and processes the buildingSMART Duplex reference end-to-end.

Current qualified baseline after the research-backed geometry/topology work:

- 2,378 metric nodes;
- 10,235 edges (~4.30 edges/node);
- 21 spaces;
- 14 door portals / 4 classified exits;
- 16 semantic transitions (including 2 inferred vertical transfers);
- 110 portable CDT navmesh cells covering all 21 CDT spaces;
- 0 split spaces;
- 0 isolated nodes;
- 0 portal-side failures;
- 0 semantic-transition errors;
- 97.8% of metric nodes connected to a classified-exit component.

The real-IFC gate also validates navmesh cell IDs, nondegenerate triangles, reciprocal cell adjacency, cell space/level consistency and node→cell references.

## Research-driven roadmap

1. Stitch semantic building routes with local funnel paths so cross-room Unreal routes use accurate geometry end-to-end.
2. Add an IFC4 fixture matrix with atria, multiple stairs, ramps, elevators and narrow doors.
3. Improve exterior/exit classification using IFC boundary semantics and properties.
4. Add route-profile constraints (wheelchair, responder, security, maintenance).
5. Add real OD-pair/reference-path length benchmarks in addition to synthetic analytic tests.
6. Add an Unreal build/automation runner so the C++ plugin itself is compiled in CI.
7. Replace debug lines with a reusable spline/Niagara route presentation layer.
