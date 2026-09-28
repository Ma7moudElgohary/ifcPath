# IFCPath

IFCPath converts BIM/IFC models into a portable indoor-navigation model for Digital Twins.

## Architecture

```text
IFC
 ↓
IfcOpenShell
 ↓
IfcSpace floor geometry + stairs / ramps
 ↓
Sparse semantic navigation graph
 + doors / exits / levels
 ↓
.inav (portable JSON)
 ↓
Unreal Engine first, other consumers later
```

The production core intentionally does **not** depend on TopologicPy. The first implementation adopts the useful ideas demonstrated by Topologic Studio—walkable sampling, stair handling, semantic door waypoints, and graph routing—while keeping IFCPath permissive and engine-independent.

## Current capabilities

### Generator

- IFC storeys, spaces, slabs, ramps, stairs and doors via IfcOpenShell.
- Actual bottom surfaces of `IfcSpace` are the primary walkable source.
- Slab sampling remains a fallback for IFCs without usable space geometry.
- Sparse spatial-hash / bounded-neighbour graph generation.
- Doors are semantic portals between IFC spaces.
- Explicit `IfcRelSpaceBoundary*` data is preferred over geometry inference.
- Cross-space movement goes through portals rather than direct radius edges.
- Level-aware projected wall obstacles are available as a fallback for unassigned geometry.
- Portable `.inav` JSON containing levels, spaces, portals, nodes, edges and metadata.

### Qualification

- Connected components and isolated nodes.
- Invalid/missing edge references.
- Unknown/disconnected portals.
- Two-sided semantic door attachment (`PORTAL_MISSING_SIDE`).
- Spaces and levels without navigation.
- Split-space diagnostics.
- Exit reachability counts and ratio.
- Graph-density gates for real IFC qualification.
- Continuous qualification against a real buildingSMART Duplex IFC fixture.

### Runtime routing

- Dijkstra routing.
- Blocked doors/portals.
- Blocked BIM spaces.
- Per-space cost multipliers for smoke, crowd density, security or other risk state.
- An occupant already inside a newly blocked space may still route outward; blocked spaces are treated as **no-entry**, not as traps.
- Node-level hazard costs are also supported by the standalone Python core.

## Generate INAV

```bash
python -m pip install -e ".[test]"
ifcpath build Building.ifc -o Building.inav
```

Useful tuning parameters:

```bash
ifcpath build Building.ifc -o Building.inav \
  --floor-spacing 0.8 \
  --stair-spacing 0.25 \
  --connect-distance 1.25
```

For automated qualification:

```bash
ifcpath build Building.ifc -o Building.inav --strict
```

## Validate a generated model

```bash
ifcpath validate Building.inav
```

Machine-readable output:

```bash
ifcpath validate Building.inav --json
```

To make warnings fail a CI gate:

```bash
ifcpath validate Building.inav --warnings-as-errors
```

## INAV principle

INAV exports the **navigation network**, not one precomputed route. The same BIM-derived model can therefore respond to changing Digital Twin state:

```text
Door locked
   ↓
SetPortalBlocked
   ↓
reroute

Fire / unsafe room
   ↓
SetSpaceBlocked
   ↓
no new route may enter that room
   ↓
occupants already inside can still escape

Smoke / congestion
   ↓
SetSpaceCostMultiplier (for example 5x)
   ↓
route prefers a safer / less crowded alternative
```

This keeps static BIM preprocessing separate from live operational state.

## Unreal Engine

Copy `Unreal/IFCPath` into your Unreal project's `Plugins` directory and rebuild.

`UIFCPathSubsystem` exposes Blueprint-callable operations:

### Load / query

- `LoadInav(FilePath)`
- `FindPath(StartNodeId, GoalNodeId)`
- `FindNearestNode(WorldPosition, MaxDistanceCm)`
- `FindPathFromWorldPositions(StartWorldPosition, GoalWorldPosition, MaxSnapDistanceCm)`

### Dynamic Digital Twin state

- `SetPortalBlocked(PortalId, Blocked)`
- `SetSpaceBlocked(SpaceId, Blocked)`
- `SetSpaceCostMultiplier(SpaceId, CostMultiplier)`
- `ClearDynamicState()`
- `IsSpaceBlocked(SpaceId)`
- `GetSpaceCostMultiplier(SpaceId)`

### Debug visualization

- `DrawDebugPath(Points, Color, Thickness, Duration)`
- `DrawDebugGraph(Color, Thickness, Duration)`

Debug graph colors use:

- requested graph color: normal navigation;
- **red**: blocked portal / blocked space;
- **yellow**: penalized space due to a cost multiplier.

The Unreal importer builds a bidirectional adjacency cache once during `LoadInav`, so route queries no longer scan the entire edge array for every visited node.

Typical prototype workflow:

```text
click / actor world position
        ↓
FindPathFromWorldPositions
        ↓
route
        ↓
DrawDebugPath

IoT / simulation event
        ↓
SetPortalBlocked / SetSpaceBlocked / SetSpaceCostMultiplier
        ↓
FindPathFromWorldPositions again
        ↓
new route
```

## Real IFC qualification

The buildingSMART Duplex reference is downloaded during CI and processed end-to-end:

```text
real IFC
 ↓
IfcOpenShell
 ↓
IFCPath generator
 ↓
INAV
 ↓
validator
```

The qualification gate has already exposed and driven fixes for graph density, semantic space ownership, storey assignment, door-side connectivity and wall handling. Generated INAV/report files are kept as workflow artifacts for diagnosis.

Current Duplex baseline is approximately:

- 1,976 navigation nodes;
- 8,507 edges (~4.31 edges/node);
- 4 storeys;
- 21 spaces;
- 14 door portals / 4 candidate exits;
- 0 semantic portal-side failures;
- 1 isolated node;
- 84.8% of nodes connected to a classified exit component.

Split-space warnings remain intentionally visible and are the next geometry-hardening target.

## Next hardening work

1. Add an IFC4 reference fixture in addition to the IFC2x3 Duplex.
2. Improve exit classification with IFC properties and exterior-boundary semantics.
3. Resolve split-space floor regions or introduce a Recast backend where real fixtures justify it.
4. Add stair/landing-specific topology validation.
5. Add an Unreal build/automation runner so the C++ plugin is compiled in CI.
6. Replace debug lines with a reusable spline/Niagara route visualization actor for presentation-quality Digital Twin UI.
