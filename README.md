# IFCPath

IFCPath converts BIM/IFC models into a portable indoor-navigation model for digital twins.

## Architecture

```text
IFC
 ↓
IfcOpenShell
 ↓
Walkable floor/stair sampling
 ↓
Spatial navigation graph
 + semantic spaces / doors / exits
 ↓
.inav (portable JSON)
 ↓
Unreal Engine first, other consumers later
```

The production core intentionally does **not** depend on TopologicPy. The MVP reuses the useful ideas proven by Topologic Studio (walkable surface sampling, separate stair handling, forced door waypoints, graph routing), while keeping IFCPath permissive and target-engine independent.

## MVP capabilities

- IFC storeys, spaces, slabs, ramps, stairs and doors via IfcOpenShell.
- Walkable triangle sampling with slope filtering.
- Spatial-hash radius graph generation.
- Doors exported as semantic portals; single-sided doors are marked as candidate exits.
- Portable `.inav` JSON containing levels, spaces, portals, nodes and edges.
- Dijkstra routing with runtime blocked portals and hazard-cost penalties.
- Qualification report for disconnected graphs, orphan nodes, disconnected portals/exits, spaces/levels without navigation, and exit reachability.
- Unreal Engine runtime plugin that loads `.inav`, converts metres to Unreal centimetres, mirrors Y for handedness, snaps arbitrary world positions to the graph, routes, and visualizes paths/graphs.
- Python tests and GitHub Actions CI.

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

For automated qualification, use strict mode:

```bash
ifcpath build Building.ifc -o Building.inav --strict
```

## Validate a generated model

```bash
ifcpath validate Building.inav
```

Machine-readable qualification output:

```bash
ifcpath validate Building.inav --json
```

To make warnings fail a CI/build gate:

```bash
ifcpath validate Building.inav --warnings-as-errors
```

Validation currently checks:

- invalid edge references and negative edge lengths;
- connected-component count and largest-component coverage;
- isolated navigation nodes;
- unknown/disconnected portal references;
- absence of classified exits;
- spaces and levels with no assigned navigation nodes;
- navigation nodes that cannot reach any classified exit.

This is intended to make real-IFC qualification measurable instead of visually guessing whether graph generation worked.

## INAV principle

INAV exports the **navigation network**, not one precomputed route. Runtime systems can therefore react to live digital-twin state:

```text
Door blocked → disable portal → recalculate
Hazard appears → increase node/edge cost → recalculate
```

That allows one static BIM-derived navigation model to support normal wayfinding, emergency routing and future accessibility/security profiles.

## Unreal

Copy `Unreal/IFCPath` into your Unreal project's `Plugins` directory and rebuild the project.

`UIFCPathSubsystem` exposes Blueprint-callable operations:

- `LoadInav(FilePath)`
- `FindPath(StartNodeId, GoalNodeId)`
- `FindNearestNode(WorldPosition, MaxDistanceCm)`
- `FindPathFromWorldPositions(StartWorldPosition, GoalWorldPosition, MaxSnapDistanceCm)`
- `SetPortalBlocked(PortalId, Blocked)`
- `DrawDebugPath(Points, Color, Thickness, Duration)`
- `DrawDebugGraph(Color, Thickness, Duration)`

The normal prototype workflow is therefore:

```text
click / actor world position
        ↓
FindPathFromWorldPositions
        ↓
INAV graph route
        ↓
DrawDebugPath (prototype)
        ↓
later replace with spline / Niagara / navigation UI
```

Blocked portal edges are shown red by `DrawDebugGraph`, which is useful while testing dynamic door/hazard scenarios.

The Unreal plugin is deliberately a thin consumer. IFC interpretation and graph generation stay in the standalone generator.

## Real IFC qualification workflow

For each fixture/building:

```text
IFC
 ↓
ifcpath build --strict
 ↓
ifcpath validate --json
 ↓
review components / spaces / exits / warnings
 ↓
load INAV in Unreal
 ↓
draw graph
 ↓
route between arbitrary world points
 ↓
block a door and verify reroute
```

The next hardening decision should be based on these fixture reports. If point sampling fails systematically on complex geometry, Recast can replace/augment the generator while preserving the same INAV consumer contract.

## Next hardening work

1. Add a curated real-IFC fixture matrix and qualification report artifacts in CI.
2. Improve exit classification using IFC property sets and building-boundary tests.
3. Add stair/landing-specific validation and richer vertical-connection semantics.
4. Add spatial acceleration for Unreal nearest-node queries and Dijkstra adjacency caching for large graphs.
5. Replace/augment point sampling with a Recast navmesh backend if fixture results justify it.
6. Replace debug route lines with a reusable spline/route visualization actor for presentation-quality Digital Twin UI.
