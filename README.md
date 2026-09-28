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
- Unreal Engine runtime plugin skeleton that loads `.inav`, converts metres to Unreal centimetres, mirrors Y for handedness, finds paths and blocks/unblocks portal IDs.
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

## INAV principle

INAV exports the **navigation network**, not one precomputed route. Runtime systems can therefore react to live digital-twin state:

```text
Door blocked → disable portal → recalculate
Hazard appears → increase node/edge cost → recalculate
```

That allows one static BIM-derived navigation model to support normal wayfinding, emergency routing and future accessibility/security profiles.

## Unreal

Copy `Unreal/IFCPath` into your Unreal project's `Plugins` directory and rebuild the project. `UIFCPathSubsystem` currently exposes:

- `LoadInav(FilePath)`
- `FindPath(StartNodeId, GoalNodeId)`
- `SetPortalBlocked(PortalId, Blocked)`

The Unreal plugin is deliberately a thin consumer. IFC interpretation and graph generation stay in the standalone generator.

## Next hardening work

1. Prefer explicit IFC space-boundary relationships before geometry-based door/space inference.
2. Attach sampled navigation nodes to levels and spaces.
3. Validate disconnected spaces, doors, stairs and exits.
4. Replace/augment point sampling with a Recast navmesh backend while preserving the same INAV schema.
5. Add Unreal start/end snapping and route visualization components.
