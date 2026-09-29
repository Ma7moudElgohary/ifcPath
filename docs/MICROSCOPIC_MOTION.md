# IFCPath Microscopic / Hybrid Motion

IFCPath separates **navigation decisions** from **pedestrian operational motion**.

IFCPath owns BIM semantics, hierarchical routes, exit selection, hazards, blocked spaces/portals, bottleneck capacities and transition policy. A local-motion backend is responsible only for moving people toward IFCPath-supplied local targets inside one connected semantic movement domain.

```text
IFC / INAV
   ↓
IFCPath semantic + metric routing
   ↓
HybridEvacuationSimulator
   ├── local route inside semantic space ──→ LocalMotionBackend
   │                                         ├── Kinematic fallback
   │                                         └── JuPedSim
   ├── semantic transition gate / queue
   └── exact door / stair / ramp / elevator transfer
                    ↓
             destination domain
```

This boundary is intentional. An operational crowd solver must not reinterpret rooms, walls, doors, exits, stairs or emergency policy.

## Common backend contract

`src/ifcpath/microscopic_motion.py` defines the host-independent `LocalMotionBackend` protocol:

- `add_agent(spec)`
- `remove_agent(agent_id)`
- `set_target(agent_id, target_m)`
- `advance(delta_seconds)`
- `snapshot(agent_id)`
- `snapshots()`

The corresponding data types are:

- `MicroscopicMotionConfig`
- `MicroscopicAgentSpec`
- `MicroscopicAgentSnapshot`

The contract contains no Qt or Unreal dependency.

`remove_agent()` is part of the contract because a building-wide simulation must remove a pedestrian from one microscopic domain before handing that same logical occupant through a semantic portal or vertical transition and into another domain.

## Route-driven local motion

`MicroscopicRouteController` converts an IFCPath route polyline into sequential local targets. It owns waypoint progression, not pathfinding. It can also replace the remaining route from an agent's exact current position after a live scenario change.

All occupants assigned to the same semantic movement domain are advanced together, preserving solver interaction inside that room/corridor/open area.

## Deterministic fallback

`KinematicLocalMotionBackend` moves each agent directly toward its current target at its configured desired speed.

It intentionally provides **no agent-agent collision physics**. Its purpose is to keep the same hybrid handoff architecture available for tests, debugging and default packaged builds that do not install a microscopic solver.

## JuPedSim backend

`JuPedSimLocalMotionBackend` is an optional adapter qualified against **JuPedSim 1.4.2**.

Install it with:

```bash
pip install -e ".[microscopic]"
```

The backend currently uses JuPedSim's collision-free speed model V2 by default and creates a **direct-steering stage**. IFCPath therefore retains route choice and updates an agent's target, while JuPedSim resolves operational pedestrian interaction and collision avoidance inside the domain geometry.

A V3 operational model can be selected through `MicroscopicMotionConfig(model="cfsm_v3")` when supported by the installed JuPedSim build.

## Geometry transfer and connected domains

JuPedSim requires a connected 2D accessible area. IFCPath already owns qualified CDT cells, so no second BIM interpretation is introduced.

For hybrid mode the domain is a semantic space on a level:

```text
qualified CDT triangles for IfcSpace X
          ↓ unary union
connected Shapely walkable geometry
          ↓
JuPedSim Simulation geometry
```

`build_level_walkable_geometry(model, level_id, space_id=...)` reconstructs that domain from existing INAV cells. Agent starts and direct-steering targets are projected inside the walkable domain when necessary so small boundary numerical differences do not create invalid JuPedSim placements.

The older whole-level helper remains available when `space_id` is omitted, but the JuPedSim adapter rejects disconnected accessible geometry with a clear error.

### Why not union the whole floor and bridge gaps?

A real IFC floor can contain multiple room polygons separated by wall thickness. JuPedSim 1.4.2 correctly rejects those polygons as one disconnected accessible area. Adding artificial geometric bridges would be dangerous: it could create a walkable path through a wall or allow a blocked door to be bypassed by collision avoidance.

Hybrid evacuation therefore uses **semantic-space-local microscopic domains**. Doors/openings are hard IFCPath-owned handoffs, so portal state remains authoritative.

## Hybrid building-wide coordinator

`src/ifcpath/hybrid_evacuation.py` implements the coordinator.

The coordinator compiles every `EvacuationPlan` into execution steps:

```text
local route in space A
→ transition capacity gate
→ exact semantic transfer polyline
→ local route in space B
→ transition capacity gate
→ exact 3D stair/ramp/elevator transfer
→ local route in destination space
→ ...
→ exit capacity gate
```

For every active semantic domain, one shared `MicroscopicRouteController` / local-motion backend is used by all occupants in that domain.

For **every semantic transition**, including same-floor doors:

1. the agent reaches the transition approach point inside its current space;
2. the transition's capacity gate controls admission and queueing;
3. the agent is removed from the origin microscopic domain;
4. IFCPath advances the occupant along the transition's exact route polyline;
5. the occupant is injected into the destination microscopic domain.

Stairs, ramps, elevators and escalators use the same handoff mechanism but retain their 3D geometry and configured vertical speed factors.

This keeps semantic capacity, walls, portal state, 3D BIM geometry and crowd interaction in the layer that actually owns each responsibility.

## Live replanning

When blocked portals, blocked spaces or runtime hazard/crowd costs change, the hybrid coordinator replans every non-evacuated occupant from its **exact live XYZ**.

Active microscopic domains are rebuilt during replan so native solver agents from the previous tactical routes cannot remain as stale solver state. Already evacuated occupants remain evacuated.

## Builder modes

The desktop Builder exposes three movement modes:

- **Mesoscopic (fast)** — existing deterministic route / queue / capacity simulator; default.
- **Hybrid deterministic** — same semantic-domain handoffs, but uses the dependency-free kinematic local backend.
- **Hybrid microscopic (JuPedSim)** — uses JuPedSim inside connected semantic spaces and IFCPath for all portal/vertical handoffs.

Selecting JuPedSim in a build where the optional dependency is not installed produces a clear UI message instead of a crash. The default Windows package therefore remains lightweight; a deployment that wants microscopic physics can install/package the `microscopic` extra explicitly.

## Qualification

CI keeps separate gates for the normal product and optional solver.

The microscopic/hybrid qualification verifies:

- CDT cells reconstruct expected continuous geometry;
- the deterministic fallback implements the same local-motion contract;
- route waypoint turns and live route replacement;
- real JuPedSim direct steering;
- two head-on JuPedSim agents maintain separation while making progress;
- a hierarchical route compiles a capacity gate before a true 3D stair transfer;
- deterministic upper-floor → stair → lower-floor → exit handoff;
- stair capacity creates waiting/queue behaviour;
- hybrid replanning starts from the exact live occupant position;
- a real JuPedSim upper-floor agent crosses a true 3D vertical handoff;
- two disconnected same-floor room domains are crossed only through their explicit door gate/transfer, never by asking JuPedSim to jump the wall gap.

The ordinary Python and desktop suites still run without JuPedSim, preserving the optional dependency boundary.

## Packaging and licensing boundary

JuPedSim is optional rather than part of IFCPath's default dependency set. This keeps the normal generator and Windows Builder lightweight and avoids forcing a large microscopic runtime into every deployment.

JuPedSim is distributed under LGPL-3.0-or-later. Any production bundle that redistributes JuPedSim must preserve the applicable LGPL notices and distribution/relinking requirements. Keeping it behind an explicit backend boundary makes that deployment decision visible rather than accidental.
