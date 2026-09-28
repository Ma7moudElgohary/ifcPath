# IFCPath Microscopic / Hybrid Motion

IFCPath separates **navigation decisions** from **pedestrian operational motion**.

IFCPath owns BIM semantics, hierarchical routes, exit selection, hazards, blocked spaces/portals, bottleneck capacities and vertical-transition policy. A local-motion backend is responsible only for moving people toward IFCPath-supplied local targets inside a horizontal level.

```text
IFC / INAV
   ↓
IFCPath semantic + metric routing
   ↓
HybridEvacuationSimulator
   ├── horizontal level leg ──→ LocalMotionBackend
   │                              ├── Kinematic fallback
   │                              └── JuPedSim
   ├── semantic transition gate / queue
   └── actual 3D stair / ramp / elevator transfer
                    ↓
             destination level
```

This boundary is intentional. An operational crowd solver must not reinterpret rooms, doors, exits, stairs or emergency policy.

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

`remove_agent()` is part of the contract because a building-wide simulation must remove a pedestrian from one level solver before handing that same logical occupant to a stair/ramp/elevator transfer and then to another level solver.

## Route-driven local motion

`MicroscopicRouteController` converts an IFCPath route polyline into sequential local targets. It owns waypoint progression, not pathfinding. It can also replace the remaining route from an agent's exact current position after a live scenario change.

All occupants assigned to the same level backend are advanced together, preserving solver interaction on that level.

## Deterministic fallback

`KinematicLocalMotionBackend` moves each agent directly toward its current target at its configured desired speed.

It intentionally provides **no agent-agent collision physics**. Its purpose is to keep the same hybrid handoff architecture available for tests, debugging and default packaged builds that do not install a microscopic solver.

## JuPedSim backend

`JuPedSimLocalMotionBackend` is an optional adapter qualified against **JuPedSim 1.4.2**.

Install it with:

```bash
pip install -e ".[microscopic]"
```

The backend currently uses JuPedSim's collision-free speed model V2 by default and creates a **direct-steering stage**. IFCPath therefore retains route choice and updates an agent's target, while JuPedSim resolves operational pedestrian interaction and collision avoidance inside the floor geometry.

A V3 operational model can be selected through `MicroscopicMotionConfig(model="cfsm_v3")` when supported by the installed JuPedSim build.

## Geometry transfer

JuPedSim operates on continuous 2D walkable geometry. IFCPath already owns the qualified floor CDT cells, so no second BIM interpretation is introduced.

For a level:

```text
qualified IFCPath CDT triangles
          ↓ unary union
continuous Shapely walkable geometry
          ↓
JuPedSim Simulation geometry
```

`build_level_walkable_geometry()` reconstructs that floor domain from the existing INAV cells. Agent starts and direct-steering targets are projected inside the walkable domain when necessary so small boundary numerical differences do not create invalid JuPedSim placements.

## Hybrid multi-floor coordinator

`src/ifcpath/hybrid_evacuation.py` implements the building-wide coordinator.

The coordinator deliberately does **not** flatten a multi-storey building into one 2D solver. Instead it compiles every `EvacuationPlan` into execution steps:

```text
level-local route
→ transition capacity gate
→ exact 3D vertical-transfer route
→ next-level local route
→ ...
→ exit capacity gate
```

For each horizontal level, one shared `MicroscopicRouteController` / local-motion backend is used by all active occupants on that level.

For stairs, ramps, elevators and escalators:

1. the agent reaches the semantic transition;
2. the transition's capacity gate controls admission and queueing;
3. the agent is removed from the origin-level local-motion backend;
4. IFCPath advances the occupant along the transition's actual 3D route polyline using its configured vertical speed factor;
5. the occupant is injected into the destination-level local-motion backend and continues on the existing hierarchical plan.

This keeps semantic capacity, 3D BIM geometry and crowd interaction in the layer that actually owns each responsibility.

## Live replanning

When blocked portals, blocked spaces or runtime hazard/crowd costs change, the hybrid coordinator replans every non-evacuated occupant from its **exact live XYZ**.

The active per-level solver set is rebuilt so native microscopic agents from the previous tactical routes cannot remain as stale solver state. Already evacuated occupants remain evacuated.

## Builder modes

The desktop Builder exposes three movement modes:

- **Mesoscopic (fast)** — existing deterministic route / queue / capacity simulator; default.
- **Hybrid deterministic** — same multi-floor coordinator and handoffs, but uses the dependency-free kinematic local backend.
- **Hybrid microscopic (JuPedSim)** — uses JuPedSim on horizontal levels and IFCPath for gates and vertical handoff.

Selecting JuPedSim in a build where the optional dependency is not installed produces a clear UI message instead of a crash. The default Windows package therefore remains lightweight; a deployment that wants microscopic physics can install/package the `microscopic` extra explicitly.

## Qualification

CI keeps separate gates for the normal product and optional solver.

The microscopic/hybrid qualification currently verifies:

- CDT cells reconstruct the expected continuous floor geometry;
- the deterministic fallback implements the same local-motion contract;
- route waypoint turns and live route replacement;
- real JuPedSim direct steering;
- two head-on JuPedSim agents maintain separation while making progress;
- a hierarchical route compiles a capacity gate before a true 3D stair transfer;
- deterministic upper-floor → stair → lower-floor → exit handoff;
- stair capacity creates waiting/queue behaviour;
- hybrid replanning starts from the exact live occupant position;
- a real JuPedSim agent is removed from the upper-level solver, traverses the 3D vertical transition, enters the lower-level solver and evacuates.

The ordinary Python and desktop suites still run without JuPedSim, preserving the optional dependency boundary.

## Packaging and licensing boundary

JuPedSim is optional rather than part of IFCPath's default dependency set. This keeps the normal generator and Windows Builder lightweight and avoids forcing a large microscopic runtime into every deployment.

JuPedSim is distributed under LGPL-3.0-or-later. Any production bundle that redistributes JuPedSim must preserve the applicable LGPL notices and distribution/relinking requirements. Keeping it behind an explicit backend boundary makes that deployment decision visible rather than accidental.
