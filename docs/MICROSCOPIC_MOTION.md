# IFCPath Microscopic Local Motion

IFCPath separates **navigation decisions** from **pedestrian operational motion**.

The existing evacuation engine remains the strategic/tactical layer. It owns BIM semantics, hierarchical routes, exit selection, hazards, blocked spaces/portals and bottleneck policy. A local-motion backend is responsible only for moving people toward IFCPath-supplied local targets in a physically plausible way.

```text
IFC / INAV
   ↓
IFCPath semantic + metric routing
   ↓
route / next local target
   ↓
LocalMotionBackend
   ├── KinematicLocalMotionBackend  (deterministic fallback)
   └── JuPedSimLocalMotionBackend   (optional microscopic solver)
   ↓
agent XYZ + facing
```

This boundary is intentional. IFCPath remains the source of truth for building semantics and routing; an operational crowd solver must not reinterpret rooms, doors, exits or emergency policy.

## Common backend contract

`src/ifcpath/microscopic_motion.py` defines a host-independent `LocalMotionBackend` protocol:

- `add_agent(spec)`
- `set_target(agent_id, target_m)`
- `advance(delta_seconds)`
- `snapshot(agent_id)`
- `snapshots()`

The corresponding data types are:

- `MicroscopicMotionConfig`
- `MicroscopicAgentSpec`
- `MicroscopicAgentSnapshot`

The contract contains no Qt or Unreal dependency.

## Deterministic fallback

`KinematicLocalMotionBackend` moves each agent directly toward its current target at its configured desired speed.

It intentionally provides **no agent-agent collision physics**. Its purpose is to keep the same backend interface available for tests, simple hosts and packaged builds that do not install a microscopic solver.

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

## Level-local scope

The current microscopic backend is deliberately **level-local**.

One `JuPedSimLocalMotionBackend` instance represents one horizontal navigation level. This matches JuPedSim's 2D operational geometry and avoids pretending that a 3D stair polyline is ordinary level-ground crowd geometry.

The next hybrid-runtime slice will own the handoff:

```text
horizontal level backend
        ↓
IFCPath semantic door / stair / ramp gate
        ↓
vertical transition handled by IFCPath
        ↓
next-level microscopic backend
```

Until that handoff controller is implemented, the existing `EvacuationSimulator` remains the production multi-floor evacuation runtime. The JuPedSim backend is real and qualified, but it is not yet presented as full multi-floor microscopic evacuation.

## Qualification

CI has a dedicated `microscopic-tests` job that installs the optional dependency and runs the actual JuPedSim adapter.

The first gate verifies:

- CDT cells reconstruct the expected continuous floor geometry;
- the deterministic fallback implements the same contract;
- JuPedSim direct steering moves an agent toward an IFCPath target;
- two head-on JuPedSim agents maintain separation while both continue making progress.

This is intentionally stronger than an import-only smoke test: the solver iterates and produces motion under interaction.

## Packaging and licensing boundary

JuPedSim is optional rather than part of IFCPath's default dependency set. This keeps the normal generator and Windows Builder lightweight and avoids forcing a large microscopic runtime into every deployment.

JuPedSim is distributed under LGPL-3.0-or-later. Any production bundle that redistributes JuPedSim must preserve the applicable LGPL notices and distribution/relinking requirements. Keeping it behind an explicit backend boundary makes that deployment decision visible rather than accidental.
