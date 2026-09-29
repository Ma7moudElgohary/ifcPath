# IFCPath Multi-Agent Evacuation Simulation

The desktop Builder exposes two complementary simulation layers on top of the same INAV hierarchy:

1. a **mesoscopic** route / capacity / queue simulator for fast BIM and Digital Twin studies;
2. a **hybrid building-wide** runtime that keeps IFCPath in charge of routing, gates and semantic transfers while delegating local pedestrian interaction to a replaceable motion backend.

The default remains the mesoscopic model because it is deterministic, lightweight and fast.

## Shared strategic model

All modes use the same IFCPath source of truth for:

- exact 3D occupant start points on walkable CDT cells;
- individual free walking speeds;
- hierarchical semantic + metric routes to classified exits;
- funnel-smoothed local paths inside spaces;
- door, stair, ramp, elevator and escalator transitions;
- capacity-aware exit assignment;
- blocked portals / spaces and runtime route-cost multipliers;
- transition and exit bottleneck capacities;
- evacuation time, waiting time and exit usage metrics.

Exit selection is capacity-aware. When several exits/routes are possible, occupants are assigned sequentially using free travel time plus projected queue delay on the route's bottlenecks. This avoids forcing an entire population toward the same geometrically shortest exit when alternatives exist.

## Builder movement modes

The **Movement model** selector contains:

- **Mesoscopic (fast)** — occupants advance on exact IFCPath route polylines; queues and capacities are modeled, but there is no shoulder-to-shoulder interaction.
- **Hybrid deterministic** — uses the new building-wide semantic-handoff coordinator with the dependency-free kinematic local backend. This exercises the same runtime architecture as microscopic mode without claiming crowd physics.
- **Hybrid microscopic (JuPedSim)** — local movement inside connected semantic spaces uses the optional JuPedSim operational solver, while IFCPath continues to own walls/portal boundaries, transition queues and exact transfer geometry.

JuPedSim remains an optional dependency. A normal Builder installation can therefore remain lightweight. Selecting microscopic mode in a build without JuPedSim gives a clear message rather than crashing.

## Hybrid execution model

A hierarchical evacuation plan is compiled into explicit execution steps:

```text
local route in semantic space
        ↓
transition capacity gate / queue
        ↓
exact IFCPath transfer route
        ↓
destination semantic-space motion domain
        ↓
...
        ↓
exit capacity gate
```

Every connected semantic motion domain has its own shared local-motion backend, so occupants in the same room/corridor/open area interact in the same microscopic simulation.

### Same-floor doors and openings

Same-floor semantic portals are explicit handoffs too. This is important because two `IfcSpace` CDT polygons can be separated by wall thickness. JuPedSim requires connected accessible geometry, and artificially bridging disconnected room polygons would create a risk of walking through walls or bypassing a blocked door.

The hybrid runtime therefore:

1. moves the occupant to the transition approach point inside the origin space;
2. applies the transition capacity gate;
3. removes the occupant from the origin local solver;
4. traverses the exact semantic transfer polyline;
5. injects the occupant into the destination-space solver.

### Vertical circulation

Stairs, ramps, elevators and escalators use the same handoff lifecycle, but their transfer route can be fully 3D and uses the configured vertical speed factor.

This avoids flattening multi-storey BIM geometry into a single 2D crowd domain.

## Bottleneck capacity

Door and final-exit capacity is calculated from effective width:

```text
capacity [persons/s] = specific flow [persons/s/m] × portal width [m]
```

The default specific flow is `1.30 persons/s/m`, matching the commonly used maximum engineering doorway specific-flow value discussed in NIST egress literature. Portal widths come from IFC where available; otherwise the model uses a configurable fallback width.

Stair, ramp and other vertical-transition capacities are separate configurable values because observed stair flow is more variable and depends on geometry, merging and occupant population.

References:

- NIST / Fire Safety Journal, *Verification and validation of selected fire models for nuclear power plant applications*, doorway/corridor/aisle/ramp specific-flow discussion: https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=861412
- NIST TN 1839, *Movement on Stairs During Building Evacuations*: https://nvlpubs.nist.gov/nistpubs/TechnicalNotes/NIST.TN.1839.pdf
- Boguslawski et al., dynamic evacuee distribution/routing with node/link capacity and transient density: https://doi.org/10.1016/j.autcon.2018.05.032

## Walking speed

The population speed range is configurable. The simulator applies additional configurable factors to stair and ramp route segments, so a person's vertical travel is slower than their own free level-ground speed while still following the actual 3D transition geometry.

The default stair speed factor is `0.75`. This is a modeling parameter, not a universal constant, and should be calibrated for a real safety study.

## Deterministic automatic population

The Builder generates occupant positions inside walkable CDT triangles using a deterministic random seed. A minimum spawn spacing is attempted to avoid identical starts. Reusing the same model, population settings and seed produces the same population.

Automatic/demo populations are sampled only from semantic spaces that have a **baseline route to at least one classified exit**. This matters because IFC navigation models can legitimately contain roof areas, terraces, service voids or other walkable geometry that should not receive arbitrary synthetic occupants.

This filter does **not** hide real egress failures. Explicitly supplied occupants are never filtered: if a person is placed in a genuinely unreachable occupied space, the simulator reports that person as `trapped`. The automatic filter can also be disabled in code with `egress_reachable_only=False` for diagnostic populations.

## Live Digital Twin scenarios

Blocked portals, blocked spaces and Smoke / Fire / Crowd route-cost multipliers remain runtime state and never modify INAV.

If scenario state changes during a multi-person evacuation, every non-evacuated occupant is replanned from its exact current XYZ under the new state. Already evacuated occupants remain evacuated.

For the hybrid runtime, active microscopic domains are rebuilt during replan so stale native solver agents from the old route cannot survive a scenario change.

## Metrics

All modes expose the same high-level metrics interface:

- elapsed simulation time;
- total / evacuated / active / waiting / trapped occupants;
- maximum observed bottleneck queue, retained across live replans;
- average evacuation time;
- final clearance time;
- exit usage counts.

This lets the Builder compare the fast mesoscopic model and the more detailed hybrid/microscopic model without changing the surrounding Digital Twin UI contract.

## Model boundary

The mesoscopic mode is intended for fast BIM/Digital Twin analysis of:

- route availability;
- multi-exit distribution;
- door/stair/exit bottlenecks;
- queue delay;
- dynamic hazards and blocked routes;
- evacuation-time comparisons.

Hybrid microscopic mode adds semantic-domain pedestrian interaction and collision avoidance through JuPedSim. It still does **not** claim to model pushing, body compression, panic, calibrated demographic distributions, smoke toxicity, visibility impairment or CFD. Those effects require separately validated models and scenario inputs.

## Qualification

The normal CI suite verifies the dependency-free simulator, hybrid handoff architecture, Qt Builder integration and Windows packaging.

A dedicated microscopic CI job installs the actual JuPedSim 1.4.2 wheel and verifies real solver motion, including:

- upper-floor → capacity gate → true 3D stair → lower-floor solver → exit;
- disconnected same-floor room domains crossed through an explicit semantic door handoff instead of a fake connected floor polygon.

The existing buildingSMART Duplex workflow remains the real-IFC regression for generation, validation and multi-person evacuation. It intentionally uses the default dependency set so the core product does not become dependent on the optional microscopic solver.
