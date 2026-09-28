# IFCPath Multi-Agent Evacuation Simulation

The desktop Builder includes a deterministic mesoscopic evacuation simulator on top of the same INAV hierarchy used for single-person routing.

## What it models

Each occupant has:

- an exact 3D start point on a walkable CDT cell;
- an individual free walking speed;
- an exact hierarchical route to a classified exit;
- local funnel-smoothed movement inside spaces;
- door/stair/ramp transfers;
- queueing and service at capacity-constrained bottlenecks;
- a live 3D pose and facing direction;
- evacuation time and accumulated waiting time.

Exit selection is capacity-aware. When several exits/routes are possible, occupants are assigned sequentially using free travel time plus projected queue delay on the route's bottlenecks. This avoids forcing an entire population toward the same geometrically shortest exit when alternatives exist.

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

The population speed range is configurable. The simulator applies additional configurable factors to stair and ramp route segments, so a person's stair travel is slower than their own free level-ground speed while still following the actual 3D stair geometry.

The default stair speed factor is `0.75`. This is a modeling parameter, not a universal constant, and should be calibrated for a real safety study.

## Deterministic automatic population

The Builder generates occupant positions inside walkable CDT triangles using a deterministic random seed. A minimum spawn spacing is attempted to avoid identical starts. Reusing the same model, population settings and seed produces the same population.

Automatic/demo populations are sampled only from semantic spaces that have a **baseline route to at least one classified exit**. This matters because IFC navigation models can legitimately contain roof areas, terraces, service voids or other walkable geometry that should not receive arbitrary synthetic occupants.

This filter does **not** hide real egress failures. Explicitly supplied occupants are never filtered: if a person is placed in a genuinely unreachable occupied space, the simulator reports that person as `trapped`. The automatic filter can also be disabled in code with `egress_reachable_only=False` for diagnostic populations.

This makes scenario A/B comparisons reproducible without allowing incidental non-occupancy navigation regions to dominate the synthetic population.

## Live Digital Twin scenarios

Blocked portals, blocked spaces and Smoke / Fire / Crowd route-cost multipliers remain runtime state and never modify INAV.

If scenario state changes during a multi-person evacuation, every non-evacuated occupant is replanned from its exact current XYZ under the new state. Already evacuated occupants remain evacuated.

## Metrics

The Builder reports:

- elapsed simulation time;
- total / evacuated / active / waiting / trapped occupants;
- maximum observed bottleneck queue, retained across live replans;
- average evacuation time;
- final clearance time;
- exit usage counts.

## Model boundary

This simulator is **mesoscopic**, not a contact/collision crowd-physics solver.

It is intended for fast BIM/Digital Twin analysis of:

- route availability;
- multi-exit distribution;
- door/stair/exit bottlenecks;
- queue delay;
- dynamic hazards and blocked routes;
- evacuation-time comparisons.

It does not yet model shoulder-to-shoulder avoidance, pushing, body compression, lane formation, detailed social-force dynamics or calibrated demographic behavior. A future microscopic backend such as JuPedSim can be connected beneath the same INAV semantic/metric model without replacing the IFCPath generator.

## Real IFC qualification

GitHub Actions builds the buildingSMART Duplex IFC, validates the INAV model, then creates 24 deterministic occupants in baseline egress-reachable spaces and runs a crowd evacuation regression. The qualification records evacuation counts, clearance metrics, queues and exit usage and fails if the real-model population cannot evacuate at an acceptable rate.
