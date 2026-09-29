# Route profiles

IFCPath route profiles are **runtime policy over the canonical INAV topology**.
They do not rewrite BIM geometry, delete navigation cells, or mutate the exported
`.inav` file. A profile filters semantic transitions for one routing/simulation
request while the source navigation model remains unchanged.

## Built-in profiles

| Profile | Default transition policy |
| --- | --- |
| `standard` | No additional restrictions |
| `accessible` | Excludes `stair` and `escalator`; ramps and elevators remain eligible |
| `emergency_responder` | Conservative default that excludes `elevator`; stairs and ramps remain eligible |
| `security` | No restriction yet; separate hook for future access-control policy |
| `maintenance` | No restriction yet; separate hook for service/equipment policy |

The built-ins intentionally avoid hard-coding jurisdiction-specific accessibility
or life-safety rules. For example, the accessible preset does **not** assume a
universal minimum door width because codes, occupancy, project requirements and
BIM completeness vary.

## Custom profiles

A project can construct `RouteProfile` directly:

```python
from ifcpath.route_profiles import RouteProfile

wide_step_free = RouteProfile(
    id="wide-step-free",
    label="Wide step-free",
    description="Project-specific mobility policy",
    blocked_transition_kinds=frozenset({"stair", "escalator"}),
    min_portal_width_m=0.90,
    require_known_portal_width=True,
)
```

`require_known_portal_width=True` is fail-closed: a semantic portal with missing
width is not accepted when a width threshold is active.

## API

```python
from ifcpath.route_profiles import find_profiled_hierarchical_path

route = find_profiled_hierarchical_path(
    model,
    start_xyz,
    goal_xyz,
    profile="accessible",
    options=scenario_options,
)
```

For evacuation and microscopic simulation, use `model_for_route_profile()` to
create a non-mutating semantic-policy view and pass that model to the existing
simulator. The desktop Builder does this automatically.

## Desktop Builder

The Builder exposes **Route profile → User / mission policy**. Changing the
profile recalculates the current Start→Goal route immediately. If an evacuation
simulation is already prepared, it is reset and must be prepared again so the
simulator cannot continue with a stale policy view.

Live blocked doors/spaces and hazard-cost multipliers still apply on top of the
selected profile.

## Scope

Profiles currently govern semantic transition eligibility and optional portal
width requirements. Future extensions can add:

- credential/access-zone constraints;
- per-agent mobility classes in mixed populations;
- elevator availability/service mode;
- one-way operational circulation;
- equipment/clearance constraints for maintenance routes;
- project-specific transition cost preferences.
