from __future__ import annotations

from dataclasses import dataclass, replace

from .geometry import Vec3
from .hierarchical_routing import (
    HierarchicalRoute,
    HierarchicalRouteOptions,
    find_hierarchical_path,
)
from .model import InavModel
from .semantic import ensure_semantic_transitions


@dataclass(frozen=True, slots=True)
class RouteProfile:
    """Routing policy applied above the canonical INAV topology.

    Profiles intentionally describe policy, not geometry. They never mutate the
    source INAV model; instead they expose a filtered view of semantic
    transitions. Project-specific accessibility or operational requirements can
    construct custom profiles without changing the generator.
    """

    id: str
    label: str
    description: str
    blocked_transition_kinds: frozenset[str] = frozenset()
    min_portal_width_m: float | None = None
    require_known_portal_width: bool = False


_BUILTIN_PROFILES: tuple[RouteProfile, ...] = (
    RouteProfile(
        id="standard",
        label="Standard",
        description="No additional transition restrictions.",
    ),
    RouteProfile(
        id="accessible",
        label="Accessible / step-free",
        description=(
            "Excludes stairs and escalators. Ramps and elevators remain eligible. "
            "No universal door-width threshold is assumed automatically."
        ),
        blocked_transition_kinds=frozenset({"stair", "escalator"}),
    ),
    RouteProfile(
        id="emergency_responder",
        label="Emergency responder",
        description=(
            "Conservative emergency default that excludes elevators while keeping "
            "stairs, ramps and normal portals eligible. Project policy can override it."
        ),
        blocked_transition_kinds=frozenset({"elevator"}),
    ),
    RouteProfile(
        id="security",
        label="Security",
        description=(
            "No transition restriction by default. Intended as a separate policy hook "
            "for future access-control and credential rules."
        ),
    ),
    RouteProfile(
        id="maintenance",
        label="Maintenance",
        description=(
            "No transition restriction by default. Intended for service circulation "
            "and future equipment/access-control constraints."
        ),
    ),
)

_PROFILE_BY_ID = {profile.id: profile for profile in _BUILTIN_PROFILES}
_PROFILE_ALIASES = {
    "default": "standard",
    "normal": "standard",
    "wheelchair": "accessible",
    "step_free": "accessible",
    "step-free": "accessible",
    "responder": "emergency_responder",
    "firefighter": "emergency_responder",
}


def available_route_profiles() -> tuple[RouteProfile, ...]:
    return _BUILTIN_PROFILES


def resolve_route_profile(profile: RouteProfile | str | None) -> RouteProfile:
    if isinstance(profile, RouteProfile):
        return profile
    key = str(profile or "standard").strip().lower().replace(" ", "_")
    key = _PROFILE_ALIASES.get(key, key)
    try:
        return _PROFILE_BY_ID[key]
    except KeyError as exc:
        available = ", ".join(item.id for item in _BUILTIN_PROFILES)
        raise ValueError(f"Unknown route profile '{profile}'. Available: {available}") from exc


def model_for_route_profile(
    model: InavModel,
    profile: RouteProfile | str | None,
) -> InavModel:
    """Return a non-mutating semantic-policy view of ``model``.

    The metric geometry, cells, nodes and portals are shared with the source
    model. Only the transition list is copied and filtered. This keeps scenario
    state and exported INAV canonical while allowing different users or
    simulations to apply different routing policy.
    """
    resolved = resolve_route_profile(profile)
    profiled = replace(model, transitions=list(model.transitions))
    ensure_semantic_transitions(profiled)
    portals = {portal.id: portal for portal in profiled.portals}

    transitions = []
    for transition in profiled.transitions:
        if transition.kind.lower() in resolved.blocked_transition_kinds:
            continue
        if transition.portal_id and resolved.min_portal_width_m is not None:
            portal = portals.get(transition.portal_id)
            if portal is not None:
                if portal.width_m is None:
                    if resolved.require_known_portal_width:
                        continue
                elif portal.width_m + 1e-9 < resolved.min_portal_width_m:
                    continue
        transitions.append(transition)

    profiled.transitions = transitions
    return profiled


def find_profiled_hierarchical_path(
    model: InavModel,
    start: Vec3,
    goal: Vec3,
    *,
    profile: RouteProfile | str | None = None,
    options: HierarchicalRouteOptions | None = None,
) -> HierarchicalRoute | None:
    return find_hierarchical_path(
        model_for_route_profile(model, profile),
        start,
        goal,
        options,
    )
