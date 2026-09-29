from __future__ import annotations

from dataclasses import dataclass

from .model import InavModel


@dataclass(frozen=True, slots=True)
class EgressDomainStats:
    required: int
    exempt_external: int
    exempt_service: int


def classify_egress_domains(model: InavModel) -> EgressDomainStats:
    """Classify which surfaced spaces are required for occupant-egress readiness.

    Navigation and maintenance routing keep every space. This classification only
    decides whether a space must be able to reach an occupant exit for the model
    to be declared evacuation-ready.

    Explicit IFC exterior classification is authoritative. As a conservative
    fallback for IFC2x3 models that do not classify service domains, an *isolated*
    space on a roof-named building storey is treated as a service/maintenance
    domain. The isolation requirement is important: a roof terrace or penthouse
    that has an authored door/stair/other semantic transition remains in the
    occupant-egress domain.
    """
    levels = {level.id: level for level in model.levels}
    semantically_accessed: set[str] = set()
    for portal in model.portals:
        semantically_accessed.update(
            side for side in (portal.from_space_id, portal.to_space_id) if side
        )
    for transition in model.transitions:
        semantically_accessed.add(transition.from_space_id)
        if transition.to_space_id:
            semantically_accessed.add(transition.to_space_id)

    external = 0
    service = 0
    for space in model.spaces:
        if space.is_external:
            space.egress_required = False
            external += 1
            continue
        # Preserve any explicit/non-default exemption supplied by a producer.
        if not space.egress_required:
            continue
        level = levels.get(space.level_id)
        level_name = (level.name if level else "").strip().casefold()
        isolated = space.id not in semantically_accessed
        if isolated and _is_roof_service_level(level_name):
            space.egress_required = False
            service += 1

    required = sum(space.egress_required and not space.is_external for space in model.spaces)
    model.metadata["egress_required_space_count"] = required
    model.metadata["egress_external_exempt_space_count"] = external
    model.metadata["egress_service_exempt_space_count"] = service
    return EgressDomainStats(required, external, service)


def _is_roof_service_level(name: str) -> bool:
    if not name:
        return False
    tokens = {token for token in name.replace("/", " ").replace("-", " ").split() if token}
    return "roof" in tokens or name.startswith("roof ") or name.endswith(" roof")
