from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import ifcopenshell.util.element


@dataclass(frozen=True, slots=True)
class ExitClassification:
    is_exit: bool
    is_external: bool | None
    source: str


def classify_door_exit(connected_space_count: int, psets: dict[str, Any] | None = None) -> ExitClassification:
    """Classify whether a door is an exterior navigation exit.

    ``Pset_DoorCommon.IsExternal`` is authoritative when present. IFC models
    frequently omit or partially export space boundaries, so the previous
    single-connected-space rule remains only as a fallback when no explicit
    externality property is available.
    """
    psets = psets or {}
    common = psets.get("Pset_DoorCommon") or {}
    raw_external = common.get("IsExternal")

    external = _as_bool(raw_external)
    if external is not None:
        return ExitClassification(
            is_exit=external,
            is_external=external,
            source="Pset_DoorCommon.IsExternal",
        )

    if connected_space_count == 1:
        return ExitClassification(
            is_exit=True,
            is_external=None,
            source="single-space-fallback",
        )

    return ExitClassification(
        is_exit=False,
        is_external=None,
        source="not-classified",
    )


def classify_ifc_door_exit(door: Any, connected_space_count: int) -> ExitClassification:
    try:
        psets = ifcopenshell.util.element.get_psets(door)
    except Exception:
        psets = {}
    return classify_door_exit(connected_space_count, psets)


def _as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "t", "yes", "1"}:
            return True
        if normalized in {"false", "f", "no", "0"}:
            return False
    return None
