from __future__ import annotations

import json
from pathlib import Path

from .model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, SemanticTransition, Space
from .open_space_adjacency import connect_open_space_boundaries
from .semantic import ensure_semantic_transitions
from .surface_portals import bind_semantic_surface_portals
from .vertical_surface import ensure_surface_vertical_transitions


def _ensure_portable_semantics(model: InavModel) -> None:
    # First recover only exact/coincident open-plan space boundaries. Wider
    # geometric gaps require loader-side wall qualification and are deliberately
    # not inferred here from portable data alone.
    open_stats = connect_open_space_boundaries(
        model,
        [],
        max_gap_m=1e-4,
        max_vertical_gap_m=1e-4,
    )
    if open_stats.connected:
        model.metadata["surface_open_boundary_count"] = sum(
            portal.kind == "open_boundary" for portal in model.portals
        )

    # Surface-derived stairs/ramps are authoritative for walkable vertical
    # circulation. Add those semantic transitions before the sampled-node
    # compatibility inference so the portable file preserves them explicitly.
    ensure_surface_vertical_transitions(model)
    ensure_semantic_transitions(model)
    bind_semantic_surface_portals(model)


def save_inav(model: InavModel, path: str | Path) -> Path:
    _ensure_portable_semantics(model)
    path = Path(path)
    if path.suffix.lower() != ".inav":
        path = path.with_suffix(".inav")
    path.write_text(json.dumps(model.to_dict(), indent=2), encoding="utf-8")
    return path


def model_from_dict(raw: dict) -> InavModel:
    """Build an INAV model from a JSON-compatible dictionary.

    This is the shared deserializer used by file loading and the live study API,
    keeping browser-posted models on exactly the same semantic/portal binding path
    as on-disk ``.inav`` files.
    """
    model = InavModel(
        schema=raw.get("schema", "ifcpath.inav/0.1"),
        units=raw.get("units", "m"),
        up_axis=raw.get("up_axis", "Z"),
        levels=[Level(**x) for x in raw.get("levels", [])],
        spaces=[Space(**_tuple_vec(x, "centroid_m")) for x in raw.get("spaces", [])],
        portals=[Portal(**_tuple_vec(x, "position_m")) for x in raw.get("portals", [])],
        transitions=[SemanticTransition(**x) for x in raw.get("transitions", [])],
        cells=[NavCell(**_cell_value(x)) for x in raw.get("cells", [])],
        nodes=[NavNode(**_tuple_vec(x, "position_m")) for x in raw.get("nodes", [])],
        edges=[NavEdge(**x) for x in raw.get("edges", [])],
        metadata=raw.get("metadata", {}),
    )
    _ensure_portable_semantics(model)
    return model


def load_inav(path: str | Path) -> InavModel:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return model_from_dict(raw)


def _tuple_vec(value: dict, key: str) -> dict:
    value = dict(value)
    if value.get(key) is not None:
        value[key] = tuple(value[key])
    return value


def _cell_value(value: dict) -> dict:
    value = dict(value)
    value["vertices_m"] = tuple(tuple(vertex) for vertex in value.get("vertices_m", ()))
    value["neighbor_ids"] = list(value.get("neighbor_ids", ()))
    value["portals"] = {
        neighbor: (tuple(segment[0]), tuple(segment[1]))
        for neighbor, segment in value.get("portals", {}).items()
    }
    value["portal_ids"] = dict(value.get("portal_ids", {}))
    return value
