from __future__ import annotations

import json
from pathlib import Path

from .model import InavModel, Level, NavEdge, NavNode, Portal, Space


def save_inav(model: InavModel, path: str | Path) -> Path:
    path = Path(path)
    if path.suffix.lower() != ".inav":
        path = path.with_suffix(".inav")
    path.write_text(json.dumps(model.to_dict(), indent=2), encoding="utf-8")
    return path


def load_inav(path: str | Path) -> InavModel:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return InavModel(
        schema=raw.get("schema", "ifcpath.inav/0.1"),
        units=raw.get("units", "m"),
        up_axis=raw.get("up_axis", "Z"),
        levels=[Level(**x) for x in raw.get("levels", [])],
        spaces=[Space(**_tuple_vec(x, "centroid_m")) for x in raw.get("spaces", [])],
        portals=[Portal(**_tuple_vec(x, "position_m")) for x in raw.get("portals", [])],
        nodes=[NavNode(**_tuple_vec(x, "position_m")) for x in raw.get("nodes", [])],
        edges=[NavEdge(**x) for x in raw.get("edges", [])],
        metadata=raw.get("metadata", {}),
    )


def _tuple_vec(value: dict, key: str) -> dict:
    value = dict(value)
    if value.get(key) is not None:
        value[key] = tuple(value[key])
    return value
