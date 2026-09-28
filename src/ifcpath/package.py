from __future__ import annotations

import json
from pathlib import Path

from .models import NavigationModel


def _document(model: NavigationModel, source_name: str | None = None) -> dict:
    return {
        "format": "IFCPath INAV",
        "schemaVersion": model.schema_version,
        "source": source_name,
        "coordinateSystem": {
            "handedness": "right",
            "upAxis": model.up_axis,
            "units": model.units,
        },
        "navigation": model.model_dump(mode="json"),
    }


def write_inav(model: NavigationModel, output: str | Path, source_name: str | None = None) -> Path:
    output = Path(output)
    if output.suffix.lower() != ".inav":
        output = output.with_suffix(".inav")
    output.write_text(json.dumps(_document(model, source_name), indent=2), encoding="utf-8")
    return output


def read_inav(path: str | Path) -> NavigationModel:
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    if doc.get("format") != "IFCPath INAV":
        raise ValueError("Not an IFCPath INAV document")
    return NavigationModel.model_validate(doc["navigation"])
