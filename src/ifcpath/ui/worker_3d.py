from __future__ import annotations

import traceback
from pathlib import Path

from PySide6.QtCore import QObject, Signal, Slot

from ..exporter import load_inav
from ..ifc_loader import BuildOptions, build_from_ifc
from ..validation import validate_model
from .preview_geometry import extract_preview_geometry


class Builder3DWorker(QObject):
    """Build/load navigation and optional IFC display geometry off the GUI thread."""

    finished = Signal(object, object, object, str)
    failed = Signal(str, str)

    def __init__(
        self,
        *,
        operation: str,
        path: str,
        options: BuildOptions | None = None,
    ) -> None:
        super().__init__()
        self.operation = operation
        self.path = path
        self.options = options

    @Slot()
    def run(self) -> None:
        try:
            preview_geometry = None
            if self.operation == "build":
                model = build_from_ifc(Path(self.path), self.options or BuildOptions())
                preview_geometry = extract_preview_geometry(Path(self.path))
            elif self.operation == "load":
                model = load_inav(Path(self.path))
            else:
                raise ValueError(f"Unsupported desktop operation: {self.operation}")

            report = validate_model(model)
            self.finished.emit(model, report, preview_geometry, self.path)
        except Exception as exc:
            self.failed.emit(str(exc), traceback.format_exc())
