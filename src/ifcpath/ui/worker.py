from __future__ import annotations

import traceback
from pathlib import Path

from PySide6.QtCore import QObject, Signal, Slot

from ..exporter import load_inav
from ..ifc_loader import BuildOptions, build_from_ifc
from ..validation import validate_model


class ModelWorker(QObject):
    """Build or load an INAV model away from the Qt GUI thread."""

    finished = Signal(object, object, str)
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
            if self.operation == "build":
                model = build_from_ifc(Path(self.path), self.options or BuildOptions())
            elif self.operation == "load":
                model = load_inav(Path(self.path))
            else:
                raise ValueError(f"Unsupported desktop operation: {self.operation}")

            report = validate_model(model)
            self.finished.emit(model, report, self.path)
        except Exception as exc:  # keep the GUI alive and show diagnostics
            self.failed.emit(str(exc), traceback.format_exc())
