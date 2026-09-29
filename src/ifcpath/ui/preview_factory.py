from __future__ import annotations

import os
from typing import Literal

from PySide6.QtWidgets import QWidget

PreviewKind = Literal["base", "scenario", "evacuation"]


def create_preview(kind: PreviewKind, parent=None) -> QWidget:
    """Create the requested preview with an explicit fail-safe GPU opt-in.

    ``IFCPATH_VIEWPORT=gpu`` selects the new WebGPU backend. Any import, adapter,
    surface or device failure falls back to the established Qt projection.
    ``auto`` intentionally resolves to Qt until Windows hardware qualification is
    complete. A successfully created GPU renderer advertises a larger bounded IFC
    preview budget; a fallback Qt renderer keeps the original 18k budget.
    """

    preference = os.environ.get("IFCPATH_VIEWPORT", "qt").strip().lower()
    if preference in {"gpu", "webgpu", "wgpu"}:
        try:
            if kind in {"scenario", "evacuation"}:
                from .gpu_selectable_preview import GpuSelectableScenarioPreview

                preview = GpuSelectableScenarioPreview(parent)
            else:
                from .gpu_preview import GpuBimPreview

                preview = GpuBimPreview(parent)
            setattr(preview, "gpu_requested", True)
            setattr(preview, "preferred_preview_triangle_budget", 250_000)
            return preview
        except Exception as exc:
            preview = _qt_preview(kind, parent)
            setattr(preview, "gpu_requested", True)
            setattr(preview, "gpu_fallback_reason", str(exc))
            return preview

    preview = _qt_preview(kind, parent)
    setattr(preview, "gpu_requested", False)
    return preview


def _qt_preview(kind: PreviewKind, parent=None) -> QWidget:
    if kind == "evacuation":
        from .evacuation_preview import Evacuation3DPreview

        return Evacuation3DPreview(parent)
    if kind == "scenario":
        from .scenario_preview import Scenario3DPreview

        return Scenario3DPreview(parent)

    from .preview_3d import Projected3DPreview

    return Projected3DPreview(parent)
