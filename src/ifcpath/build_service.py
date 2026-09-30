from __future__ import annotations

from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any

from .exporter import model_from_dict
from .ifc_loader import BuildOptions, build_from_ifc
from .surface_fragments import prune_tiny_space_fragments
from .surface_reconstruction import reconstruct_walkable_surface
from .validation import validate_model


class IfcBuildRequestError(ValueError):
    """Raised when an uploaded IFC cannot be accepted by the local app."""


def build_inav_payload(
    data: bytes,
    filename: str = "upload.ifc",
    *,
    options: BuildOptions | None = None,
    max_bytes: int = 256 * 1024 * 1024,
) -> dict[str, Any]:
    """Build and qualify portable INAV directly from uploaded IFC bytes.

    The legacy importer still extracts IFC semantics and supplies a qualified
    fallback surface. Before portable semantic finalization, the local app now
    attempts to replace those cells with a physical support surface reconstructed
    from the complete IFC geometry. This makes floors/stairs/ramps and collision
    clearance authoritative while retaining a deterministic fallback during
    corpus qualification.
    """
    if not data:
        raise IfcBuildRequestError("IFC upload is empty")
    if len(data) > max_bytes:
        raise IfcBuildRequestError(
            f"IFC upload is too large ({len(data) / (1024 * 1024):.1f} MiB); "
            f"local-app limit is {max_bytes / (1024 * 1024):.0f} MiB"
        )
    header = data[:4096].lstrip(b"\xef\xbb\xbf\x00\t\r\n ")
    if not header.startswith(b"ISO-10303-21;"):
        raise IfcBuildRequestError("upload is not an IFC STEP file (missing ISO-10303-21 header)")

    safe_name = Path(filename or "upload.ifc").name
    suffix = Path(safe_name).suffix if Path(safe_name).suffix.lower() == ".ifc" else ".ifc"
    temp_path: Path | None = None
    try:
        with NamedTemporaryFile(prefix="ifcpath-upload-", suffix=suffix, delete=False) as stream:
            stream.write(data)
            temp_path = Path(stream.name)

        resolved_options = options or BuildOptions()
        raw_model = build_from_ifc(temp_path, resolved_options)
        surface_stats = reconstruct_walkable_surface(
            temp_path,
            raw_model,
            cell_size_m=min(max(resolved_options.stair_spacing_m, 0.10), 0.20),
            agent_height_m=resolved_options.agent_height_m,
            # A zero-radius centreline navmesh is exactly the behaviour that let
            # routes touch furniture/railings. Use a human-body default unless
            # the caller explicitly requests a larger clearance.
            agent_radius_m=max(0.22, resolved_options.agent_clearance_m),
            max_slope_deg=resolved_options.max_slope_deg,
            max_climb_m=0.24,
        )
        if surface_stats.replaced_legacy_surface:
            # Grid/BRep intersections can leave tiny detached triangles inside a
            # correctly labelled room. Before semantic binding, authored portal
            # proximity is conservative evidence that a tiny patch could be a real
            # threshold, so keep it for the first finalisation pass.
            fragment_stats = prune_tiny_space_fragments(raw_model.cells, raw_model)
            raw_model.metadata["surface_fragment_pruning"] = {
                "removed_cells": fragment_stats.removed_cells,
                "removed_components": fragment_stats.removed_components,
            }
            raw_model.metadata["cell_count"] = len(raw_model.cells)
        else:
            raw_model.metadata["surface_reconstruction"] = surface_stats.to_dict()
            raw_model.metadata.setdefault("surface_source", "legacy-qualified-fallback")

        # First semantic pass binds real door/open-boundary crossings and derives
        # vertical resources from the reconstructed physical manifold.
        model = model_from_dict(raw_model.to_dict())

        if surface_stats.replaced_legacy_surface:
            # Now that portal_ids are authoritative, portal *proximity* alone is no
            # longer a reason to retain a tiny island. This second conservative pass
            # removes an unused door-near sampling sliver while explicitly protecting
            # any component that participates in a bound semantic crossing or actual
            # stair/ramp/escalator connection.
            post_stats = prune_tiny_space_fragments(
                model.cells,
                model,
                protect_portal_proximity=False,
                protect_bound_portals=True,
            )
            model.metadata["surface_postbind_fragment_pruning"] = {
                "removed_cells": post_stats.removed_cells,
                "removed_components": post_stats.removed_components,
            }
            model.metadata["cell_count"] = len(model.cells)
            if post_stats.removed_cells:
                # Re-run the shared idempotent portable finalisation so door/open
                # boundary and surface-vertical semantics are rebuilt against the
                # cleaned authoritative cell set.
                model = model_from_dict(model.to_dict())

        model.metadata["source_ifc"] = safe_name
        model.metadata["source"] = "local-app-upload"
        report = validate_model(model)
        return {
            "model": model.to_dict(),
            "qualification": report.to_dict(),
        }
    except IfcBuildRequestError:
        raise
    except Exception as exc:
        raise IfcBuildRequestError(f"IFC build failed: {exc}") from exc
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
