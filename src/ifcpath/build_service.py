from __future__ import annotations

from pathlib import Path
from tempfile import NamedTemporaryFile
from time import perf_counter
from typing import Any

from .exporter import model_from_dict
from .ifc_loader import BuildOptions, build_from_ifc
from .semantic_skeleton import build_semantic_skeleton_from_ifc
from .surface_fragments import prune_tiny_space_fragments
from .surface_reconstruction import reconstruct_walkable_surface
from .validation import validate_model


class IfcBuildRequestError(ValueError):
    """Raised when an uploaded IFC cannot be accepted by the local app."""


def _log_build_stage(name: str, seconds: float, **details: object) -> None:
    detail_text = " ".join(f"{key}={value}" for key, value in sorted(details.items()))
    suffix = f" {detail_text}" if detail_text else ""
    print(f"[ifcpath-build] stage={name} seconds={seconds:.3f}{suffix}", flush=True)


def build_inav_payload(
    data: bytes,
    filename: str = "upload.ifc",
    *,
    options: BuildOptions | None = None,
    max_bytes: int = 256 * 1024 * 1024,
) -> dict[str, Any]:
    """Build and qualify portable INAV directly from uploaded IFC bytes.

    The local app first extracts a semantic skeleton (levels, spaces and authored
    doors) and lets complete IFC geometry create the authoritative physical
    surface. Building the old sampled/CDT compatibility graph first is wasted work
    when that graph is immediately replaced, especially in the frozen Windows app.

    Safety is preserved in both exceptional cases: IFCs containing elevators use
    the full importer until elevator semantics have a graph-free extraction path,
    and a failed physical reconstruction triggers the full legacy importer before
    returning a qualified fallback surface.
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
    build_started = perf_counter()
    stage_seconds: dict[str, float] = {}
    try:
        with NamedTemporaryFile(prefix="ifcpath-upload-", suffix=suffix, delete=False) as stream:
            stream.write(data)
            temp_path = Path(stream.name)

        resolved_options = options or BuildOptions()
        # A 0.20 m clearance field can collapse a physically passable ~1 m neck
        # to a single sample row after a 0.22 m body-radius erosion. One row has
        # no area and therefore cannot form NavCells. Cap the authoritative field
        # at 0.15 m: this preserves two-dimensional passage geometry without the
        # roughly 4x XY workload of immediately dropping to a 0.10 m global grid.
        surface_cell_size_m = min(max(resolved_options.stair_spacing_m, 0.10), 0.15)

        stage_started = perf_counter()
        raw_model = build_semantic_skeleton_from_ifc(temp_path, resolved_options)
        stage_seconds["semantic_import"] = perf_counter() - stage_started
        import_mode = str(raw_model.metadata.get("semantic_import_mode", "semantics-only"))
        _log_build_stage(
            "semantic_import",
            stage_seconds["semantic_import"],
            mode=import_mode,
            cells=len(raw_model.cells),
            nodes=len(raw_model.nodes),
            spaces=len(raw_model.spaces),
            portals=len(raw_model.portals),
        )

        stage_started = perf_counter()
        surface_stats = reconstruct_walkable_surface(
            temp_path,
            raw_model,
            cell_size_m=surface_cell_size_m,
            agent_height_m=resolved_options.agent_height_m,
            # A zero-radius centreline navmesh is exactly the behaviour that let
            # routes touch furniture/railings. Use a human-body default unless
            # the caller explicitly requests a larger clearance.
            agent_radius_m=max(0.22, resolved_options.agent_clearance_m),
            max_slope_deg=resolved_options.max_slope_deg,
            max_climb_m=0.24,
        )
        stage_seconds["physical_reconstruction"] = perf_counter() - stage_started
        _log_build_stage(
            "physical_reconstruction",
            stage_seconds["physical_reconstruction"],
            cells=len(raw_model.cells),
            replaced=surface_stats.replaced_legacy_surface,
            supports=surface_stats.support_elements,
        )

        # The semantic-only path intentionally has no compatibility floor graph.
        # If physical reconstruction cannot replace it, rebuild the proven legacy
        # representation rather than returning an empty or partially useful model.
        if (
            not surface_stats.replaced_legacy_surface
            and import_mode == "semantics-only"
        ):
            stage_started = perf_counter()
            raw_model = build_from_ifc(temp_path, resolved_options)
            stage_seconds["legacy_fallback_import"] = perf_counter() - stage_started
            raw_model.metadata["semantic_import_mode"] = "full-legacy-physical-fallback"
            raw_model.metadata["surface_reconstruction"] = surface_stats.to_dict()
            raw_model.metadata["surface_source"] = "legacy-qualified-fallback"
            _log_build_stage(
                "legacy_fallback_import",
                stage_seconds["legacy_fallback_import"],
                cells=len(raw_model.cells),
                nodes=len(raw_model.nodes),
                spaces=len(raw_model.spaces),
            )

        stage_started = perf_counter()
        if surface_stats.replaced_legacy_surface:
            # Grid/BRep intersections can leave tiny detached triangles inside a
            # correctly labelled room. Before semantic binding, authored portal
            # proximity is conservative evidence that a tiny patch could be a real
            # threshold, so keep it for the first finalisation pass.
            fragment_stats = prune_tiny_space_fragments(
                raw_model.cells,
                raw_model,
                sampling_cell_size_m=surface_cell_size_m,
            )
            raw_model.metadata["surface_fragment_pruning"] = {
                "removed_cells": fragment_stats.removed_cells,
                "removed_components": fragment_stats.removed_components,
                "protected_vertical_components": fragment_stats.protected_vertical_components,
                "protected_bound_portal_components": fragment_stats.protected_bound_portal_components,
                "protected_portal_proximity_components": fragment_stats.protected_portal_proximity_components,
            }
            raw_model.metadata["cell_count"] = len(raw_model.cells)
        else:
            raw_model.metadata["surface_reconstruction"] = surface_stats.to_dict()
            raw_model.metadata.setdefault("surface_source", "legacy-qualified-fallback")
        stage_seconds["prebind_fragment_pruning"] = perf_counter() - stage_started
        _log_build_stage(
            "prebind_fragment_pruning",
            stage_seconds["prebind_fragment_pruning"],
            cells=len(raw_model.cells),
        )

        # First semantic pass binds real door/open-boundary crossings and derives
        # vertical resources from the reconstructed physical manifold.
        stage_started = perf_counter()
        model = model_from_dict(raw_model.to_dict())
        stage_seconds["semantic_finalization"] = perf_counter() - stage_started
        _log_build_stage(
            "semantic_finalization",
            stage_seconds["semantic_finalization"],
            cells=len(model.cells),
            transitions=len(model.transitions),
        )

        stage_started = perf_counter()
        if surface_stats.replaced_legacy_surface:
            # Now that portal_ids are authoritative, portal *proximity* alone is no
            # longer a reason to retain a tiny island. This second conservative pass
            # removes an unused door-near sampling sliver while explicitly protecting
            # any component that participates in a bound semantic crossing or actual
            # stair/ramp/escalator connection.
            post_stats = prune_tiny_space_fragments(
                model.cells,
                model,
                sampling_cell_size_m=surface_cell_size_m,
                protect_portal_proximity=False,
                protect_bound_portals=True,
            )
            model.metadata["surface_postbind_fragment_pruning"] = {
                "removed_cells": post_stats.removed_cells,
                "removed_components": post_stats.removed_components,
                "protected_vertical_components": post_stats.protected_vertical_components,
                "protected_bound_portal_components": post_stats.protected_bound_portal_components,
                "protected_portal_proximity_components": post_stats.protected_portal_proximity_components,
            }
            model.metadata["cell_count"] = len(model.cells)
            if post_stats.removed_cells:
                # Re-run the shared idempotent portable finalisation so door/open
                # boundary and surface-vertical semantics are rebuilt against the
                # cleaned authoritative cell set.
                model = model_from_dict(model.to_dict())
        stage_seconds["postbind_cleanup"] = perf_counter() - stage_started
        _log_build_stage(
            "postbind_cleanup",
            stage_seconds["postbind_cleanup"],
            cells=len(model.cells),
        )

        model.metadata["source_ifc"] = safe_name
        model.metadata["source"] = "local-app-upload"

        stage_started = perf_counter()
        report = validate_model(model)
        stage_seconds["validation"] = perf_counter() - stage_started
        _log_build_stage(
            "validation",
            stage_seconds["validation"],
            valid=report.valid,
        )

        stage_seconds["total"] = perf_counter() - build_started
        model.metadata["build_stage_seconds"] = {
            key: round(value, 6)
            for key, value in stage_seconds.items()
        }
        _log_build_stage("total", stage_seconds["total"], cells=len(model.cells))
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
