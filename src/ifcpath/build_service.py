from __future__ import annotations

from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any

from .exporter import model_from_dict
from .ifc_loader import BuildOptions, build_from_ifc
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

    The same IfcOpenShell -> continuous surface -> portable semantic finalization
    path used by the CLI is reused here. The temporary filesystem path is never
    persisted into the returned model metadata.
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

        raw_model = build_from_ifc(temp_path, options or BuildOptions())
        # model_from_dict runs the exact portable semantic finalization used when
        # an INAV is loaded/saved (door recovery, open boundaries, vertical
        # transitions and egress-domain classification).
        model = model_from_dict(raw_model.to_dict())
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
