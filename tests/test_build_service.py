from __future__ import annotations

from pathlib import Path

import pytest

from ifcpath.app_runner import _resolve_viewer_root
from ifcpath.build_service import IfcBuildRequestError, build_inav_payload
from ifcpath.study_api import _resolve_viewer_dir


def test_build_service_rejects_empty_upload() -> None:
    with pytest.raises(IfcBuildRequestError, match="empty"):
        build_inav_payload(b"")


def test_build_service_rejects_non_ifc_upload() -> None:
    with pytest.raises(IfcBuildRequestError, match="ISO-10303-21"):
        build_inav_payload(b"not an IFC")


def test_build_service_enforces_configurable_size_limit() -> None:
    payload = b"ISO-10303-21;" + b"x" * 100
    with pytest.raises(IfcBuildRequestError, match="too large"):
        build_inav_payload(payload, max_bytes=32)


def test_local_app_resolves_built_viewer(tmp_path: Path) -> None:
    root = tmp_path / "viewer"
    dist = root / "dist"
    dist.mkdir(parents=True)
    (root / "package.json").write_text("{}", encoding="utf-8")
    (dist / "index.html").write_text("<html></html>", encoding="utf-8")

    assert _resolve_viewer_root(str(root)) == root.resolve()
    assert _resolve_viewer_dir(dist) == dist.resolve()
