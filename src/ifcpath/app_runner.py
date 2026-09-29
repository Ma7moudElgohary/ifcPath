from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path
from threading import Timer
import webbrowser

from .study_api import create_app


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build and launch the complete local IfcPath web application"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--viewer-root", help="Viewer source directory (defaults to ./viewer)")
    parser.add_argument("--skip-build", action="store_true", help="Require existing viewer assets instead of running Vite")
    parser.add_argument("--no-open", action="store_true", help="Do not open the default browser")
    args = parser.parse_args()

    # Release wheels embed viewer/dist inside the Python package, so end users do
    # not need Node.js. Source checkouts keep the development behavior: locate
    # viewer/, build it with Vite, then serve its dist directory.
    packaged = _packaged_viewer_dir()
    if args.viewer_root:
        viewer_root = _resolve_viewer_root(args.viewer_root)
        dist = viewer_root / "dist"
        if not args.skip_build:
            _build_viewer(viewer_root)
    elif packaged is not None:
        dist = packaged
    else:
        viewer_root = _resolve_viewer_root(None)
        dist = viewer_root / "dist"
        if not args.skip_build:
            _build_viewer(viewer_root)

    if not (dist / "index.html").is_file():
        raise SystemExit(
            f"Built viewer not found at {dist}. In a source checkout run "
            "'npm install && npm run build' in viewer/, or install a release wheel "
            "that contains the packaged web viewer."
        )

    try:
        import uvicorn
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise SystemExit(
            "IfcPath app server requires the 'study-server' or 'study' extra. "
            "Install the release bundle or use: pip install -e '.[study]'"
        ) from exc

    url = f"http://{args.host}:{args.port}/"
    if not args.no_open:
        Timer(0.8, lambda: webbrowser.open(url)).start()
    print(f"IfcPath local app: {url}")
    print("Choose an IFC in the browser; INAV is generated automatically by the local Python kernel.")
    uvicorn.run(create_app(dist), host=args.host, port=args.port)


def _packaged_viewer_dir() -> Path | None:
    candidate = Path(__file__).resolve().parent / "web_dist"
    return candidate if (candidate / "index.html").is_file() else None


def _resolve_viewer_root(value: str | None) -> Path:
    candidates = []
    if value:
        candidates.append(Path(value))
    candidates.extend([
        Path.cwd() / "viewer",
        Path(__file__).resolve().parents[2] / "viewer",
    ])
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if (resolved / "package.json").is_file():
            return resolved
    requested = f" ({value})" if value else ""
    raise SystemExit(f"Could not find IfcPath viewer source directory{requested}")


def _build_viewer(viewer_root: Path) -> None:
    npm = shutil.which("npm")
    if npm is None:
        raise SystemExit(
            "Node.js/npm is required only when building the That Open viewer from source. "
            "Install Node.js 22+, or install an IfcPath release wheel with embedded viewer assets."
        )
    node_modules = viewer_root / "node_modules"
    if not node_modules.is_dir():
        print("Installing viewer dependencies…")
        subprocess.run([npm, "install"], cwd=viewer_root, check=True)
    print("Building That Open viewer…")
    subprocess.run([npm, "run", "build"], cwd=viewer_root, check=True)


if __name__ == "__main__":
    main()
