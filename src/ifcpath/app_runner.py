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
    parser.add_argument("--skip-build", action="store_true", help="Require an existing viewer/dist instead of running Vite")
    parser.add_argument("--no-open", action="store_true", help="Do not open the default browser")
    args = parser.parse_args()

    viewer_root = _resolve_viewer_root(args.viewer_root)
    dist = viewer_root / "dist"
    if not args.skip_build:
        _build_viewer(viewer_root)
    if not (dist / "index.html").is_file():
        raise SystemExit(
            f"Built viewer not found at {dist}. Run 'npm install && npm run build' in {viewer_root}."
        )

    try:
        import uvicorn
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise SystemExit(
            "IfcPath app server requires the 'study-server' or 'study' extra: "
            "pip install -e '.[study]'"
        ) from exc

    url = f"http://{args.host}:{args.port}/"
    if not args.no_open:
        Timer(0.8, lambda: webbrowser.open(url)).start()
    print(f"IfcPath local app: {url}")
    print("Choose an IFC in the browser; INAV is generated automatically by the local Python kernel.")
    uvicorn.run(create_app(dist), host=args.host, port=args.port)


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
            "Node.js/npm is required to build the That Open viewer. Install Node.js 22+, "
            "or run ifcpath-app --skip-build with an existing viewer/dist."
        )
    node_modules = viewer_root / "node_modules"
    if not node_modules.is_dir():
        print("Installing viewer dependencies…")
        subprocess.run([npm, "install"], cwd=viewer_root, check=True)
    print("Building That Open viewer…")
    subprocess.run([npm, "run", "build"], cwd=viewer_root, check=True)


if __name__ == "__main__":
    main()
