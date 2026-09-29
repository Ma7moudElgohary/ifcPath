from __future__ import annotations

import argparse
import os
from pathlib import Path
from threading import Timer
from typing import Any
import webbrowser

from .build_service import IfcBuildRequestError, build_inav_payload
from .microscopic_motion import MicroscopicBackendUnavailable
from .study_service import StudyRequestError, run_study


def create_app(viewer_dir: str | Path | None = None):
    try:
        from fastapi import FastAPI, HTTPException, Request
        from fastapi.middleware.cors import CORSMiddleware
        from fastapi.staticfiles import StaticFiles
    except ImportError as exc:  # pragma: no cover - optional runtime dependency
        raise RuntimeError(
            "Live study API requires the 'study-server' extra: "
            "pip install -e '.[study-server]'"
        ) from exc

    static_root = _resolve_viewer_dir(viewer_dir)
    app = FastAPI(title="IfcPath Local Navigation & Study App", version="0.2")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://localhost:5173",
            "http://127.0.0.1:5173",
            "http://localhost:4173",
            "http://127.0.0.1:4173",
            "http://localhost:8765",
            "http://127.0.0.1:8765",
        ],
        allow_credentials=False,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )

    @app.get("/health")
    def health() -> dict[str, Any]:
        jupedsim = False
        version = None
        try:
            import jupedsim as jps

            jupedsim = True
            version = getattr(jps, "__version__", None)
            if version is None:
                build_info = getattr(jps, "build_info", None)
                version = getattr(build_info, "library_version", None) if build_info else None
        except ImportError:
            pass
        return {
            "service": "ifcpath-local-app",
            "status": "ok",
            "jupedsim_available": jupedsim,
            "jupedsim_version": str(version) if version is not None else None,
            "viewer_available": static_root is not None,
        }

    @app.post("/inav/build")
    async def build_inav(request: Request) -> dict[str, Any]:
        try:
            filename = request.headers.get("x-ifc-filename", "upload.ifc")
            return build_inav_payload(await request.body(), filename)
        except IfcBuildRequestError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @app.post("/study/run")
    def run(payload: dict[str, Any]) -> dict[str, Any]:
        try:
            return run_study(payload)
        except StudyRequestError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        except (MicroscopicBackendUnavailable, ImportError) as exc:
            raise HTTPException(
                status_code=503,
                detail=(
                    "Requested microscopic backend is unavailable. Install IFCPath with "
                    "the 'study' extra to enable JuPedSim."
                ),
            ) from exc

    # Mount last so /health, /inav/build and /study/run always win over static
    # paths. html=True provides index.html for the local single-page viewer.
    if static_root is not None:
        app.mount("/", StaticFiles(directory=str(static_root), html=True), name="viewer")

    return app


def _resolve_viewer_dir(value: str | Path | None) -> Path | None:
    candidates: list[Path] = []
    if value:
        candidates.append(Path(value))
    env_value = os.environ.get("IFCPATH_VIEWER_DIR")
    if env_value:
        candidates.append(Path(env_value))
    candidates.extend([
        # Release wheels contain the prebuilt That Open app here. Keeping this
        # first means installed users do not need the source tree or Node.js.
        Path(__file__).resolve().parent / "web_dist",
        Path.cwd() / "viewer" / "dist",
        Path(__file__).resolve().parents[2] / "viewer" / "dist",
    ])
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if (resolved / "index.html").is_file():
            return resolved
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local IfcPath navigation and evacuation-study app")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--reload", action="store_true")
    parser.add_argument("--viewer-dir", help="Built viewer directory; packaged assets are discovered automatically")
    parser.add_argument("--open", action="store_true", dest="open_browser", help="Open the local viewer in the default browser")
    args = parser.parse_args()
    try:
        import uvicorn
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "Live study API requires the 'study-server' extra: pip install -e '.[study-server]'"
        ) from exc

    viewer_dir = _resolve_viewer_dir(args.viewer_dir)
    if args.viewer_dir and viewer_dir is None:
        raise SystemExit(f"Viewer directory does not contain index.html: {args.viewer_dir}")
    if viewer_dir is not None:
        os.environ["IFCPATH_VIEWER_DIR"] = str(viewer_dir)
    if args.open_browser and viewer_dir is not None:
        Timer(0.8, lambda: webbrowser.open(f"http://{args.host}:{args.port}/")).start()

    if args.reload:
        uvicorn.run(
            "ifcpath.study_api:create_app",
            host=args.host,
            port=args.port,
            factory=True,
            reload=True,
        )
    else:
        uvicorn.run(create_app(viewer_dir), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
