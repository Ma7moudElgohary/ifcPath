from __future__ import annotations

import argparse
from typing import Any

from .microscopic_motion import MicroscopicBackendUnavailable
from .study_service import StudyRequestError, run_study


def create_app():
    try:
        from fastapi import FastAPI, HTTPException
        from fastapi.middleware.cors import CORSMiddleware
    except ImportError as exc:  # pragma: no cover - optional runtime dependency
        raise RuntimeError(
            "Live study API requires the 'study-server' extra: "
            "pip install -e '.[study-server]'"
        ) from exc

    app = FastAPI(title="IfcPath Live Study API", version="0.1")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://localhost:5173",
            "http://127.0.0.1:5173",
            "http://localhost:4173",
            "http://127.0.0.1:4173",
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
            "service": "ifcpath-live-study",
            "status": "ok",
            "jupedsim_available": jupedsim,
            "jupedsim_version": str(version) if version is not None else None,
        }

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

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local IfcPath live evacuation-study API")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--reload", action="store_true")
    args = parser.parse_args()
    try:
        import uvicorn
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "Live study API requires the 'study-server' extra: pip install -e '.[study-server]'"
        ) from exc
    uvicorn.run(
        "ifcpath.study_api:create_app",
        host=args.host,
        port=args.port,
        factory=True,
        reload=args.reload,
    )


if __name__ == "__main__":
    main()
