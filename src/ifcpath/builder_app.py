from __future__ import annotations

import os
import sys


def main() -> None:
    """Launch the optional IFCPath Builder desktop application."""
    try:
        preference = os.environ.get("IFCPATH_VIEWPORT", "qt").strip().lower()
        if preference in {"gpu", "webgpu", "wgpu"}:
            from .ui.gpu_profile_window import run_gpu_profile_app

            runner = run_gpu_profile_app
        else:
            from .ui.profile_window import run_profile_app

            runner = run_profile_app
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith("PySide6"):
            print(
                "IFCPath Builder requires the desktop extras.\n"
                "Install them with: pip install -e '.[desktop]'",
                file=sys.stderr,
            )
            raise SystemExit(2) from exc
        raise

    raise SystemExit(runner())


if __name__ == "__main__":
    main()
