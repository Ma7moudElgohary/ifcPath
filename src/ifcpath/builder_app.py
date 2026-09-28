from __future__ import annotations

import sys


def main() -> None:
    """Launch the optional IFCPath Builder desktop application."""
    try:
        from .ui.main_window_3d import run_app_3d
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith("PySide6"):
            print(
                "IFCPath Builder requires the desktop extras.\n"
                "Install them with: pip install -e '.[desktop]'",
                file=sys.stderr,
            )
            raise SystemExit(2) from exc
        raise

    raise SystemExit(run_app_3d())


if __name__ == "__main__":
    main()
