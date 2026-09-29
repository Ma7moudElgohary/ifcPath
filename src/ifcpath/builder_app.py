from __future__ import annotations

import sys


def main() -> None:
    """Launch the optional IFCPath Builder desktop application."""
    try:
        from .ui.profile_window import run_profile_app
    except ModuleNotFoundError as exc:
        if exc.name and exc.name.startswith("PySide6"):
            print(
                "IFCPath Builder requires the desktop extras.\n"
                "Install them with: pip install -e '.[desktop]'",
                file=sys.stderr,
            )
            raise SystemExit(2) from exc
        raise

    raise SystemExit(run_profile_app())


if __name__ == "__main__":
    main()
