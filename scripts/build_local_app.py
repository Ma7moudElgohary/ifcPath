from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    entry = root / "scripts" / "ifcpath_app_entry.py"
    web_dist = root / "src" / "ifcpath" / "web_dist"
    if not (web_dist / "index.html").is_file():
        raise SystemExit(
            "Packaged viewer assets are missing. Build viewer/ and copy viewer/dist "
            "to src/ifcpath/web_dist before building the standalone app."
        )

    separator = ";" if os.name == "nt" else ":"
    add_data = f"{web_dist}{separator}ifcpath/web_dist"
    command = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--name",
        "IFCPathApp",
        "--paths",
        str(root / "src"),
        "--collect-submodules",
        "ifcpath",
        "--collect-all",
        "ifcopenshell",
        "--collect-all",
        "shapely",
        "--collect-all",
        "jupedsim",
        "--collect-all",
        "fastapi",
        "--collect-all",
        "uvicorn",
        "--add-data",
        add_data,
        str(entry),
    ]
    print(" ".join(command))
    subprocess.run(command, cwd=root, check=True)
    print(f"Standalone local app created under: {root / 'dist' / 'IFCPathApp'}")


if __name__ == "__main__":
    main()
