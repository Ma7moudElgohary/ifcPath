from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    entry = root / "scripts" / "ifcpath_builder_entry.py"
    command = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--windowed",
        "--name",
        "IFCPathBuilder",
        "--paths",
        str(root / "src"),
        "--collect-all",
        "ifcopenshell",
        "--collect-all",
        "shapely",
        str(entry),
    ]
    print(" ".join(command))
    subprocess.run(command, cwd=root, check=True)
    print(f"Desktop bundle created under: {root / 'dist' / 'IFCPathBuilder'}")


if __name__ == "__main__":
    main()
