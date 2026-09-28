# IFCPath Builder Desktop

The desktop Builder is a PySide6 application on top of the same engine used by the `ifcpath` CLI. It does not spawn the CLI as a subprocess; IFC generation, validation and export call the Python core directly.

## Install and run

```bash
python -m pip install -e ".[desktop]"
ifcpath-builder
```

The Builder supports:

- opening IFC models;
- opening existing `.inav`/JSON navigation models;
- selecting CDT or sampled floor generation;
- pedestrian clearance and height;
- configurable fixed IFC obstacle classes;
- floor/stair sampling and graph-connection parameters;
- non-blocking Build & Validate processing;
- level-filtered navigation preview;
- navmesh, graph, portal and exit overlays;
- validation diagnostics and summary metrics;
- `.inav` export;
- drag/drop for IFC and INAV files;
- persisted generation settings.

## Preview

The first Builder viewport is deliberately a lightweight engine-independent top-down navigation preview. It renders the actual generated INAV representation:

- CDT cells grouped by semantic space;
- metric graph edges;
- semantic portals/doors;
- classified exits;
- individual storeys or all storeys.

This keeps the desktop application small and responsive. A full 3D BIM/IFC viewport can replace the preview widget later without changing generation, validation, export or the UI shell.

## Native desktop bundle

Install build extras:

```bash
python -m pip install -e ".[desktop-build]"
```

Then run:

```bash
python scripts/build_desktop.py
```

PyInstaller creates `IFCPathBuilder` under `dist/`. Build on the target operating system (for example, build the Windows executable on Windows).

The packaging helper collects IfcOpenShell and Shapely runtime files in addition to the Qt runtime.

## CI

The normal test job compiles all Python sources. A second `desktop-smoke` job installs PySide6, uses Qt's `offscreen` platform and instantiates the main window with a synthetic INAV model. This catches missing Qt APIs, packaging/import mistakes and preview/model-binding regressions without requiring a display server.
