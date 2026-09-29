# IfcPath Web Viewer

The web viewer consumes the portable `.inav` navigation model next to the original IFC. The Python kernel remains the geometry/routing source of truth; the browser is an interactive BIM/navigation/simulation client.

## Architecture

- **That Open Components / FragmentsManager / IfcLoader**: IFC import, worker-based BIM rendering, culling and LOD.
- **IfcPath INAV**: continuous 3D walkable cells, terrain tags, full geometric cell portals, semantic IFC door IDs and blocked-door rerouting.
- **Pathfinder-style routing**: weighted cell A* produces a polygon corridor; a classic funnel/string-pull consumes full left/right portal segments to create the route centreline.
- **JuPedSim / hybrid simulation**: IFCPath owns BIM semantics, route choice, doors, stairs, elevators and queues; JuPedSim owns local collision-avoidance motion inside connected semantic domains.
- **Live study service**: a local FastAPI endpoint accepts the already-loaded INAV model plus browser-authored populations/scenario state and returns `ifcpath.simulation/0.1` playback.
- **Three.js overlays**: navmesh terrain, route line, start/end markers, population sources, route-demo crowd and solver playback.

The viewer loads IFC **without coordinate-to-origin transforms**. INAV geometry is generated in IFC world coordinates, so recentering only the BIM would misalign the layers.

## Interactive routing

1. Load the IFC.
2. Load its matching `.inav` file.
3. Shift-click the nav surface for start and destination.
4. Enter one or more `door:GUID` IDs in **Blocked door IDs** and reroute to test closures.
5. The displayed route uses the same portal-funnel model as the Python surface router.

## Live evacuation study

Install and start the local study service:

```bash
pip install -e '.[study]'
ifcpath-study-server
```

The default endpoint is `http://127.0.0.1:8765`. The server exposes `/health` and `/study/run`; CORS is limited to the local Vite/preview origins used by this viewer.

Then in the viewer:

1. Load IFC + matching INAV.
2. Press **Check server**. It reports whether JuPedSim is installed.
3. Choose **JuPedSim** (microscopic movement) or **Kinematic** (dependency-free deterministic fallback).
4. Set people per source, walking speed and minimum spawn spacing.
5. **Alt-click** the navigation surface to add one or more population sources. Each source is expanded deterministically inside the containing walkable space, respecting the requested minimum spacing.
6. Optionally enter blocked door IDs, blocked space IDs and hazard cost multipliers such as `space-guid=3`.
7. Press **Run study**. The viewer posts the INAV model and scenario to the local service and immediately loads the returned solver playback.
8. Use the existing simulation controls to play/pause, scrub time and change playback speed.

Door/space restrictions and cost multipliers are applied by the same hierarchical route planner used outside the viewer. No browser-only shortcut graph is introduced.

## Offline microscopic simulation playback

The same `ifcpath.simulation/0.1` format can be generated from the CLI:

```bash
pip install -e '.[microscopic]'
python scripts/export_simulation_playback.py model.inav agents.json playback.json --backend jupedsim
```

Example `agents.json`:

```json
{
  "agents": [
    {"id": "person-1", "start_m": [2.0, 3.0, 0.0], "speed_mps": 1.2},
    {"id": "person-2", "start_m": [2.6, 3.1, 0.0], "speed_mps": 1.1}
  ]
}
```

Load `playback.json` with **Solver playback**. Agent colors indicate moving/transfer/waiting/elevator/evacuated/trapped state.

## Human assets

The route-demo crowd renderer is intentionally asset-independent. A good free starter pack is **Kenney Mini Characters** (`https://kenney.nl/assets/mini-characters`): animated CC0 character files with multiple people variants. Keep third-party binary assets outside the kernel and swap them through the Human GLB input or a deployment asset pipeline.

Solver playback uses a highly efficient instanced representation so thousands of recorded agent positions can be inspected without requiring one skeleton per pedestrian. The renderer contract is independent of the solver and human asset pack.

## Viewer development

```bash
cd viewer
npm install
npm run dev
npm run build
```

The implementation follows the current That Open `Worlds` + `FragmentsManager` + `IfcLoader` pattern rather than constructing a parallel BIM renderer.
