# IfcPath Web Viewer

The web viewer consumes the portable `.inav` navigation model next to the original IFC. The Python kernel remains the geometry/routing source of truth; the browser is an interactive BIM/navigation/simulation client.

## Architecture

- **That Open Components / FragmentsManager / IfcLoader**: IFC import, worker-based BIM rendering, culling and LOD.
- **IfcPath INAV**: continuous 3D walkable cells, terrain tags, full geometric cell portals, semantic IFC door IDs and blocked-door rerouting.
- **Pathfinder-style routing**: weighted cell A* produces a polygon corridor; a classic funnel/string-pull consumes the full left/right portal segments to create the route centreline.
- **JuPedSim / hybrid simulation**: IFCPath owns BIM semantics, route choice, doors, stairs, elevators and queues; JuPedSim can own local collision-avoidance motion inside connected semantic domains.
- **Three.js overlays**: navmesh terrain, route line, start/end markers, route-demo crowd and solver playback.

The viewer deliberately loads IFC **without coordinate-to-origin transforms**. INAV geometry is generated in IFC world coordinates, so recentering only the BIM would misalign the two layers.

## Interactive routing

1. Load the IFC.
2. Load its matching `.inav` file.
3. Shift-click the nav surface for start and destination.
4. Enter one or more `door:GUID` IDs in **Blocked door IDs** and reroute to test closures.
5. The displayed route uses the same portal-funnel model as the Python surface router.
6. Change route-demo crowd count/speed, toggle the navigation overlay or fit the camera to navigation geometry.

## Microscopic simulation playback

IFCPath can export live `HybridEvacuationSimulator` state as `ifcpath.simulation/0.1`. When the hybrid backend is `jupedsim`, these frames contain the actual microscopic solver positions and headings, rather than decorative interpolation along one route.

Generate playback with:

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

Load `playback.json` with **Solver playback** in the viewer. The controls support play/pause, time scrubbing and playback speed. Agent colors indicate moving/transfer/waiting/elevator/evacuated/trapped state.

## Human assets

The route-demo crowd renderer is intentionally asset-independent. A good free starter pack is **Kenney Mini Characters** (`https://kenney.nl/assets/mini-characters`): animated CC0 character files with multiple people variants. Keep third-party binary assets outside the kernel and swap them through the Human GLB input or a deployment asset pipeline.

The solver-playback layer currently uses a highly efficient instanced representation so thousands of recorded agent positions can be inspected without requiring one skeleton per pedestrian. The renderer contract is independent of the solver and human asset pack.

## Development

```bash
npm install
npm run dev
npm run build
```

The implementation follows the current That Open `Worlds` + `FragmentsManager` + `IfcLoader` pattern rather than constructing a parallel BIM renderer.
