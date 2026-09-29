# IfcPath Web Viewer

The web viewer consumes the portable `.inav` navigation model next to the original IFC. The Python kernel remains the geometry/routing source of truth; the browser is an interactive BIM/navigation/simulation client.

## Architecture

- **That Open Components / FragmentsManager / IfcLoader**: IFC import, worker-based BIM rendering, culling and LOD.
- **IfcPath INAV**: continuous 3D walkable cells, terrain tags, full geometric cell portals, semantic IFC door IDs and blocked-door rerouting.
- **Pathfinder-style routing**: weighted cell A* produces a polygon corridor; a classic funnel/string-pull consumes full left/right portal segments to create the route centreline.
- **JuPedSim / hybrid simulation**: IFCPath owns BIM semantics, route choice, doors, stairs, elevators and queues; JuPedSim owns local collision-avoidance motion inside connected semantic domains.
- **Live study service**: a local FastAPI endpoint accepts the already-loaded INAV model plus browser-authored populations/scenario state and returns `ifcpath.simulation/0.1` playback.
- **Visual scenario layer**: click-authored blocked spaces, hazard spaces and blocked semantic doors are overlays only; they do not modify the navigation geometry.
- **Three.js overlays**: navmesh terrain, route line, start/end markers, population sources, scenario state, route-demo crowd and solver playback.

The viewer loads IFC **without coordinate-to-origin transforms**. INAV geometry is generated in IFC world coordinates, so recentering only the BIM would misalign the layers.

## Interactive routing

1. Load the IFC and its matching `.inav` file.
2. **Shift-click** the nav surface for start and destination.
3. The displayed route uses the same portal-funnel model as the Python surface router.
4. Door closures authored in the visual scenario editor immediately affect route-demo rerouting too.

## Visual scenario authoring

Raw GUID fields remain available as an advanced/debug path, but normal studies can be authored visually:

1. Choose an **Edit mode** in **Visual scenario**.
2. Choose **Population** and click a walkable area, or use the **Alt-click** shortcut.
3. Choose **Block space** and click a room to toggle it red/non-traversable.
4. Choose **Hazard space**, set the multiplier, and click a room to toggle an orange route-cost region.
5. Choose **Block nearest door** and click near a semantic door to toggle its red closure marker.
6. **Clear scenario** removes visual restrictions and the advanced raw scenario fields.

Scenario overlays resolve clicked navigation cells back to semantic spaces and semantic IFC door portals. The overlay never rewrites the qualified navmesh; the resulting IDs/costs are sent to the same Python hierarchical planner used by non-viewer workflows.

## Live evacuation study

Install and start the local study service:

```bash
pip install -e '.[study]'
ifcpath-study-server
```

The default endpoint is `http://127.0.0.1:8765`. The server exposes `/health` and `/study/run`; CORS is limited to the local Vite/preview origins used by this viewer.

Then:

1. Load IFC + matching INAV.
2. Press **Check server** to verify JuPedSim availability.
3. Choose **JuPedSim** or the deterministic **Kinematic** fallback.
4. Set people per source, walking speed and minimum spawn spacing.
5. Place one or more population sources visually.
6. Author blocked/hazard areas and doors visually; raw ID fields can still be combined with the visual state.
7. Press **Run study**. The browser posts the INAV model and scenario to the local service and immediately loads the returned solver playback.
8. Play/pause, scrub time and change playback speed.

## Results analytics

Every loaded or newly run `ifcpath.simulation/0.1` result feeds the analytics panel. It shows:

- total, evacuated and trapped agents
- clearance time and average evacuation time
- maximum queue
- simulation duration
- exit usage
- a down-sampled timeline of moving, waiting, evacuated and trapped populations

The analytics are derived from the portable playback/summary contract, not from browser-side re-simulation.

## Offline microscopic simulation playback

The same format can be generated from the CLI:

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

The route-demo crowd renderer is asset-independent. **Kenney Mini Characters** is the current lightweight CC0 starter reference. Keep third-party binary assets outside the geometry kernel and swap them through the Human GLB input or a deployment asset pipeline.

Solver playback uses an efficient instanced representation so thousands of recorded agent positions can be inspected without requiring one skeleton per pedestrian.

## Viewer development

```bash
cd viewer
npm install
npm run dev
npm run build
```

The implementation follows the current That Open `Worlds` + `FragmentsManager` + `IfcLoader` pattern rather than constructing a parallel BIM renderer.
