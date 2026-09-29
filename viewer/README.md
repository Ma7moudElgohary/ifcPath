# IfcPath Web Viewer

The web viewer consumes the portable `.inav` navigation model next to the original IFC. The Python kernel remains the geometry/routing source of truth; the browser is an interactive BIM/navigation/simulation client.

## Architecture

- **That Open Components / FragmentsManager / IfcLoader**: IFC import, worker-based BIM rendering, culling and LOD.
- **IfcPath INAV**: continuous 3D walkable cells, terrain tags, geometric cell portals, semantic IFC door IDs and blocked-door rerouting.
- **Three.js overlays**: navmesh terrain, route line, start/end markers and crowd agents.
- **CrowdLayer**: up to 64 skinned/animated GLB agents plus instanced lightweight agents for large visual crowds.

The viewer deliberately loads IFC **without coordinate-to-origin transforms**. INAV geometry is generated in IFC world coordinates, so recentering only the BIM would misalign the two layers.

## Interaction

1. Load the IFC.
2. Load its matching `.inav` file.
3. Shift-click the nav surface for start and destination.
4. Enter one or more `door:GUID` IDs in **Blocked door IDs** and reroute to test closures.
5. Change crowd count/speed, pause playback, toggle the navigation overlay or fit the camera to navigation geometry.
6. Optionally load an animated human `.glb`/`.gltf`. Without one, the viewer uses highly efficient instanced proxy agents.

## Human assets

The crowd renderer is intentionally asset-independent. A good free starter pack is **Kenney Mini Characters** (`https://kenney.nl/assets/mini-characters`): 25 animated 3D character files under **CC0**, including people/disability variants. Keep third-party binary assets outside the kernel and swap them through the Human GLB input or a deployment asset pipeline.

## Development

```bash
npm install
npm run dev
npm run build
```

The implementation follows the current That Open `Worlds` + `FragmentsManager` + `IfcLoader` pattern rather than constructing a parallel BIM renderer.
