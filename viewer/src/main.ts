import * as THREE from "three";
import * as OBC from "@thatopen/components";
import "./style.css";
import { InavScene, SurfacePick, toThree, Vec3 } from "./inav-scene";
import { CrowdLayer } from "./crowd";
import { PlaybackFile, SimulationPlaybackLayer } from "./playback";

const host = document.querySelector<HTMLDivElement>("#app")!;
host.innerHTML = `
  <div id="viewport"></div>
  <div class="hud">
    <div class="brand">IfcPath</div>
    <label>IFC <input id="ifc" type="file" accept=".ifc"/></label>
    <label>INAV <input id="inav" type="file" accept=".inav,.json"/></label>
    <label>Human GLB <input id="human" type="file" accept=".glb,.gltf"/></label>
    <label>Agents <input id="agents" type="number" min="1" max="2000" value="100"/></label>
    <label>Speed <input id="speed" type="number" min="0.1" max="5" step="0.1" value="1"/></label>
    <label><input id="nav-visible" type="checkbox" checked/> Nav surface</label>
    <button id="play">Pause route demo</button>
    <button id="fit">Fit navigation</button>
    <button id="clear">Clear route</button>
    <span id="status">Load IFC + INAV. Shift-click navigation surface for start and end.</span>
  </div>
  <div class="door-panel">
    <label>Blocked door IDs <input id="blocked" placeholder="door:GUID, door:GUID"/></label>
    <button id="apply-blocked">Apply + reroute</button>
    <span id="door-count"></span>
  </div>
  <div class="simulation-panel">
    <label>Solver playback <input id="playback-file" type="file" accept=".json"/></label>
    <button id="sim-play">Play simulation</button>
    <input id="sim-time" type="range" min="0" max="0" value="0" step="0.01"/>
    <label>x <input id="sim-speed" type="number" min="0.1" max="8" step="0.1" value="1"/></label>
    <span id="sim-status">No playback loaded</span>
  </div>
  <div class="legend"><span class="open"></span>open <span class="stair"></span>stair <span class="ramp"></span>ramp</div>
`;
const viewport = document.querySelector<HTMLDivElement>("#viewport")!;
const status = document.querySelector<HTMLSpanElement>("#status")!;
const agentInput = document.querySelector<HTMLInputElement>("#agents")!;
const speedInput = document.querySelector<HTMLInputElement>("#speed")!;
const blockedInput = document.querySelector<HTMLInputElement>("#blocked")!;
const doorCount = document.querySelector<HTMLSpanElement>("#door-count")!;
const playButton = document.querySelector<HTMLButtonElement>("#play")!;
const simPlayButton = document.querySelector<HTMLButtonElement>("#sim-play")!;
const simTime = document.querySelector<HTMLInputElement>("#sim-time")!;
const simSpeed = document.querySelector<HTMLInputElement>("#sim-speed")!;
const simStatus = document.querySelector<HTMLSpanElement>("#sim-status")!;

const components = new OBC.Components();
const worlds = components.get(OBC.Worlds);
const world = worlds.create<OBC.SimpleScene, OBC.OrthoPerspectiveCamera, OBC.SimpleRenderer>();
world.scene = new OBC.SimpleScene(components);
world.scene.setup();
world.renderer = new OBC.SimpleRenderer(components, viewport);
world.renderer.three.setPixelRatio(Math.min(window.devicePixelRatio, 2));
world.camera = new OBC.OrthoPerspectiveCamera(components);
await world.camera.controls.setLookAt(12, 12, 12, 0, 0, 0);
components.init();
components.get(OBC.Grids).create(world);

const fragments = components.get(OBC.FragmentsManager);
fragments.init(await OBC.FragmentsManager.getWorker());
world.camera.controls.addEventListener("update", () => fragments.core.update());
world.onCameraChanged.add((camera) => {
  for (const [, model] of fragments.list) model.useCamera(camera.three);
  fragments.core.update(true);
});
fragments.core.models.materials.list.onItemSet.add(({ value: material }) => {
  if (!("isLodMaterial" in material && material.isLodMaterial)) {
    material.polygonOffset = true;
    material.polygonOffsetUnits = 1;
    material.polygonOffsetFactor = 1;
  }
});
fragments.list.onItemSet.add(({ value: model }) => {
  model.useCamera(world.camera.three);
  world.scene.three.add(model.object);
  fragments.core.update(true);
});

const ifcLoader = components.get(OBC.IfcLoader);
await ifcLoader.setup({
  autoSetWasm: true,
  webIfc: { COORDINATE_TO_ORIGIN: false },
});

const nav = new InavScene(world.scene.three);
const crowd = new CrowdLayer(world.scene.three);
const playback = new SimulationPlaybackLayer(world.scene.three);
let start: SurfacePick | null = null;
let goal: SurfacePick | null = null;
let currentRoute: Vec3[] = [];
let paused = false;

function rebuildCrowd() {
  if (currentRoute.length < 2) return;
  playback.clear();
  simStatus.textContent = "Route demo mode";
  const count = Math.max(1, Math.min(2000, Number(agentInput.value) || 100));
  crowd.spawnRouteAgents(currentRoute.map(toThree), count);
}

function frameNavigation() {
  const box = nav.bounds();
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const radius = Math.max(size.x, size.y, size.z, 4) * 1.25;
  void world.camera.controls.setLookAt(
    center.x + radius,
    center.y + radius * 0.75,
    center.z + radius,
    center.x,
    center.y,
    center.z,
    true,
  );
}

function clearRoute() {
  start = null;
  goal = null;
  currentRoute = [];
  nav.clearRoute();
  crowd.clear();
  status.textContent = "Shift-click navigation surface to choose a start point.";
}

function blockedIds() {
  return blockedInput.value.split(/[\s,;]+/).map((x) => x.trim()).filter(Boolean);
}

function reroute() {
  nav.setBlockedPortalIds(blockedIds());
  if (!start || !goal) {
    status.textContent = `${blockedIds().length} door crossing(s) blocked.`;
    return;
  }
  currentRoute = nav.route(start, goal);
  nav.drawRoute(currentRoute, start.point, goal.point);
  if (!currentRoute.length) {
    crowd.clear();
    status.textContent = `No route with ${blockedIds().length} blocked door(s).`;
    return;
  }
  rebuildCrowd();
  status.textContent = `Funnel route ${nav.routeLength(currentRoute).toFixed(1)} m · ${currentRoute.length} corners · ${blockedIds().length} blocked door(s).`;
}

document.querySelector<HTMLInputElement>("#ifc")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  status.textContent = "Importing IFC with That Open…";
  const bytes = new Uint8Array(await file.arrayBuffer());
  await ifcLoader.load(bytes, false, file.name.replace(/\.ifc$/i, ""));
  status.textContent = "IFC loaded in original coordinates. Load matching INAV to route.";
};

document.querySelector<HTMLInputElement>("#inav")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  const model = JSON.parse(await file.text());
  nav.load(model);
  playback.clear();
  clearRoute();
  const doors = nav.portalIds();
  doorCount.textContent = `${doors.length} door portal(s)`;
  doorCount.title = doors.join("\n");
  frameNavigation();
  status.textContent = `${model.cells?.length ?? 0} navigation cells loaded. Shift-click start and end.`;
};

document.querySelector<HTMLInputElement>("#human")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  status.textContent = "Loading animated human GLB…";
  await crowd.loadHuman(file);
  rebuildCrowd();
  status.textContent = "Human asset ready for route-demo agents.";
};

document.querySelector<HTMLInputElement>("#playback-file")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  try {
    const data = JSON.parse(await file.text()) as PlaybackFile;
    crowd.clear();
    playback.load(data);
    simTime.min = "0";
    simTime.max = String(playback.durationS);
    simTime.value = "0";
    simPlayButton.textContent = "Play simulation";
    simStatus.textContent = `${playback.backend} · ${playback.durationS.toFixed(1)} s`;
    status.textContent = "Loaded microscopic solver playback.";
  } catch (error) {
    simStatus.textContent = error instanceof Error ? error.message : "Invalid playback file";
  }
};

document.querySelector<HTMLButtonElement>("#clear")!.onclick = clearRoute;
document.querySelector<HTMLButtonElement>("#fit")!.onclick = frameNavigation;
document.querySelector<HTMLButtonElement>("#apply-blocked")!.onclick = reroute;
document.querySelector<HTMLInputElement>("#nav-visible")!.onchange = (event) => {
  nav.setVisible((event.target as HTMLInputElement).checked);
};
agentInput.onchange = rebuildCrowd;
speedInput.onchange = () => crowd.setSpeedMultiplier(Number(speedInput.value));
playButton.onclick = () => {
  paused = !paused;
  crowd.setPaused(paused);
  playButton.textContent = paused ? "Resume route demo" : "Pause route demo";
};
simTime.oninput = () => {
  playback.seek(Number(simTime.value));
  simStatus.textContent = `${playback.backend} · ${playback.currentTimeS.toFixed(1)} / ${playback.durationS.toFixed(1)} s`;
};
simSpeed.onchange = () => playback.setSpeed(Number(simSpeed.value));
simPlayButton.onclick = () => {
  const next = !playback.isPlaying;
  playback.setPlaying(next);
  simPlayButton.textContent = next ? "Pause simulation" : "Play simulation";
};

const canvas = world.renderer.three.domElement;
let pointerDown: { x: number; y: number } | null = null;
canvas.addEventListener("pointerdown", (event) => {
  if (event.shiftKey) pointerDown = { x: event.clientX, y: event.clientY };
});
canvas.addEventListener("pointerup", (event) => {
  if (!event.shiftKey || !pointerDown) return;
  const movement = Math.hypot(event.clientX - pointerDown.x, event.clientY - pointerDown.y);
  pointerDown = null;
  if (movement > 4) return;
  const pick = nav.pick(event.clientX, event.clientY, canvas, world.camera.three);
  if (!pick) {
    status.textContent = "No navigation surface under cursor.";
    return;
  }
  if (!start || goal) {
    start = pick;
    goal = null;
    currentRoute = [];
    crowd.clear();
    nav.drawRoute([], start.point);
    status.textContent = `Start: ${start.cellId}. Shift-click destination.`;
    return;
  }
  goal = pick;
  reroute();
});

let last = performance.now();
world.renderer.onBeforeUpdate.add(() => {
  const now = performance.now();
  const dt = Math.min((now - last) / 1000, 0.05);
  crowd.update(dt);
  playback.update(dt);
  if (playback.durationS > 0) {
    simTime.value = String(playback.currentTimeS);
    simPlayButton.textContent = playback.isPlaying ? "Pause simulation" : "Play simulation";
    simStatus.textContent = `${playback.backend} · ${playback.currentTimeS.toFixed(1)} / ${playback.durationS.toFixed(1)} s`;
  }
  last = now;
});

window.addEventListener("beforeunload", () => components.dispose());
