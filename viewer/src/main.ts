import * as THREE from "three";
import * as OBC from "@thatopen/components";
import "./style.css";
import { InavScene, SurfacePick, toThree, Vec3 } from "./inav-scene";
import { CrowdLayer } from "./crowd";

const host = document.querySelector<HTMLDivElement>("#app")!;
host.innerHTML = `
  <div id="viewport"></div>
  <div class="hud">
    <div class="brand">IfcPath</div>
    <label>IFC <input id="ifc" type="file" accept=".ifc"/></label>
    <label>INAV <input id="inav" type="file" accept=".inav,.json"/></label>
    <label>Human GLB <input id="human" type="file" accept=".glb,.gltf"/></label>
    <label>Agents <input id="agents" type="number" min="1" max="2000" value="100"/></label>
    <button id="clear">Clear route</button>
    <span id="status">Load IFC + INAV. Shift-click navigation surface for start and end.</span>
  </div>
  <div class="legend"><span class="open"></span>open <span class="stair"></span>stair <span class="ramp"></span>ramp</div>
`;
const viewport = document.querySelector<HTMLDivElement>("#viewport")!;
const status = document.querySelector<HTMLSpanElement>("#status")!;
const agentInput = document.querySelector<HTMLInputElement>("#agents")!;

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
await ifcLoader.setup({ autoSetWasm: true });
const nav = new InavScene(world.scene.three);
const crowd = new CrowdLayer(world.scene.three);

let start: SurfacePick | null = null;
let goal: SurfacePick | null = null;
let currentRoute: Vec3[] = [];

function rebuildCrowd() {
  if (currentRoute.length < 2) return;
  const count = Math.max(1, Math.min(2000, Number(agentInput.value) || 100));
  crowd.spawnRouteAgents(currentRoute.map(toThree), count);
}

function clearRoute() {
  start = null;
  goal = null;
  currentRoute = [];
  nav.clearRoute();
  crowd.clear();
  status.textContent = "Shift-click navigation surface to choose a start point.";
}

document.querySelector<HTMLInputElement>("#ifc")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  status.textContent = "Importing IFC…";
  const bytes = new Uint8Array(await file.arrayBuffer());
  await ifcLoader.load(bytes, true, file.name.replace(/\.ifc$/i, ""));
  status.textContent = "IFC loaded. Load matching INAV to route.";
};

document.querySelector<HTMLInputElement>("#inav")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  const model = JSON.parse(await file.text());
  nav.load(model);
  clearRoute();
  status.textContent = `${model.cells?.length ?? 0} navigation cells loaded. Shift-click start and end.`;
};

document.querySelector<HTMLInputElement>("#human")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  status.textContent = "Loading animated human GLB…";
  await crowd.loadHuman(file);
  rebuildCrowd();
  status.textContent = "Human asset ready. First 64 agents use the animated model; larger crowds are instanced.";
};

document.querySelector<HTMLButtonElement>("#clear")!.onclick = clearRoute;
agentInput.onchange = rebuildCrowd;

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
  currentRoute = nav.route(start, goal);
  nav.drawRoute(currentRoute, start.point, goal.point);
  if (!currentRoute.length) {
    status.textContent = `No connected surface route from ${start.cellId} to ${goal.cellId}.`;
    crowd.clear();
    return;
  }
  rebuildCrowd();
  status.textContent = `Route ready: ${currentRoute.length} waypoints. ${agentInput.value} agents.`;
});

let last = performance.now();
world.renderer.onBeforeUpdate.add(() => {
  const now = performance.now();
  crowd.update(Math.min((now - last) / 1000, 0.05));
  last = now;
});

window.addEventListener("beforeunload", () => components.dispose());
