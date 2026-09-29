import * as THREE from "three";
import * as OBC from "@thatopen/components";
import "./style.css";
import { StudyAnalyticsPanel } from "./analytics";
import { InavModel, InavScene, SurfacePick, toThree, Vec3 } from "./inav-scene";
import { CrowdLayer } from "./crowd";
import { PlaybackFile, SimulationPlaybackLayer } from "./playback";
import { ScenarioEditorLayer, ScenarioEditMode } from "./scenario-editor";
import { checkStudyServer, runLiveStudy, StudyScenarioLayer } from "./study-client";

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
    <span id="status">Load IFC + INAV. Shift-click route points; Alt-click population sources.</span>
  </div>
  <div class="door-panel">
    <label>Blocked door IDs <input id="blocked" placeholder="advanced: door IDs"/></label>
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
  <div class="study-panel">
    <label>API <input id="study-api" value="http://127.0.0.1:8765"/></label>
    <label>Backend
      <select id="study-backend"><option value="jupedsim">JuPedSim</option><option value="kinematic">Kinematic</option></select>
    </label>
    <label>Per source <input id="source-count" type="number" min="1" max="5000" value="25"/> agents</label>
    <label>Walk <input id="source-speed" type="number" min="0.2" max="3" step="0.05" value="1.2"/> m/s</label>
    <label>Spacing <input id="source-spacing" type="number" min="0.2" max="2" step="0.05" value="0.45"/> m</label>
    <label>Blocked spaces <input id="blocked-spaces" placeholder="advanced: space IDs"/></label>
    <label>Hazard costs <input id="space-costs" placeholder="advanced: space=3"/></label>
    <button id="study-health">Check server</button>
    <button id="study-clear">Clear sources</button>
    <button id="study-run">Run study</button>
    <span id="study-status">Alt-click navigation surface to place population sources.</span>
  </div>
  <div class="scenario-panel">
    <b>Visual scenario</b>
    <label>Edit mode
      <select id="scenario-mode">
        <option value="navigate">Navigate</option>
        <option value="population">Population</option>
        <option value="block-space">Block space</option>
        <option value="hazard-space">Hazard space</option>
        <option value="block-door">Block nearest door</option>
      </select>
    </label>
    <label>Hazard x <input id="hazard-multiplier" type="number" min="1" max="100" step="0.5" value="3"/></label>
    <button id="scenario-clear">Clear scenario</button>
    <span id="scenario-status">Choose a mode, then click the navigation surface.</span>
  </div>
  <div id="analytics" class="analytics-panel"></div>
  <div class="legend"><span class="open"></span>open <span class="stair"></span>stair <span class="ramp"></span>ramp <span class="source"></span>population <span class="hazard"></span>hazard <span class="blocked"></span>blocked</div>
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
const studyApi = document.querySelector<HTMLInputElement>("#study-api")!;
const studyBackend = document.querySelector<HTMLSelectElement>("#study-backend")!;
const sourceCount = document.querySelector<HTMLInputElement>("#source-count")!;
const sourceSpeed = document.querySelector<HTMLInputElement>("#source-speed")!;
const sourceSpacing = document.querySelector<HTMLInputElement>("#source-spacing")!;
const blockedSpacesInput = document.querySelector<HTMLInputElement>("#blocked-spaces")!;
const spaceCostsInput = document.querySelector<HTMLInputElement>("#space-costs")!;
const studyStatus = document.querySelector<HTMLSpanElement>("#study-status")!;
const studyRunButton = document.querySelector<HTMLButtonElement>("#study-run")!;
const scenarioMode = document.querySelector<HTMLSelectElement>("#scenario-mode")!;
const hazardMultiplier = document.querySelector<HTMLInputElement>("#hazard-multiplier")!;
const scenarioStatus = document.querySelector<HTMLSpanElement>("#scenario-status")!;

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
await ifcLoader.setup({ autoSetWasm: true, webIfc: { COORDINATE_TO_ORIGIN: false } });

const nav = new InavScene(world.scene.three);
const crowd = new CrowdLayer(world.scene.three);
const playback = new SimulationPlaybackLayer(world.scene.three);
const study = new StudyScenarioLayer(world.scene.three);
const scenario = new ScenarioEditorLayer(world.scene.three);
const analytics = new StudyAnalyticsPanel(document.querySelector<HTMLElement>("#analytics")!);
let loadedModel: InavModel | null = null;
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
    center.x + radius, center.y + radius * 0.75, center.z + radius,
    center.x, center.y, center.z, true,
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

function splitIds(value: string) {
  return value.split(/[\s,;]+/).map((x) => x.trim()).filter(Boolean);
}
function mergeIds(...collections: string[][]) {
  return [...new Set(collections.flat())].sort();
}
function blockedIds() { return splitIds(blockedInput.value); }
function parseSpaceCosts() {
  const result: Record<string, number> = {};
  for (const token of spaceCostsInput.value.split(/[,;]+/)) {
    const [key, raw] = token.split("=").map((x) => x.trim());
    if (!key || !raw) continue;
    const value = Number(raw);
    if (Number.isFinite(value) && value >= 1) result[key] = value;
  }
  return result;
}
function effectiveScenario() {
  const visual = scenario.state();
  return {
    blockedPortals: mergeIds(blockedIds(), visual.blockedPortals),
    blockedSpaces: mergeIds(splitIds(blockedSpacesInput.value), visual.blockedSpaces),
    spaceCostMultipliers: { ...parseSpaceCosts(), ...visual.spaceCostMultipliers },
  };
}
function updateSourceStatus(prefix = "") {
  studyStatus.textContent = `${prefix}${study.count} source(s) · ${study.agentCount} planned agent(s)`;
}
function updateScenarioStatus(message?: string) {
  scenarioStatus.textContent = `${message ? `${message} ` : ""}${scenario.describe()}`;
}

function reroute() {
  const effective = effectiveScenario();
  nav.setBlockedPortalIds(effective.blockedPortals);
  if (!start || !goal) {
    status.textContent = `${effective.blockedPortals.length} door crossing(s) blocked.`;
    return;
  }
  currentRoute = nav.route(start, goal);
  nav.drawRoute(currentRoute, start.point, goal.point);
  if (!currentRoute.length) {
    crowd.clear();
    status.textContent = `No route with ${effective.blockedPortals.length} blocked door(s).`;
    return;
  }
  rebuildCrowd();
  status.textContent = `Funnel route ${nav.routeLength(currentRoute).toFixed(1)} m · ${currentRoute.length} corners · ${effective.blockedPortals.length} blocked door(s).`;
}

function loadPlayback(data: PlaybackFile, message: string) {
  crowd.clear();
  playback.load(data);
  analytics.render(data);
  simTime.min = "0";
  simTime.max = String(playback.durationS);
  simTime.value = "0";
  simPlayButton.textContent = "Play simulation";
  simStatus.textContent = `${playback.backend} · ${playback.durationS.toFixed(1)} s`;
  status.textContent = message;
}

document.querySelector<HTMLInputElement>("#ifc")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  status.textContent = "Importing IFC with That Open…";
  const bytes = new Uint8Array(await file.arrayBuffer());
  await ifcLoader.load(bytes, false, file.name.replace(/\.ifc$/i, ""));
  status.textContent = "IFC loaded in original coordinates. Load matching INAV to route/simulate.";
};

document.querySelector<HTMLInputElement>("#inav")!.onchange = async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (!file) return;
  const model = JSON.parse(await file.text()) as InavModel;
  loadedModel = model;
  nav.load(model);
  scenario.load(model);
  analytics.clear();
  playback.clear();
  study.clear();
  updateSourceStatus();
  updateScenarioStatus();
  clearRoute();
  const doors = nav.portalIds();
  doorCount.textContent = `${doors.length} door portal(s)`;
  doorCount.title = doors.join("\n");
  frameNavigation();
  status.textContent = `${model.cells?.length ?? 0} navigation cells loaded. Shift-click route; Alt-click population; visual edit modes use plain click.`;
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
    loadPlayback(JSON.parse(await file.text()) as PlaybackFile, "Loaded microscopic solver playback.");
  } catch (error) {
    simStatus.textContent = error instanceof Error ? error.message : "Invalid playback file";
  }
};

document.querySelector<HTMLButtonElement>("#clear")!.onclick = clearRoute;
document.querySelector<HTMLButtonElement>("#fit")!.onclick = frameNavigation;
document.querySelector<HTMLButtonElement>("#apply-blocked")!.onclick = reroute;
document.querySelector<HTMLInputElement>("#nav-visible")!.onchange = (event) => nav.setVisible((event.target as HTMLInputElement).checked);
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

document.querySelector<HTMLButtonElement>("#study-clear")!.onclick = () => {
  study.clear();
  updateSourceStatus();
};
document.querySelector<HTMLButtonElement>("#scenario-clear")!.onclick = () => {
  scenario.clear();
  blockedInput.value = "";
  blockedSpacesInput.value = "";
  spaceCostsInput.value = "";
  updateScenarioStatus("Scenario cleared.");
  reroute();
};
document.querySelector<HTMLButtonElement>("#study-health")!.onclick = async () => {
  studyStatus.textContent = "Checking study server…";
  try {
    const health = await checkStudyServer(studyApi.value);
    studyStatus.textContent = health.jupedsim_available
      ? `Server ready · JuPedSim ${health.jupedsim_version ?? "available"}`
      : "Server ready · JuPedSim not installed (kinematic available)";
  } catch (error) {
    studyStatus.textContent = error instanceof Error ? error.message : "Study server unavailable";
  }
};
studyRunButton.onclick = async () => {
  if (!loadedModel) {
    studyStatus.textContent = "Load an INAV model first.";
    return;
  }
  const effective = effectiveScenario();
  studyRunButton.disabled = true;
  studyStatus.textContent = `Running ${study.agentCount} agents · ${scenario.describe()}…`;
  try {
    const result = await runLiveStudy(studyApi.value, loadedModel, study.values(), {
      backend: studyBackend.value as "kinematic" | "jupedsim",
      blockedPortals: effective.blockedPortals,
      blockedSpaces: effective.blockedSpaces,
      spaceCostMultipliers: effective.spaceCostMultipliers,
      frameIntervalS: 0.20,
      maxTimeS: 900,
    });
    loadPlayback(result, "Live evacuation study complete. Solver playback loaded.");
    playback.setPlaying(true);
    simPlayButton.textContent = "Pause simulation";
    const summary = result.summary ?? {};
    const clearance = Number(summary.clearance_time_s);
    studyStatus.textContent = `Complete · ${study.agentCount} agents${Number.isFinite(clearance) ? ` · clearance ${clearance.toFixed(1)} s` : ""}`;
  } catch (error) {
    studyStatus.textContent = error instanceof Error ? error.message : "Study failed";
  } finally {
    studyRunButton.disabled = false;
  }
};

const canvas = world.renderer.three.domElement;
type PointerMode = "route" | "population" | "scenario";
let pointerDown: { x: number; y: number; mode: PointerMode } | null = null;
canvas.addEventListener("pointerdown", (event) => {
  if (event.altKey) pointerDown = { x: event.clientX, y: event.clientY, mode: "population" };
  else if (event.shiftKey) pointerDown = { x: event.clientX, y: event.clientY, mode: "route" };
  else if (scenarioMode.value !== "navigate") {
    pointerDown = {
      x: event.clientX,
      y: event.clientY,
      mode: scenarioMode.value === "population" ? "population" : "scenario",
    };
  }
});
canvas.addEventListener("pointerup", (event) => {
  if (!pointerDown) return;
  const state = pointerDown;
  pointerDown = null;
  const movement = Math.hypot(event.clientX - state.x, event.clientY - state.y);
  if (movement > 4) return;
  const pick = nav.pick(event.clientX, event.clientY, canvas, world.camera.three);
  if (!pick) {
    status.textContent = "No navigation surface under cursor.";
    return;
  }
  if (state.mode === "population") {
    study.addSource(
      pick.point,
      Number(sourceCount.value),
      Number(sourceSpeed.value),
      Number(sourceSpacing.value),
    );
    updateSourceStatus("Population added · ");
    return;
  }
  if (state.mode === "scenario") {
    const result = scenario.applyPick(
      pick,
      scenarioMode.value as ScenarioEditMode,
      Number(hazardMultiplier.value),
    );
    updateScenarioStatus(result.message);
    if (result.changed) reroute();
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
