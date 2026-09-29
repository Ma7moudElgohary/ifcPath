import * as THREE from "three";
import { AgentVisual, InstancedAgentRenderer } from "./agent-instancing";
import { FramePerformanceMonitor, PerformanceSnapshot } from "./performance";
import "./style.css";

const root = document.querySelector<HTMLDivElement>("#benchmark")!;
root.innerHTML = `
  <div class="benchmark-shell">
    <h1>IfcPath Crowd Rendering Benchmark</h1>
    <p>Measures the production instanced pedestrian renderer, independent of routing/solver time.</p>
    <button id="run-benchmark">Run 100 → 10,000 agents</button>
    <button id="download-results" disabled>Download JSON</button>
    <span id="benchmark-status">Ready</span>
    <table>
      <thead><tr><th>Agents</th><th>Median ms</th><th>P95 ms</th><th>Average FPS</th><th>P95 FPS</th></tr></thead>
      <tbody id="results"></tbody>
    </table>
    <div id="benchmark-view"></div>
  </div>
`;

const viewport = document.querySelector<HTMLDivElement>("#benchmark-view")!;
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
renderer.setSize(Math.max(640, viewport.clientWidth || 960), 480, false);
viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf2f4f7);
scene.add(new THREE.HemisphereLight(0xffffff, 0x555555, 2.2));
const camera = new THREE.PerspectiveCamera(50, renderer.domElement.width / renderer.domElement.height, 0.1, 500);
camera.position.set(24, 28, 32);
camera.lookAt(0, 0, 0);
const crowd = new InstancedAgentRenderer(scene, 20_000, 256, 2_048);
const grid = new THREE.GridHelper(60, 60);
scene.add(grid);

const runButton = document.querySelector<HTMLButtonElement>("#run-benchmark")!;
const downloadButton = document.querySelector<HTMLButtonElement>("#download-results")!;
const status = document.querySelector<HTMLSpanElement>("#benchmark-status")!;
const table = document.querySelector<HTMLTableSectionElement>("#results")!;
let latestResults: Array<{ agents: number } & PerformanceSnapshot> = [];

runButton.onclick = async () => {
  runButton.disabled = true;
  downloadButton.disabled = true;
  table.innerHTML = "";
  latestResults = [];
  try {
    for (const count of [100, 500, 1_000, 5_000, 10_000]) {
      status.textContent = `Running ${count.toLocaleString()} agents…`;
      const snapshot = await runStage(count);
      latestResults.push({ agents: count, ...snapshot });
      table.insertAdjacentHTML(
        "beforeend",
        `<tr><td>${count.toLocaleString()}</td><td>${snapshot.medianFrameMs.toFixed(2)}</td><td>${snapshot.p95FrameMs.toFixed(2)}</td><td>${snapshot.averageFps.toFixed(1)}</td><td>${snapshot.p95Fps.toFixed(1)}</td></tr>`,
      );
    }
    status.textContent = "Complete";
    downloadButton.disabled = false;
  } finally {
    runButton.disabled = false;
  }
};

downloadButton.onclick = () => {
  const payload = {
    schema: "ifcpath.viewer-benchmark/0.1",
    userAgent: navigator.userAgent,
    devicePixelRatio,
    generatedAt: new Date().toISOString(),
    renderer: renderer.info,
    results: latestResults,
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "ifcpath-viewer-benchmark.json";
  link.click();
  URL.revokeObjectURL(url);
};

async function runStage(count: number): Promise<PerformanceSnapshot> {
  const agents = createAgents(count);
  const monitor = new FramePerformanceMonitor(180);
  let last = performance.now();
  let frame = 0;

  return new Promise((resolve) => {
    const tick = (now: number) => {
      const dt = Math.min(0.05, Math.max(0.001, (now - last) / 1000));
      last = now;
      animateAgents(agents, frame, dt);
      crowd.update(agents);
      renderer.render(scene, camera);
      if (frame >= 30) monitor.push(dt * 1000);
      frame++;
      if (frame < 210) requestAnimationFrame(tick);
      else resolve(monitor.snapshot());
    };
    requestAnimationFrame(tick);
  });
}

function createAgents(count: number): AgentVisual[] {
  const columns = Math.ceil(Math.sqrt(count));
  const result: AgentVisual[] = [];
  for (let index = 0; index < count; index++) {
    const x = (index % columns) * 0.55 - columns * 0.275;
    const z = Math.floor(index / columns) * 0.55 - columns * 0.275;
    result.push({
      position: new THREE.Vector3(x, 0.65, z),
      yaw: (index % 16) * Math.PI / 8,
      color: 0x4f6f8f + (index % 3) * 0x090603,
      scale: 0.92 + (index % 11) * 0.012,
    });
  }
  return result;
}

function animateAgents(agents: AgentVisual[], frame: number, dt: number) {
  const phase = frame * dt;
  for (let index = 0; index < agents.length; index++) {
    const agent = agents[index];
    const angle = phase * (0.8 + (index % 5) * 0.05) + index * 0.017;
    agent.position.x += Math.cos(angle) * 0.003;
    agent.position.z += Math.sin(angle) * 0.003;
    agent.yaw = angle;
  }
}
