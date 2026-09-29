import * as THREE from "three";
import { AgentVisual, InstancedAgentRenderer } from "./agent-instancing";
import { toThree, Vec3 } from "./inav-scene";

export type PlaybackAgent = {
  id: string;
  position_m: Vec3;
  forward_xy: [number, number];
  status: string;
  level_id?: string | null;
  space_id?: string | null;
};
export type PlaybackFrame = {
  time_s: number;
  agents: PlaybackAgent[];
  stats?: Record<string, unknown>;
};
export type PlaybackFile = {
  schema: string;
  backend?: string;
  frames: PlaybackFrame[];
  summary?: Record<string, unknown>;
};

const STATUS_COLOR: Record<string, number> = {
  moving: 0x2c7be5,
  transfer: 0x6f42c1,
  waiting: 0xffa500,
  elevator_waiting: 0xffa500,
  elevator: 0x8a63d2,
  evacuated: 0x2eb85c,
  trapped: 0xd9534f,
};

/** High-count interpolation/replay of solver output using shared crowd LOD. */
export class SimulationPlaybackLayer {
  private data?: PlaybackFile;
  private timeS = 0;
  private playing = false;
  private speed = 1;
  private readonly capacity = 20_000;
  private readonly instances: InstancedAgentRenderer;
  private visuals: AgentVisual[] = [];
  private cachedNextFrame?: PlaybackFrame;
  private cachedNextById = new Map<string, PlaybackAgent>();

  constructor(scene: THREE.Scene) {
    this.instances = new InstancedAgentRenderer(scene, this.capacity, 256, 2_048);
  }

  load(data: PlaybackFile) {
    if (!data.schema?.startsWith("ifcpath.simulation/") || !Array.isArray(data.frames)) {
      throw new Error("Not an IFCPath simulation playback file");
    }
    this.data = data;
    this.timeS = 0;
    this.playing = false;
    this.cachedNextFrame = undefined;
    this.cachedNextById.clear();
    this.render();
  }

  clear() {
    this.data = undefined;
    this.timeS = 0;
    this.playing = false;
    this.visuals.length = 0;
    this.cachedNextFrame = undefined;
    this.cachedNextById.clear();
    this.instances.clear();
  }

  setPlaying(value: boolean) {
    this.playing = value;
  }

  get isPlaying() {
    return this.playing;
  }

  setSpeed(value: number) {
    this.speed = Math.max(0.1, Math.min(8, value || 1));
  }

  get durationS() {
    const frames = this.data?.frames ?? [];
    return frames.length ? frames[frames.length - 1].time_s : 0;
  }

  get currentTimeS() {
    return this.timeS;
  }

  get backend() {
    return this.data?.backend ?? "unknown";
  }

  get visibleAgentCount() {
    return this.instances.stats.total;
  }

  get lodStats() {
    return this.instances.stats;
  }

  seek(timeS: number) {
    this.timeS = Math.max(0, Math.min(this.durationS, timeS));
    this.render();
  }

  update(dt: number) {
    if (!this.data || !this.playing || this.durationS <= 0) return;
    this.timeS = Math.min(this.durationS, this.timeS + Math.max(0, dt) * this.speed);
    if (this.timeS >= this.durationS) this.playing = false;
    this.render();
  }

  private render() {
    const frames = this.data?.frames ?? [];
    if (!frames.length) {
      this.instances.clear();
      return;
    }
    const [a, b, t] = surroundingFrames(frames, this.timeS);
    if (b !== this.cachedNextFrame) {
      this.cachedNextFrame = b;
      this.cachedNextById = new Map(b.agents.map((agent) => [agent.id, agent]));
    }

    const count = Math.min(a.agents.length, this.capacity);
    if (this.visuals.length > count) this.visuals.length = count;

    for (let index = 0; index < count; index++) {
      const first = a.agents[index];
      const second = this.cachedNextById.get(first.id) ?? first;
      const position: Vec3 = [
        lerp(first.position_m[0], second.position_m[0], t),
        lerp(first.position_m[1], second.position_m[1], t),
        lerp(first.position_m[2], second.position_m[2], t),
      ];
      const forwardX = lerp(first.forward_xy[0], second.forward_xy[0], t);
      const forwardY = lerp(first.forward_xy[1], second.forward_xy[1], t);
      let visual = this.visuals[index];
      if (!visual) {
        visual = { position: new THREE.Vector3(), yaw: 0 };
        this.visuals[index] = visual;
      }
      visual.position.copy(toThree(position));
      visual.yaw = Math.atan2(forwardX, -forwardY);
      visual.color = STATUS_COLOR[first.status] ?? 0x607080;
      visual.scale = first.status === "evacuated" ? 0.75 : 1;
    }
    this.instances.update(this.visuals);
  }
}

function surroundingFrames(frames: PlaybackFrame[], timeS: number): [PlaybackFrame, PlaybackFrame, number] {
  if (frames.length === 1 || timeS <= frames[0].time_s) return [frames[0], frames[0], 0];
  const last = frames[frames.length - 1];
  if (timeS >= last.time_s) return [last, last, 0];

  let low = 0;
  let high = frames.length - 1;
  while (low + 1 < high) {
    const mid = Math.floor((low + high) / 2);
    if (frames[mid].time_s <= timeS) low = mid;
    else high = mid;
  }
  const a = frames[low], b = frames[high];
  const span = Math.max(1e-9, b.time_s - a.time_s);
  return [a, b, (timeS - a.time_s) / span];
}

function lerp(a: number, b: number, t: number) {
  return a + (b - a) * t;
}
