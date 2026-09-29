import * as THREE from "three";
import { InavModel, toThree, Vec3 } from "./inav-scene";
import { PlaybackFile } from "./playback";

export type PopulationSource = {
  id: string;
  position_m: Vec3;
  count: number;
  speed_mps: number;
  spacing_m: number;
};

export type StudyRunOptions = {
  backend: "kinematic" | "jupedsim";
  blockedPortals: string[];
  blockedSpaces: string[];
  spaceCostMultipliers: Record<string, number>;
  frameIntervalS?: number;
  maxTimeS?: number;
};

export class StudyScenarioLayer {
  private readonly group = new THREE.Group();
  private readonly sources: PopulationSource[] = [];

  constructor(scene: THREE.Scene) {
    this.group.name = "IfcPath study population sources";
    scene.add(this.group);
  }

  addSource(point: Vec3, count: number, speedMps: number, spacingM: number) {
    const source: PopulationSource = {
      id: `source:${this.sources.length + 1}`,
      position_m: [...point] as Vec3,
      count: Math.max(1, Math.min(5000, Math.round(count || 1))),
      speed_mps: Math.max(0.05, speedMps || 1.2),
      spacing_m: Math.max(0.20, spacingM || 0.45),
    };
    this.sources.push(source);
    const marker = new THREE.Mesh(
      new THREE.CylinderGeometry(0.24, 0.34, 0.12, 18),
      new THREE.MeshStandardMaterial({ color: 0x7c3aed, emissive: 0x240066 }),
    );
    marker.position.copy(toThree(point));
    marker.position.y += 0.08;
    marker.userData.populationSourceId = source.id;
    this.group.add(marker);
    return source;
  }

  clear() {
    this.sources.length = 0;
    for (const child of [...this.group.children]) {
      this.group.remove(child);
      if (child instanceof THREE.Mesh) {
        child.geometry.dispose();
        if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
        else child.material.dispose();
      }
    }
  }

  get count() {
    return this.sources.length;
  }

  get agentCount() {
    return this.sources.reduce((sum, source) => sum + source.count, 0);
  }

  values() {
    return this.sources.map((source) => ({ ...source, position_m: [...source.position_m] as Vec3 }));
  }
}

export async function checkStudyServer(apiBase: string) {
  const response = await fetch(`${normalizeBase(apiBase)}/health`);
  if (!response.ok) throw new Error(`Study server health check failed (${response.status})`);
  return (await response.json()) as {
    status: string;
    jupedsim_available?: boolean;
    jupedsim_version?: string | null;
  };
}

export async function runLiveStudy(
  apiBase: string,
  model: InavModel,
  sources: PopulationSource[],
  options: StudyRunOptions,
): Promise<PlaybackFile & { scenario?: Record<string, unknown> }> {
  if (!sources.length) throw new Error("Add at least one population source with Alt-click.");
  const payload = {
    model,
    backend: options.backend,
    groups: sources,
    blocked_portals: options.blockedPortals,
    blocked_spaces: options.blockedSpaces,
    space_cost_multipliers: options.spaceCostMultipliers,
    frame_interval_s: options.frameIntervalS ?? 0.20,
    max_time_s: options.maxTimeS ?? 900,
  };
  const response = await fetch(`${normalizeBase(apiBase)}/study/run`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const raw = await response.json().catch(() => ({}));
  if (!response.ok) {
    const detail = typeof raw?.detail === "string" ? raw.detail : `Study failed (${response.status})`;
    throw new Error(detail);
  }
  return raw as PlaybackFile & { scenario?: Record<string, unknown> };
}

function normalizeBase(value: string) {
  return (value || "http://127.0.0.1:8765").replace(/\/+$/, "");
}
