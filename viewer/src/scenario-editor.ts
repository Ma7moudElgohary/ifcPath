import * as THREE from "three";
import { InavModel, NavCell, SurfacePick, toThree, Vec3 } from "./inav-scene";

export type ScenarioEditMode = "population" | "block-space" | "hazard-space" | "block-door";

export type ScenarioState = {
  blockedPortals: string[];
  blockedSpaces: string[];
  spaceCostMultipliers: Record<string, number>;
};

export class ScenarioEditorLayer {
  private readonly group = new THREE.Group();
  private model: InavModel = {};
  private byCell = new Map<string, NavCell>();
  private blockedSpaces = new Set<string>();
  private blockedPortals = new Set<string>();
  private hazardSpaces = new Map<string, number>();

  constructor(scene: THREE.Scene) {
    this.group.name = "IfcPath scenario overlays";
    scene.add(this.group);
  }

  load(model: InavModel) {
    this.model = model;
    this.byCell = new Map((model.cells ?? []).map((cell) => [cell.id, cell]));
    this.clear();
  }

  clear() {
    this.blockedSpaces.clear();
    this.blockedPortals.clear();
    this.hazardSpaces.clear();
    this.rebuildOverlay();
  }

  state(): ScenarioState {
    return {
      blockedPortals: [...this.blockedPortals].sort(),
      blockedSpaces: [...this.blockedSpaces].sort(),
      spaceCostMultipliers: Object.fromEntries([...this.hazardSpaces.entries()].sort()),
    };
  }

  describe() {
    return `${this.blockedSpaces.size} blocked space(s) · ${this.hazardSpaces.size} hazard space(s) · ${this.blockedPortals.size} blocked door(s)`;
  }

  applyPick(pick: SurfacePick, mode: ScenarioEditMode, hazardMultiplier = 3) {
    const cell = this.byCell.get(pick.cellId);
    if (!cell) return { changed: false, message: "Navigation cell not found." };

    if (mode === "block-space") {
      const spaceId = cell.space_id;
      if (!spaceId) return { changed: false, message: "This navigation cell has no semantic space." };
      toggleSet(this.blockedSpaces, spaceId);
      if (this.blockedSpaces.has(spaceId)) this.hazardSpaces.delete(spaceId);
      this.rebuildOverlay();
      return { changed: true, message: `Space ${spaceId} ${this.blockedSpaces.has(spaceId) ? "blocked" : "unblocked"}.` };
    }

    if (mode === "hazard-space") {
      const spaceId = cell.space_id;
      if (!spaceId) return { changed: false, message: "This navigation cell has no semantic space." };
      if (this.hazardSpaces.has(spaceId)) this.hazardSpaces.delete(spaceId);
      else {
        this.hazardSpaces.set(spaceId, Math.max(1, hazardMultiplier || 3));
        this.blockedSpaces.delete(spaceId);
      }
      this.rebuildOverlay();
      return { changed: true, message: `Hazard cost for ${spaceId} ${this.hazardSpaces.has(spaceId) ? `set to ${this.hazardSpaces.get(spaceId)}x` : "cleared"}.` };
    }

    if (mode === "block-door") {
      const door = this.nearestDoor(pick.point, 2.0);
      if (!door) return { changed: false, message: "No semantic door within 2 m of the click." };
      toggleSet(this.blockedPortals, door.id);
      this.rebuildOverlay();
      return { changed: true, message: `Door ${door.id} ${this.blockedPortals.has(door.id) ? "blocked" : "unblocked"}.` };
    }

    return { changed: false, message: "Population mode is handled by the population-source tool." };
  }

  private nearestDoor(point: Vec3, maxDistanceM: number) {
    let best: { id: string; position_m: Vec3; distance: number } | undefined;
    for (const portal of this.model.portals ?? []) {
      if (portal.kind !== "door") continue;
      const distance = Math.hypot(
        portal.position_m[0] - point[0],
        portal.position_m[1] - point[1],
        portal.position_m[2] - point[2],
      );
      if (distance > maxDistanceM || (best && distance >= best.distance)) continue;
      best = { id: portal.id, position_m: portal.position_m, distance };
    }
    return best;
  }

  private rebuildOverlay() {
    disposeGroup(this.group);
    const cells = this.model.cells ?? [];
    for (const spaceId of this.blockedSpaces) {
      this.addSpaceOverlay(cells.filter((cell) => cell.space_id === spaceId), 0xd93232, 0.42);
    }
    for (const spaceId of this.hazardSpaces.keys()) {
      this.addSpaceOverlay(cells.filter((cell) => cell.space_id === spaceId), 0xffa31a, 0.38);
    }
    for (const portal of this.model.portals ?? []) {
      if (!this.blockedPortals.has(portal.id)) continue;
      const marker = new THREE.Mesh(
        new THREE.BoxGeometry(0.55, 1.9, 0.18),
        new THREE.MeshStandardMaterial({ color: 0xd93232, emissive: 0x500000, transparent: true, opacity: 0.85 }),
      );
      marker.position.copy(toThree(portal.position_m));
      marker.position.y += 0.95;
      marker.renderOrder = 60;
      marker.userData.portalId = portal.id;
      this.group.add(marker);
    }
  }

  private addSpaceOverlay(cells: NavCell[], color: number, opacity: number) {
    if (!cells.length) return;
    const positions: number[] = [];
    for (const cell of cells) {
      for (const vertex of cell.vertices_m) {
        const p = toThree(vertex);
        positions.push(p.x, p.y + 0.025, p.z);
      }
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const mesh = new THREE.Mesh(
      geometry,
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false }),
    );
    mesh.renderOrder = 55;
    mesh.raycast = () => undefined;
    this.group.add(mesh);
  }
}

function toggleSet(values: Set<string>, value: string) {
  if (values.has(value)) values.delete(value);
  else values.add(value);
}

function disposeGroup(group: THREE.Group) {
  for (const child of [...group.children]) {
    group.remove(child);
    if (!(child instanceof THREE.Mesh)) continue;
    child.geometry.dispose();
    if (Array.isArray(child.material)) child.material.forEach((material) => material.dispose());
    else child.material.dispose();
  }
}
