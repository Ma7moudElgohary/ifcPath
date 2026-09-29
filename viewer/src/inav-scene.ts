import * as THREE from "three";

export type Vec3 = [number, number, number];
export type NavCell = {
  id: string;
  vertices_m: [Vec3, Vec3, Vec3];
  neighbor_ids?: string[];
  terrain?: string;
  space_id?: string | null;
  level_id?: string | null;
  portals?: Record<string, [Vec3, Vec3]>;
  portal_ids?: Record<string, string>;
};
export type InavPortal = {
  id: string;
  kind: string;
  position_m: Vec3;
  from_space_id?: string | null;
  to_space_id?: string | null;
  is_exit?: boolean;
};
export type InavModel = { cells?: NavCell[]; portals?: InavPortal[]; metadata?: Record<string, unknown> };
export type SurfacePick = { cellId: string; point: Vec3 };

const TERRAIN_COST: Record<string, number> = { open: 1, ramp: 1.1, stair: 1.25 };
const TERRAIN_COLOR: Record<string, THREE.Color> = {
  open: new THREE.Color(0x33aa66),
  ramp: new THREE.Color(0x3388dd),
  stair: new THREE.Color(0xf09a35),
};
const EPSILON = 1e-9;

export const toThree = ([x, y, z]: Vec3) => new THREE.Vector3(x, z, -y);
export const fromThree = (p: THREE.Vector3): Vec3 => [p.x, -p.z, p.y];

export class InavScene {
  private group = new THREE.Group();
  private routeGroup = new THREE.Group();
  private mesh?: THREE.Mesh;
  private cells: NavCell[] = [];
  private model: InavModel = {};
  private byId = new Map<string, NavCell>();
  private blockedPortalIds = new Set<string>();
  private boundsBox = new THREE.Box3();

  constructor(private scene: THREE.Scene) {
    this.group.name = "IfcPath navigation surface";
    this.routeGroup.name = "IfcPath route";
    scene.add(this.group, this.routeGroup);
  }

  load(model: InavModel) {
    this.group.clear();
    this.clearRoute();
    this.model = model;
    this.cells = model.cells ?? [];
    this.byId = new Map(this.cells.map((cell) => [cell.id, cell]));
    this.blockedPortalIds.clear();
    this.boundsBox.makeEmpty();

    const positions: number[] = [];
    const colors: number[] = [];
    for (const cell of this.cells) {
      const color = TERRAIN_COLOR[cell.terrain ?? "open"] ?? new THREE.Color(0x999999);
      for (const vertex of cell.vertices_m) {
        const p = toThree(vertex);
        positions.push(p.x, p.y, p.z);
        colors.push(color.r, color.g, color.b);
        this.boundsBox.expandByPoint(p);
      }
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    geometry.computeVertexNormals();

    const material = new THREE.MeshBasicMaterial({
      transparent: true,
      opacity: 0.28,
      side: THREE.DoubleSide,
      vertexColors: true,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    });
    this.mesh = new THREE.Mesh(geometry, material);
    this.mesh.name = "IfcPath pickable navmesh";
    this.group.add(this.mesh);

    const wire = new THREE.LineSegments(
      new THREE.WireframeGeometry(geometry),
      new THREE.LineBasicMaterial({ transparent: true, opacity: 0.35 }),
    );
    wire.raycast = () => undefined;
    this.group.add(wire);
  }

  setVisible(visible: boolean) {
    this.group.visible = visible;
  }

  bounds() {
    return this.boundsBox.clone();
  }

  portalIds() {
    return (this.model.portals ?? []).filter((p) => p.kind === "door").map((p) => p.id);
  }

  setBlockedPortalIds(ids: Iterable<string>) {
    this.blockedPortalIds = new Set([...ids].filter(Boolean));
  }

  pick(clientX: number, clientY: number, canvas: HTMLElement, camera: THREE.Camera): SurfacePick | null {
    if (!this.mesh) return null;
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);
    const hit = raycaster.intersectObject(this.mesh, false)[0];
    if (!hit || hit.faceIndex == null) return null;
    const cell = this.cells[hit.faceIndex];
    if (!cell) return null;
    return { cellId: cell.id, point: fromThree(hit.point) };
  }

  route(start: SurfacePick, goal: SurfacePick): Vec3[] {
    const corridor = this.findCorridor(start.cellId, goal.cellId);
    if (!corridor.length) return [];
    if (corridor.length === 1) return [start.point, goal.point];

    const portals: Array<[Vec3, Vec3]> = [[start.point, start.point]];
    for (let i = 0; i + 1 < corridor.length; i++) {
      const current = this.byId.get(corridor[i])!;
      const next = this.byId.get(corridor[i + 1])!;
      portals.push(orientedPortal(current, next, portalSegment(current, next)));
    }
    portals.push([goal.point, goal.point]);
    return stringPull(portals);
  }

  routeLength(route: Vec3[]) {
    let total = 0;
    for (let i = 0; i + 1 < route.length; i++) total += distance(route[i], route[i + 1]);
    return total;
  }

  drawRoute(route: Vec3[], start?: Vec3, goal?: Vec3) {
    this.clearRoute();
    if (route.length >= 2) {
      const geometry = new THREE.BufferGeometry().setFromPoints(route.map(toThree));
      const material = new THREE.LineBasicMaterial({ color: 0xff2255, depthTest: false });
      const line = new THREE.Line(geometry, material);
      line.renderOrder = 30;
      this.routeGroup.add(line);
    }
    if (start) this.routeGroup.add(this.marker(start, 0x00cc66));
    if (goal) this.routeGroup.add(this.marker(goal, 0xff3344));
  }

  clearRoute() {
    for (const child of [...this.routeGroup.children]) {
      this.routeGroup.remove(child);
      const obj = child as THREE.Mesh;
      obj.geometry?.dispose();
      const material = obj.material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(material)) material.forEach((m) => m.dispose());
      else material?.dispose();
    }
  }

  private marker(point: Vec3, color: number) {
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.16, 16, 12),
      new THREE.MeshBasicMaterial({ color, depthTest: false }),
    );
    mesh.position.copy(toThree(point));
    mesh.renderOrder = 40;
    return mesh;
  }

  private findCorridor(startId: string, goalId: string): string[] {
    if (!this.byId.has(startId) || !this.byId.has(goalId)) return [];
    const goal = centroid(this.byId.get(goalId)!);
    const open: Array<[number, string]> = [[0, startId]];
    const g = new Map<string, number>([[startId, 0]]);
    const previous = new Map<string, string>();

    while (open.length) {
      open.sort((a, b) => a[0] - b[0]);
      const [, currentId] = open.shift()!;
      if (currentId === goalId) break;
      const current = this.byId.get(currentId)!;
      const cp = centroid(current);
      for (const neighborId of current.neighbor_ids ?? []) {
        const neighbor = this.byId.get(neighborId);
        if (!neighbor) continue;
        const portalId = current.portal_ids?.[neighborId] ?? neighbor.portal_ids?.[currentId];
        if (portalId && this.blockedPortalIds.has(portalId)) continue;
        const np = centroid(neighbor);
        const terrainCost = TERRAIN_COST[neighbor.terrain ?? "open"] ?? 1;
        const candidate = (g.get(currentId) ?? Infinity) + distance(cp, np) * terrainCost;
        if (candidate >= (g.get(neighborId) ?? Infinity)) continue;
        g.set(neighborId, candidate);
        previous.set(neighborId, currentId);
        open.push([candidate + distance(np, goal), neighborId]);
      }
    }

    if (!g.has(goalId)) return [];
    const corridor = [goalId];
    while (corridor[corridor.length - 1] !== startId) {
      const p = previous.get(corridor[corridor.length - 1]);
      if (!p) return [];
      corridor.push(p);
    }
    corridor.reverse();
    return corridor;
  }
}

function centroid(cell: NavCell): Vec3 {
  const [a, b, c] = cell.vertices_m;
  return [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
}

function distance(a: Vec3, b: Vec3) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function portalSegment(a: NavCell, b: NavCell): [Vec3, Vec3] {
  const stored = a.portals?.[b.id] ?? b.portals?.[a.id];
  if (stored) return stored;

  const common = a.vertices_m.filter((pa) => b.vertices_m.some((pb) => distance(pa, pb) <= 1e-4));
  if (common.length >= 2) return [common[0], common[1]];

  const ca = centroid(a), cb = centroid(b);
  const midpoint: Vec3 = [(ca[0] + cb[0]) / 2, (ca[1] + cb[1]) / 2, (ca[2] + cb[2]) / 2];
  return [midpoint, midpoint];
}

function orientedPortal(current: NavCell, next: NavCell, [a, b]: [Vec3, Vec3]): [Vec3, Vec3] {
  const currentCenter = centroid(current);
  const nextCenter = centroid(next);
  const dx = nextCenter[0] - currentCenter[0];
  const dy = nextCenter[1] - currentCenter[1];
  const mx = (a[0] + b[0]) * 0.5;
  const my = (a[1] + b[1]) * 0.5;
  const crossA = dx * (a[1] - my) - dy * (a[0] - mx);
  const crossB = dx * (b[1] - my) - dy * (b[0] - mx);
  return crossA >= crossB ? [b, a] : [a, b];
}

function stringPull(portals: Array<[Vec3, Vec3]>): Vec3[] {
  if (!portals.length) return [];

  const path: Vec3[] = [portals[0][0]];
  let apex = portals[0][0];
  let left = portals[0][0];
  let right = portals[0][1];
  let apexIndex = 0;
  let leftIndex = 0;
  let rightIndex = 0;
  let i = 1;

  while (i < portals.length) {
    const [newLeft, newRight] = portals[i];

    if (triArea2(apex, right, newRight) <= EPSILON) {
      if (vecEqual(apex, right) || triArea2(apex, left, newRight) > EPSILON) {
        right = newRight;
        rightIndex = i;
      } else {
        path.push(left);
        apex = left;
        apexIndex = leftIndex;
        left = apex;
        right = apex;
        leftIndex = apexIndex;
        rightIndex = apexIndex;
        i = apexIndex + 1;
        continue;
      }
    }

    if (triArea2(apex, left, newLeft) >= -EPSILON) {
      if (vecEqual(apex, left) || triArea2(apex, right, newLeft) < -EPSILON) {
        left = newLeft;
        leftIndex = i;
      } else {
        path.push(right);
        apex = right;
        apexIndex = rightIndex;
        left = apex;
        right = apex;
        leftIndex = apexIndex;
        rightIndex = apexIndex;
        i = apexIndex + 1;
        continue;
      }
    }
    i += 1;
  }

  const goal = portals[portals.length - 1][0];
  if (!vecEqual(path[path.length - 1], goal)) path.push(goal);
  return removeConsecutiveDuplicates(path);
}

function triArea2(a: Vec3, b: Vec3, c: Vec3) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

function vecEqual(a: Vec3, b: Vec3) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]) <= EPSILON;
}

function removeConsecutiveDuplicates(points: Vec3[]) {
  const result: Vec3[] = [];
  for (const point of points) if (!result.length || !vecEqual(result[result.length - 1], point)) result.push(point);
  return result;
}
