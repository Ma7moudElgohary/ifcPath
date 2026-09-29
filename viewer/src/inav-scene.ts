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
};
export type InavModel = { cells?: NavCell[] };
export type SurfacePick = { cellId: string; point: Vec3 };

const TERRAIN_COST: Record<string, number> = { open: 1, ramp: 1.1, stair: 1.25 };
const TERRAIN_COLOR: Record<string, THREE.Color> = {
  open: new THREE.Color(0x33aa66),
  ramp: new THREE.Color(0x3388dd),
  stair: new THREE.Color(0xf09a35),
};

export const toThree = ([x, y, z]: Vec3) => new THREE.Vector3(x, z, -y);
export const fromThree = (p: THREE.Vector3): Vec3 => [p.x, -p.z, p.y];

export class InavScene {
  private group = new THREE.Group();
  private routeGroup = new THREE.Group();
  private mesh?: THREE.Mesh;
  private cells: NavCell[] = [];
  private byId = new Map<string, NavCell>();

  constructor(private scene: THREE.Scene) {
    this.group.name = "IfcPath navigation surface";
    this.routeGroup.name = "IfcPath route";
    scene.add(this.group, this.routeGroup);
  }

  load(model: InavModel) {
    this.group.clear();
    this.clearRoute();
    this.cells = model.cells ?? [];
    this.byId = new Map(this.cells.map((cell) => [cell.id, cell]));

    const positions: number[] = [];
    const colors: number[] = [];
    for (const cell of this.cells) {
      const color = TERRAIN_COLOR[cell.terrain ?? "open"] ?? new THREE.Color(0x999999);
      for (const vertex of cell.vertices_m) {
        const p = toThree(vertex);
        positions.push(p.x, p.y, p.z);
        colors.push(color.r, color.g, color.b);
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
    const points: Vec3[] = [start.point];
    for (let i = 0; i + 1 < corridor.length; i++) {
      const a = this.byId.get(corridor[i])!;
      const b = this.byId.get(corridor[i + 1])!;
      points.push(this.portalMidpoint(a, b));
    }
    points.push(goal.point);
    return simplifyPolyline(points);
  }

  drawRoute(route: Vec3[], start?: Vec3, goal?: Vec3) {
    this.clearRoute();
    if (route.length >= 2) {
      const geometry = new THREE.BufferGeometry().setFromPoints(route.map(toThree));
      this.routeGroup.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0xff2255 })));
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
    mesh.renderOrder = 20;
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

  private portalMidpoint(a: NavCell, b: NavCell): Vec3 {
    const portal = a.portals?.[b.id] ?? b.portals?.[a.id];
    if (portal) return midpoint(portal[0], portal[1]);
    return midpoint(centroid(a), centroid(b));
  }
}

function centroid(cell: NavCell): Vec3 {
  const [a, b, c] = cell.vertices_m;
  return [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
}
function midpoint(a: Vec3, b: Vec3): Vec3 {
  return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
}
function distance(a: Vec3, b: Vec3) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
function simplifyPolyline(points: Vec3[]): Vec3[] {
  if (points.length <= 2) return points;
  const result = [points[0]];
  for (let i = 1; i + 1 < points.length; i++) {
    const a = result[result.length - 1], b = points[i], c = points[i + 1];
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const bc = [c[0] - b[0], c[1] - b[1], c[2] - b[2]];
    const cross = [ab[1] * bc[2] - ab[2] * bc[1], ab[2] * bc[0] - ab[0] * bc[2], ab[0] * bc[1] - ab[1] * bc[0]];
    if (Math.hypot(cross[0], cross[1], cross[2]) > 1e-5) result.push(b);
  }
  result.push(points[points.length - 1]);
  return result;
}
