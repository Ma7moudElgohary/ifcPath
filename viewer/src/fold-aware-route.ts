export type RoutingVec3 = [number, number, number];

export type RoutingCell = {
  id: string;
  vertices_m: [RoutingVec3, RoutingVec3, RoutingVec3];
  portals?: Record<string, [RoutingVec3, RoutingVec3]>;
};

const EPSILON = 1e-9;
const COPLANAR_TOLERANCE_M = 1e-3;

type PlaneBasis = {
  origin: RoutingVec3;
  u: RoutingVec3;
  v: RoutingVec3;
  normal: RoutingVec3;
};

/**
 * Convert an ordered NavCell corridor into a surface-constrained XYZ route.
 *
 * A classic funnel is planar. Indoor stairs are not: successive treads and
 * landings can overlap in XY while living at different Z values. Running the
 * funnel directly in building XY lets it shortcut across those folds. We keep
 * the funnel on coplanar runs, but every non-coplanar fold is traversed
 * explicitly by projecting the stored portal onto both adjacent triangles.
 */
export function foldAwareCorridorRoute<T extends RoutingCell>(
  byId: Map<string, T>,
  corridor: string[],
  start: RoutingVec3,
  goal: RoutingVec3,
): RoutingVec3[] {
  if (!corridor.length) return [];
  if (corridor.length === 1) return dedupe3D([start, goal]);

  const result: RoutingVec3[] = [];
  let runStartIndex = 0;
  let runStartPoint = start;

  for (let index = 0; index + 1 < corridor.length; index += 1) {
    const current = byId.get(corridor[index]);
    const next = byId.get(corridor[index + 1]);
    if (!current || !next) return [];
    if (cellsCoplanar(current, next)) continue;

    const midpoint = segmentMidpoint(portalSegment(current, next));
    const exitPoint = closestPointOnTriangle(midpoint, ...current.vertices_m);
    const entryPoint = closestPointOnTriangle(midpoint, ...next.vertices_m);

    appendPoints(
      result,
      planarFunnelRun(
        byId,
        corridor.slice(runStartIndex, index + 1),
        runStartPoint,
        exitPoint,
      ),
    );
    appendPoints(result, [entryPoint]);
    runStartIndex = index + 1;
    runStartPoint = entryPoint;
  }

  appendPoints(
    result,
    planarFunnelRun(byId, corridor.slice(runStartIndex), runStartPoint, goal),
  );
  return result;
}

function planarFunnelRun<T extends RoutingCell>(
  byId: Map<string, T>,
  corridor: string[],
  start: RoutingVec3,
  goal: RoutingVec3,
): RoutingVec3[] {
  if (!corridor.length) return [];
  if (corridor.length === 1) return dedupe3D([start, goal]);

  const first = byId.get(corridor[0]);
  if (!first) return [];
  const basis = planeBasis(first);
  if (!basis) return safePortalMidpointRoute(byId, corridor, start, goal);

  const project = (point: RoutingVec3): RoutingVec3 => {
    const delta = sub(point, basis.origin);
    return [dot(delta, basis.u), dot(delta, basis.v), 0];
  };
  const unproject = (point: RoutingVec3): RoutingVec3 =>
    add(basis.origin, add(scale(basis.u, point[0]), scale(basis.v, point[1])));

  const portals: Array<[RoutingVec3, RoutingVec3]> = [[project(start), project(start)]];
  for (let index = 0; index + 1 < corridor.length; index += 1) {
    const current = byId.get(corridor[index]);
    const next = byId.get(corridor[index + 1]);
    if (!current || !next) return [];
    const [a, b] = portalSegment(current, next);
    portals.push(
      orientedProjectedPortal(
        project(centroid(current)),
        project(centroid(next)),
        [project(a), project(b)],
      ),
    );
  }
  portals.push([project(goal), project(goal)]);

  const route = stringPull(portals).map(unproject);
  if (route.length) {
    route[0] = start;
    route[route.length - 1] = goal;
  }
  return dedupe3D(route);
}

function safePortalMidpointRoute<T extends RoutingCell>(
  byId: Map<string, T>,
  corridor: string[],
  start: RoutingVec3,
  goal: RoutingVec3,
): RoutingVec3[] {
  const points: RoutingVec3[] = [start];
  for (let index = 0; index + 1 < corridor.length; index += 1) {
    const current = byId.get(corridor[index]);
    const next = byId.get(corridor[index + 1]);
    if (!current || !next) return [];
    points.push(segmentMidpoint(portalSegment(current, next)));
  }
  points.push(goal);
  return dedupe3D(points);
}

function cellsCoplanar(a: RoutingCell, b: RoutingCell) {
  const basis = planeBasis(a);
  if (!basis) return false;
  return b.vertices_m.every(
    (vertex) => Math.abs(dot(sub(vertex, basis.origin), basis.normal)) <= COPLANAR_TOLERANCE_M,
  );
}

function planeBasis(cell: RoutingCell): PlaneBasis | null {
  const [a, b, c] = cell.vertices_m;
  const ab = sub(b, a);
  const ac = sub(c, a);
  let normal = cross(ab, ac);
  const normalLength = length(normal);
  if (normalLength <= EPSILON) return null;
  normal = scale(normal, 1 / normalLength);

  let u = length(ab) > EPSILON ? ab : ac;
  const uLength = length(u);
  if (uLength <= EPSILON) return null;
  u = scale(u, 1 / uLength);

  let v = cross(normal, u);
  const vLength = length(v);
  if (vLength <= EPSILON) return null;
  v = scale(v, 1 / vLength);
  return { origin: a, u, v, normal };
}

function portalSegment(a: RoutingCell, b: RoutingCell): [RoutingVec3, RoutingVec3] {
  const stored = a.portals?.[b.id] ?? b.portals?.[a.id];
  if (stored) return stored;

  const common = a.vertices_m.filter((pa) =>
    b.vertices_m.some((pb) => distance(pa, pb) <= 1e-4),
  );
  if (common.length >= 2) return [common[0], common[1]];

  const ca = centroid(a);
  const cb = centroid(b);
  const midpoint: RoutingVec3 = [
    (ca[0] + cb[0]) * 0.5,
    (ca[1] + cb[1]) * 0.5,
    (ca[2] + cb[2]) * 0.5,
  ];
  return [midpoint, midpoint];
}

function orientedProjectedPortal(
  currentCenter: RoutingVec3,
  nextCenter: RoutingVec3,
  [a, b]: [RoutingVec3, RoutingVec3],
): [RoutingVec3, RoutingVec3] {
  const dx = nextCenter[0] - currentCenter[0];
  const dy = nextCenter[1] - currentCenter[1];
  const mx = (a[0] + b[0]) * 0.5;
  const my = (a[1] + b[1]) * 0.5;
  const crossA = dx * (a[1] - my) - dy * (a[0] - mx);
  const crossB = dx * (b[1] - my) - dy * (b[0] - mx);
  return crossA >= crossB ? [b, a] : [a, b];
}

function stringPull(portals: Array<[RoutingVec3, RoutingVec3]>): RoutingVec3[] {
  if (!portals.length) return [];

  const path: RoutingVec3[] = [portals[0][0]];
  let apex = portals[0][0];
  let left = portals[0][0];
  let right = portals[0][1];
  let apexIndex = 0;
  let leftIndex = 0;
  let rightIndex = 0;
  let index = 1;

  while (index < portals.length) {
    const [newLeft, newRight] = portals[index];

    if (triArea2(apex, right, newRight) <= EPSILON) {
      if (vecEqual2D(apex, right) || triArea2(apex, left, newRight) > EPSILON) {
        right = newRight;
        rightIndex = index;
      } else {
        path.push(left);
        apex = left;
        apexIndex = leftIndex;
        left = apex;
        right = apex;
        leftIndex = apexIndex;
        rightIndex = apexIndex;
        index = apexIndex + 1;
        continue;
      }
    }

    if (triArea2(apex, left, newLeft) >= -EPSILON) {
      if (vecEqual2D(apex, left) || triArea2(apex, right, newLeft) < -EPSILON) {
        left = newLeft;
        leftIndex = index;
      } else {
        path.push(right);
        apex = right;
        apexIndex = rightIndex;
        left = apex;
        right = apex;
        leftIndex = apexIndex;
        rightIndex = apexIndex;
        index = apexIndex + 1;
        continue;
      }
    }
    index += 1;
  }

  const goal = portals[portals.length - 1][0];
  if (!vecEqual2D(path[path.length - 1], goal)) path.push(goal);
  return removeConsecutiveDuplicates2D(path);
}

function closestPointOnTriangle(
  p: RoutingVec3,
  a: RoutingVec3,
  b: RoutingVec3,
  c: RoutingVec3,
): RoutingVec3 {
  const ab = sub(b, a);
  const ac = sub(c, a);
  const ap = sub(p, a);
  const d1 = dot(ab, ap);
  const d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return a;

  const bp = sub(p, b);
  const d3 = dot(ab, bp);
  const d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return b;

  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const t = d1 / (d1 - d3);
    return add(a, scale(ab, t));
  }

  const cp = sub(p, c);
  const d5 = dot(ab, cp);
  const d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return c;

  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const t = d2 / (d2 - d6);
    return add(a, scale(ac, t));
  }

  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return add(b, scale(sub(c, b), t));
  }

  const denominator = 1 / (va + vb + vc);
  const v = vb * denominator;
  const w = vc * denominator;
  return add(a, add(scale(ab, v), scale(ac, w)));
}

function centroid(cell: RoutingCell): RoutingVec3 {
  const [a, b, c] = cell.vertices_m;
  return [
    (a[0] + b[0] + c[0]) / 3,
    (a[1] + b[1] + c[1]) / 3,
    (a[2] + b[2] + c[2]) / 3,
  ];
}

function segmentMidpoint([a, b]: [RoutingVec3, RoutingVec3]): RoutingVec3 {
  return [
    (a[0] + b[0]) * 0.5,
    (a[1] + b[1]) * 0.5,
    (a[2] + b[2]) * 0.5,
  ];
}

function triArea2(a: RoutingVec3, b: RoutingVec3, c: RoutingVec3) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

function vecEqual2D(a: RoutingVec3, b: RoutingVec3) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]) <= EPSILON;
}

function removeConsecutiveDuplicates2D(points: RoutingVec3[]) {
  const result: RoutingVec3[] = [];
  for (const point of points) {
    if (!result.length || !vecEqual2D(result[result.length - 1], point)) result.push(point);
  }
  return result;
}

function dedupe3D(points: RoutingVec3[]) {
  const result: RoutingVec3[] = [];
  appendPoints(result, points);
  return result;
}

function appendPoints(target: RoutingVec3[], points: RoutingVec3[]) {
  for (const point of points) {
    if (!target.length || distance(target[target.length - 1], point) > EPSILON) target.push(point);
  }
}

function sub(a: RoutingVec3, b: RoutingVec3): RoutingVec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function add(a: RoutingVec3, b: RoutingVec3): RoutingVec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(a: RoutingVec3, value: number): RoutingVec3 {
  return [a[0] * value, a[1] * value, a[2] * value];
}

function dot(a: RoutingVec3, b: RoutingVec3) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a: RoutingVec3, b: RoutingVec3): RoutingVec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function length(a: RoutingVec3) {
  return Math.sqrt(dot(a, a));
}

function distance(a: RoutingVec3, b: RoutingVec3) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
