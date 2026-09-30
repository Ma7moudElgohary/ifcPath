import { foldAwareCorridorRoute, type RoutingCell, type RoutingVec3 } from "./fold-aware-route";

export function assertFoldAwareRouteSelfTest() {
  const cells: RoutingCell[] = [
    {
      id: "tread:0",
      vertices_m: [[0, 1, 0], [1, 1, 0], [0.5, 0, 0]],
      portals: { "tread:1": [[0.2, 1, 0.1], [0.8, 1, 0.1]] },
    },
    {
      id: "tread:1",
      vertices_m: [[0, 1, 0.2], [1, 1, 0.2], [0.5, 2, 0.2]],
      portals: {
        "tread:0": [[0.2, 1, 0.1], [0.8, 1, 0.1]],
        "tread:2": [[0.2, 2, 0.3], [0.8, 2, 0.3]],
      },
    },
    {
      id: "tread:2",
      vertices_m: [[0, 2, 0.4], [1, 2, 0.4], [0.5, 3, 0.4]],
      portals: { "tread:1": [[0.2, 2, 0.3], [0.8, 2, 0.3]] },
    },
  ];
  const byId = new Map(cells.map((cell) => [cell.id, cell]));
  const start: RoutingVec3 = [0.5, 0.25, 0];
  const goal: RoutingVec3 = [0.5, 2.75, 0.4];
  const route = foldAwareCorridorRoute(byId, ["tread:0", "tread:1", "tread:2"], start, goal);

  if (route.length < 6) throw new Error(`fold-aware route collapsed to ${route.length} points`);
  const maxRise = Math.max(...route.slice(1).map((point, index) => Math.abs(point[2] - route[index][2])));
  if (maxRise > 0.200001) throw new Error(`fold-aware route skipped a tread: rise=${maxRise}`);
  if (!route.some((point) => Math.abs(point[2] - 0.2) < 1e-9)) {
    throw new Error("fold-aware route did not preserve the intermediate tread elevation");
  }
}
