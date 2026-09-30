import { expect, test } from "@playwright/test";
import {
  foldAwareCorridorRoute,
  type RoutingCell,
  type RoutingVec3,
} from "../src/fold-aware-route";

const distance = (a: RoutingVec3, b: RoutingVec3) =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

test("fold-aware browser route preserves stepped stair folds", () => {
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
  const route = foldAwareCorridorRoute(
    byId,
    ["tread:0", "tread:1", "tread:2"],
    start,
    goal,
  );

  expect(route.length).toBeGreaterThanOrEqual(6);
  expect(route[0]).toEqual(start);
  expect(route.at(-1)).toEqual(goal);
  expect(route.some((point) => Math.abs(point[2] - 0.2) < 1e-9)).toBe(true);

  const rises = route.slice(1).map((point, index) => Math.abs(point[2] - route[index][2]));
  expect(Math.max(...rises)).toBeLessThanOrEqual(0.200001);

  const length = route.slice(1).reduce(
    (sum, point, index) => sum + distance(route[index], point),
    0,
  );
  expect(length).toBeGreaterThan(distance(start, goal) + 0.1);
});
