import { expect, test } from "@playwright/test";
import path from "node:path";

const ifcPath = process.env.IFCPATH_E2E_IFC;
const baseUrl = process.env.IFCPATH_E2E_URL ?? "http://127.0.0.1:8767";

test("real IFC renders and builds a physical ready INAV in the browser", async ({ page }) => {
  test.skip(!ifcPath, "IFCPATH_E2E_IFC must point to a real IFC file");
  const pageErrors: string[] = [];
  page.on("pageerror", (error) => pageErrors.push(error.message));

  await page.goto("/", { waitUntil: "domcontentloaded" });
  await expect(page.locator("#ifc")).toBeVisible();
  await expect(page.locator("#study-health")).toBeVisible();

  const html = page.locator("html");
  await expect(html).toHaveAttribute("data-auto-inav", "ready", { timeout: 15_000 });
  await expect(html).toHaveAttribute("data-viewer-health", "ready", { timeout: 15_000 });

  // Capture the server response itself, not only UI state. The physical build can
  // return a multi-megabyte INAV; Chromium may evict that body from the inspector
  // cache before Playwright later calls response.json() on the frozen Windows app.
  // Compact qualification headers prove the same physical invariants without
  // retaining/decoding the large response a second time.
  const buildResponsePromise = page.waitForResponse(
    (response) =>
      response.request().method() === "POST" &&
      new URL(response.url()).pathname === "/inav/build",
    { timeout: 210_000 },
  );

  // Auto-INAV reads this field at the moment the IFC input changes.
  await page.locator("#study-api").fill(baseUrl);
  await page.locator("#ifc").setInputFiles(path.resolve(ifcPath!));

  const buildResponse = await buildResponsePromise;
  expect(buildResponse.ok(), `physical /inav/build failed with HTTP ${buildResponse.status()}`).toBeTruthy();
  const headers = buildResponse.headers();
  expect(headers["x-ifcpath-surface-source"]).toBe("ifc-physical-raycast");
  expect(headers["x-ifcpath-qualification-valid"]).toBe("true");
  expect(headers["x-ifcpath-surface-authoritative"]).toBe("true");
  expect(headers["x-ifcpath-surface-navigation-ready"]).toBe("true");
  expect(Number(headers["x-ifcpath-surface-unreachable-spaces"] ?? -1)).toBe(0);
  // Duplex must retain a real physical stair manifold rather than merely a flat
  // floor navmesh.
  expect(Number(headers["x-ifcpath-surface-vertical-transitions"] ?? 0)).toBeGreaterThan(0);
  expect(Number(headers["x-ifcpath-surface-cell-count"] ?? 0)).toBeGreaterThan(1_000);

  await waitForTerminalState(page, "inavBuild", 210_000);
  await waitForTerminalState(page, "ifcViewer", 210_000);

  expect(await html.getAttribute("data-inav-build"), await html.getAttribute("data-inav-error") ?? "INAV build failed").toBe("ready");
  expect(await html.getAttribute("data-ifc-viewer"), await html.getAttribute("data-ifc-viewer-error") ?? "IFC viewer failed").toBe("ready");
  await expect(html).toHaveAttribute("data-inav-ready", "true");

  const cellCount = Number(await html.getAttribute("data-inav-cells"));
  expect(cellCount).toBeGreaterThan(1_000);
  await expect(page.locator("#door-count")).toContainText("door portal");

  await page.locator("#study-health").click();
  await expect(page.locator("#study-status")).toContainText("Server ready");

  await expect(page.locator("#scenario-mode")).toBeEnabled();
  await expect(page.locator("#study-run")).toBeEnabled();
  expect(pageErrors).toEqual([]);
});

async function waitForTerminalState(
  page: import("@playwright/test").Page,
  datasetKey: "inavBuild" | "ifcViewer",
  timeout: number,
) {
  await page.waitForFunction(
    ({ key }) => {
      const value = document.documentElement.dataset[key];
      return value === "ready" || value === "error";
    },
    { key: datasetKey },
    { timeout },
  );
}
