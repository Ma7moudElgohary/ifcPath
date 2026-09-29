import { expect, test } from "@playwright/test";
import path from "node:path";

const ifcPath = process.env.IFCPATH_E2E_IFC;
const baseUrl = process.env.IFCPATH_E2E_URL ?? "http://127.0.0.1:8767";

test("real IFC renders and builds a ready INAV in the browser", async ({ page }) => {
  test.skip(!ifcPath, "IFCPATH_E2E_IFC must point to a real IFC file");
  const pageErrors: string[] = [];
  page.on("pageerror", (error) => pageErrors.push(error.message));

  await page.goto("/", { waitUntil: "domcontentloaded" });
  await expect(page.locator("#ifc")).toBeVisible();
  await expect(page.locator("#study-health")).toBeVisible();

  // Auto-INAV reads this field at the moment the IFC input changes.
  await page.locator("#study-api").fill(baseUrl);
  await page.locator("#ifc").setInputFiles(path.resolve(ifcPath!));

  await waitForTerminalState(page, "inavBuild", 210_000);
  await waitForTerminalState(page, "ifcViewer", 210_000);

  const html = page.locator("html");
  expect(await html.getAttribute("data-inav-build"), await html.getAttribute("data-inav-error") ?? "INAV build failed").toBe("ready");
  expect(await html.getAttribute("data-ifc-viewer"), await html.getAttribute("data-ifc-viewer-error") ?? "IFC viewer failed").toBe("ready");
  await expect(html).toHaveAttribute("data-inav-ready", "true");

  const cellCount = Number(await html.getAttribute("data-inav-cells"));
  expect(cellCount).toBeGreaterThan(50);
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
