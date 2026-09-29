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

  const html = page.locator("html");
  await expect(html).toHaveAttribute("data-inav-build", "ready", { timeout: 210_000 });
  await expect(html).toHaveAttribute("data-ifc-viewer", "ready", { timeout: 210_000 });
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
