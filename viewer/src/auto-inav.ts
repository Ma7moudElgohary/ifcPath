type BuildResponse = {
  model: Record<string, unknown>;
  qualification?: {
    valid?: boolean;
    stats?: Record<string, unknown>;
  };
};

/**
 * End-to-end IFC workflow adapter.
 *
 * The main viewer intentionally keeps its existing explicit INAV input path. This
 * module listens for an IFC selection, asks the local Python app to build the
 * qualified INAV, then feeds that JSON back through the same input event. No
 * routing/viewer logic is duplicated in the browser.
 */
void attachAutoBuild();

async function attachAutoBuild() {
  const ifcInput = await waitFor<HTMLInputElement>("#ifc");
  const inavInput = await waitFor<HTMLInputElement>("#inav");
  const status = await waitFor<HTMLElement>("#status");

  ifcInput.addEventListener("change", async () => {
    const file = ifcInput.files?.[0];
    if (!file) return;
    const apiInput = document.querySelector<HTMLInputElement>("#study-api");
    const apiBase = (apiInput?.value || window.location.origin).replace(/\/$/, "");
    const root = document.documentElement;
    root.dataset.inavBuild = "building";
    delete root.dataset.inavCells;
    delete root.dataset.inavReady;
    status.textContent = "Building continuous navigation surface from IFC…";

    try {
      const response = await fetch(`${apiBase}/inav/build`, {
        method: "POST",
        headers: {
          "Content-Type": "application/octet-stream",
          "X-IFC-Filename": file.name,
        },
        body: file,
      });
      if (!response.ok) {
        let detail = `${response.status} ${response.statusText}`;
        try {
          const error = await response.json() as { detail?: string };
          if (error.detail) detail = error.detail;
        } catch {
          // Keep HTTP status text when the server did not return JSON.
        }
        throw new Error(detail);
      }

      const result = await response.json() as BuildResponse;
      if (!result.model || typeof result.model !== "object") {
        throw new Error("local app returned no INAV model");
      }
      const stats = result.qualification?.stats ?? {};
      const ready = stats.navigation_ready ?? stats.surface_navigation_ready;
      const cells = Number(stats.surface_cell_count ?? (result.model as { cells?: unknown[] }).cells?.length ?? 0);
      const generated = new File(
        [JSON.stringify(result.model)],
        file.name.replace(/\.ifc$/i, "") + ".inav",
        { type: "application/json" },
      );
      const transfer = new DataTransfer();
      transfer.items.add(generated);
      inavInput.files = transfer.files;
      inavInput.dispatchEvent(new Event("change", { bubbles: true }));

      root.dataset.inavCells = String(cells);
      root.dataset.inavReady = String(ready === true);
      root.dataset.inavBuild = "ready";
      root.dispatchEvent(new CustomEvent("ifcpath:inav-ready", { detail: { cells, ready: ready === true } }));
      status.textContent = `IFC navigation built automatically · ${cells} cells${ready === true ? " · ready" : ""}`;
    } catch (error) {
      // Manual .inav loading remains available if the API is not running.
      const message = error instanceof Error ? error.message : String(error);
      root.dataset.inavBuild = "error";
      root.dataset.inavReady = "false";
      root.dataset.inavError = message;
      status.textContent = `IFC loaded visually; automatic INAV build unavailable (${message}). You can still load an INAV manually.`;
    }
  });
}

async function waitFor<T extends Element>(selector: string): Promise<T> {
  for (let attempt = 0; attempt < 200; attempt++) {
    const element = document.querySelector<T>(selector);
    if (element) return element;
    await new Promise((resolve) => setTimeout(resolve, 25));
  }
  throw new Error(`viewer control did not initialize: ${selector}`);
}
