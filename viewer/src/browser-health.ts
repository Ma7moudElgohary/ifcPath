export {};

async function attachViewerHealth() {
  const ifcInput = await waitFor<HTMLInputElement>("#ifc");
  const status = await waitFor<HTMLElement>("#status");
  const root = document.documentElement;

  ifcInput.addEventListener("change", () => {
    if (!ifcInput.files?.length) return;
    root.dataset.ifcViewer = "loading";
    delete root.dataset.ifcViewerError;
  });

  const update = () => {
    const text = status.textContent ?? "";
    if (text.startsWith("IFC loaded in original coordinates")) {
      root.dataset.ifcViewer = "ready";
      root.dispatchEvent(new CustomEvent("ifcpath:ifc-viewer-ready"));
    }
  };
  new MutationObserver(update).observe(status, { childList: true, characterData: true, subtree: true });
  update();

  window.addEventListener("error", (event) => {
    if (root.dataset.ifcViewer === "loading") {
      root.dataset.ifcViewer = "error";
      root.dataset.ifcViewerError = event.message || "browser error during IFC import";
    }
  });
  window.addEventListener("unhandledrejection", (event) => {
    if (root.dataset.ifcViewer === "loading") {
      root.dataset.ifcViewer = "error";
      root.dataset.ifcViewerError = String(event.reason ?? "unhandled rejection during IFC import");
    }
  });

  root.dataset.viewerHealth = "ready";
  root.dispatchEvent(new CustomEvent("ifcpath:viewer-health-ready"));
}

async function waitFor<T extends Element>(selector: string): Promise<T> {
  for (let attempt = 0; attempt < 200; attempt++) {
    const element = document.querySelector<T>(selector);
    if (element) return element;
    await new Promise((resolve) => setTimeout(resolve, 25));
  }
  throw new Error(`viewer control did not initialize: ${selector}`);
}

void attachViewerHealth();
