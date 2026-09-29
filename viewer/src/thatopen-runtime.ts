import * as OBC from "@thatopen/components";

/**
 * Production runtime guard for That Open's IfcLoader.
 *
 * The upstream autoSetWasm() helper resolves the web-ifc peer dependency through
 * unpkg at runtime. That is convenient for examples, but it makes a packaged app
 * vulnerable to JS/WASM version drift and requires network access. IfcPath ships
 * the exact WASM files installed with its pinned web-ifc dependency instead.
 */
const originalSetup = OBC.IfcLoader.prototype.setup;

OBC.IfcLoader.prototype.setup = async function (config) {
  const wasmBase = new URL("wasm/", window.location.href).href;
  return originalSetup.call(this, {
    ...config,
    autoSetWasm: false,
    wasm: {
      path: wasmBase,
      absolute: true,
    },
  });
};

export {};
