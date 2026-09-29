import { copyFile, mkdir } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import path from "node:path";

const viewerRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const sourceRoot = path.join(viewerRoot, "node_modules", "web-ifc");
const destinationRoot = path.join(viewerRoot, "public", "wasm");

await mkdir(destinationRoot, { recursive: true });

for (const name of ["web-ifc.wasm", "web-ifc-mt.wasm", "web-ifc-mt.worker.js"]) {
  await copyFile(path.join(sourceRoot, name), path.join(destinationRoot, name));
}

console.log(`Bundled WebIFC WASM assets from ${sourceRoot} -> ${destinationRoot}`);
