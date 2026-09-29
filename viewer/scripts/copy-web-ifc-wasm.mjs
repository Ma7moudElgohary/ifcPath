import { access, copyFile, mkdir } from "node:fs/promises";
import { constants } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const viewerRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const sourceRoot = path.join(viewerRoot, "node_modules", "web-ifc");
const destinationRoot = path.join(viewerRoot, "public", "wasm");

await mkdir(destinationRoot, { recursive: true });

async function copyRequired(name) {
  const source = path.join(sourceRoot, name);
  await access(source, constants.R_OK);
  await copyFile(source, path.join(destinationRoot, name));
}

async function copyOptional(name) {
  const source = path.join(sourceRoot, name);
  try {
    await access(source, constants.R_OK);
  } catch {
    return false;
  }
  await copyFile(source, path.join(destinationRoot, name));
  return true;
}

await copyRequired("web-ifc.wasm");
for (const name of ["web-ifc-mt.wasm", "web-ifc-mt.worker.js", "web-ifc-mt.worker.mjs"]) {
  await copyOptional(name);
}

console.log(`Bundled WebIFC runtime assets from ${sourceRoot} -> ${destinationRoot}`);
