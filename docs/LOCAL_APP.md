# IfcPath Local Web Application

IfcPath can run as one local application combining:

- That Open / Fragments IFC visualization
- Python/IfcOpenShell continuous navigation-surface generation
- qualified `.inav` semantics
- interactive route/scenario authoring
- JuPedSim or kinematic evacuation studies
- high-count solver playback and analytics

## Fast path

From a source checkout with Python and Node.js 22+ installed:

```bash
pip install -e '.[study]'
ifcpath-app
```

`ifcpath-app` installs viewer npm dependencies when `node_modules` is absent, builds the Vite viewer, starts the local server at `http://127.0.0.1:8765`, and opens the browser.

Then select an IFC. The browser does two things in parallel:

1. That Open loads the IFC for BIM visualization.
2. The same IFC bytes are posted locally to `/inav/build`; IfcPath/IfcOpenShell generates and validates the continuous navigation model and the browser loads it automatically.

A separate `.inav` file is no longer required for the normal local workflow. Manual INAV loading remains available for debugging, interoperability and offline/reproducible studies.

## Reuse an existing viewer build

```bash
cd viewer
npm install
npm run build
cd ..
ifcpath-app --skip-build
```

Or run only the API/static server:

```bash
ifcpath-study-server --viewer-dir viewer/dist --open
```

## API

The local server exposes:

- `GET /health` — server, viewer and JuPedSim availability
- `POST /inav/build` — raw IFC STEP bytes in the request body; optional `X-IFC-Filename` header; returns `{model, qualification}`
- `POST /study/run` — existing browser-authored evacuation study endpoint
- `/` — built That Open viewer when `viewer/dist` is available

`/inav/build` runs the same qualified IFCPath pipeline used by the CLI, including continuous floors/stairs/ramps, semantic doors, open-plan recovery, vertical transitions, egress-domain classification and surface-native validation.

## Limits and deployment

The local raw IFC upload limit is 256 MiB by default. The endpoint is designed for a trusted local workstation workflow; it is not intended to be exposed directly as an unauthenticated public conversion service.

Third-party human binaries remain presentation assets and are not required by the kernel. See `viewer/public/humans/README.md`.
