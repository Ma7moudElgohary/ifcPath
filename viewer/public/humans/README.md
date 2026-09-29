# Human asset slot

IfcPath does not couple navigation or simulation to one character mesh.

Recommended lightweight starter source:

- **Kenney Mini Characters** — https://kenney.nl/assets/mini-characters
- License: **CC0 / public domain** per the asset page and Kenney support/license documentation.
- The pack is animated and includes useful visual variation for evacuation demos.

Place deployment-ready `.glb` / `.gltf` files here (or serve them from your own asset CDN) and load them through `CrowdLayer.loadHumanUrl(...)`. The normal viewer also accepts a local Human GLB file.

For large studies keep skeletal animation to a small foreground cohort. The production renderer automatically represents the remaining crowd through progressively cheaper instanced geometry; microscopic positions still come from IfcPath/JuPedSim, not from animation clips.

Do not make scenario logic depend on a model filename, skeleton, material, body type, or source pack. Character assets are presentation only.
