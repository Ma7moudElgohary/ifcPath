import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { clone } from "three/examples/jsm/utils/SkeletonUtils.js";
import { AgentVisual, InstancedAgentRenderer } from "./agent-instancing";

export type CrowdRoute = THREE.Vector3[];

type DetailedAgent = {
  object: THREE.Object3D;
  mixer?: THREE.AnimationMixer;
  route: CrowdRoute;
  segment: number;
  speed: number;
};

type SimpleAgent = {
  route: CrowdRoute;
  segment: number;
  speed: number;
  position: THREE.Vector3;
  visual: AgentVisual;
};

/**
 * Route-demo crowd visualization.
 *
 * A small number of loaded GLB humans may use skeletal animation; all remaining
 * pedestrians go through the same high-count instanced LOD path as solver
 * playback. The navigation/simulation model never depends on a particular mesh.
 */
export class CrowdLayer {
  private detailed: DetailedAgent[] = [];
  private simple: SimpleAgent[] = [];
  private humanTemplate?: THREE.Group;
  private humanAnimations: THREE.AnimationClip[] = [];
  private paused = false;
  private speedMultiplier = 1;
  private readonly maxAnimatedHumans = 32;
  private readonly simpleCapacity = 20_000;
  private readonly instances: InstancedAgentRenderer;

  constructor(private scene: THREE.Scene) {
    this.instances = new InstancedAgentRenderer(scene, this.simpleCapacity, 256, 2_048);
  }

  async loadHuman(file: File) {
    const loader = new GLTFLoader();
    const url = URL.createObjectURL(file);
    try {
      const gltf = await loader.loadAsync(url);
      this.useLoadedHuman(gltf.scene, gltf.animations);
    } finally {
      URL.revokeObjectURL(url);
    }
  }

  /** Load a replaceable GLB/GLTF asset without coupling simulation to its source. */
  async loadHumanUrl(url: string) {
    const gltf = await new GLTFLoader().loadAsync(url);
    this.useLoadedHuman(gltf.scene, gltf.animations);
  }

  setPaused(paused: boolean) {
    this.paused = paused;
  }

  setSpeedMultiplier(value: number) {
    this.speedMultiplier = Math.max(0.1, Math.min(5, value || 1));
  }

  clear() {
    for (const agent of this.detailed) {
      this.scene.remove(agent.object);
      agent.mixer?.stopAllAction();
    }
    this.detailed = [];
    this.simple = [];
    this.instances.clear();
  }

  spawnRouteAgents(route: CrowdRoute, count: number) {
    this.clear();
    if (route.length < 2 || count <= 0) return;
    const total = Math.min(count, this.simpleCapacity + this.maxAnimatedHumans);
    const detailedCount = this.humanTemplate ? Math.min(total, this.maxAnimatedHumans) : 0;

    for (let i = 0; i < detailedCount; i++) {
      const object = clone(this.humanTemplate!);
      const scale = 0.92 + (i % 7) * 0.025;
      object.scale.setScalar(scale);
      object.position.copy(route[0]).add(this.lateralOffset(route, i));
      this.scene.add(object);
      let mixer: THREE.AnimationMixer | undefined;
      if (this.humanAnimations.length) {
        mixer = new THREE.AnimationMixer(object);
        mixer.clipAction(this.humanAnimations[0]).play();
      }
      this.detailed.push({ object, mixer, route, segment: 0, speed: 1.1 + (i % 5) * 0.08 });
    }

    for (let i = detailedCount; i < total; i++) {
      const position = route[0].clone().add(this.lateralOffset(route, i));
      const visual: AgentVisual = {
        position,
        yaw: 0,
        color: 0x607a8c,
        scale: 0.94 + (i % 9) * 0.012,
      };
      this.simple.push({
        route,
        segment: 0,
        speed: 1.05 + (i % 7) * 0.07,
        position,
        visual,
      });
    }
    this.flushInstances();
  }

  update(dt: number) {
    if (this.paused) return;
    const scaledDt = dt * this.speedMultiplier;
    for (const agent of this.detailed) {
      agent.mixer?.update(scaledDt);
      this.advance(agent.object.position, agent.route, agent, scaledDt);
      this.faceNext(agent.object, agent.route, agent.segment);
    }
    for (const agent of this.simple) this.advance(agent.position, agent.route, agent, scaledDt);
    this.flushInstances();
  }

  get agentCount() {
    return this.detailed.length + this.simple.length;
  }

  get lodStats() {
    return this.instances.stats;
  }

  private useLoadedHuman(scene: THREE.Group, animations: THREE.AnimationClip[]) {
    this.humanTemplate = scene;
    this.humanAnimations = animations;
    this.humanTemplate.traverse((object) => {
      if (object instanceof THREE.Mesh) {
        object.castShadow = true;
        object.receiveShadow = true;
      }
    });
  }

  private advance(
    position: THREE.Vector3,
    route: CrowdRoute,
    state: { segment: number; speed: number },
    dt: number,
  ) {
    let remaining = state.speed * dt;
    while (remaining > 0 && route.length > 1) {
      const nextIndex = (state.segment + 1) % route.length;
      const target = route[nextIndex];
      const distance = position.distanceTo(target);
      if (distance <= remaining || distance < 1e-4) {
        position.copy(target);
        remaining -= distance;
        state.segment = nextIndex;
        if (state.segment === route.length - 1) {
          state.segment = 0;
          position.copy(route[0]);
        }
      } else {
        position.lerp(target, remaining / distance);
        remaining = 0;
      }
    }
  }

  private faceNext(object: THREE.Object3D, route: CrowdRoute, segment: number) {
    if (route.length < 2) return;
    const next = route[Math.min(segment + 1, route.length - 1)];
    const dx = next.x - object.position.x;
    const dz = next.z - object.position.z;
    if (dx * dx + dz * dz > 1e-6) object.rotation.y = Math.atan2(dx, dz);
  }

  private lateralOffset(route: CrowdRoute, index: number) {
    if (route.length < 2) return new THREE.Vector3();
    const direction = route[1].clone().sub(route[0]).setY(0).normalize();
    const side = new THREE.Vector3(-direction.z, 0, direction.x);
    const lane = ((index % 9) - 4) * 0.12;
    return side.multiplyScalar(lane);
  }

  private flushInstances() {
    for (const agent of this.simple) {
      const next = agent.route[Math.min(agent.segment + 1, agent.route.length - 1)] ?? agent.position;
      const dx = next.x - agent.position.x;
      const dz = next.z - agent.position.z;
      agent.visual.yaw = dx * dx + dz * dz > 1e-6 ? Math.atan2(dx, dz) : 0;
    }
    this.instances.update(this.simple.map((agent) => agent.visual));
  }
}
