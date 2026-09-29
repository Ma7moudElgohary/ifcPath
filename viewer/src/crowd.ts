import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { clone } from "three/examples/jsm/utils/SkeletonUtils.js";

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
};

export class CrowdLayer {
  private detailed: DetailedAgent[] = [];
  private simple: SimpleAgent[] = [];
  private humanTemplate?: THREE.Group;
  private humanAnimations: THREE.AnimationClip[] = [];
  private readonly maxAnimatedHumans = 64;
  private readonly simpleCapacity = 2000;
  private readonly simpleMesh: THREE.InstancedMesh;
  private readonly matrix = new THREE.Matrix4();

  constructor(private scene: THREE.Scene) {
    this.simpleMesh = new THREE.InstancedMesh(
      new THREE.CapsuleGeometry(0.20, 0.90, 3, 6),
      new THREE.MeshStandardMaterial({ roughness: 0.9 }),
      this.simpleCapacity,
    );
    this.simpleMesh.count = 0;
    this.simpleMesh.name = "IfcPath instanced crowd";
    this.scene.add(this.simpleMesh);
  }

  async loadHuman(file: File) {
    const loader = new GLTFLoader();
    const url = URL.createObjectURL(file);
    try {
      const gltf = await loader.loadAsync(url);
      this.humanTemplate = gltf.scene;
      this.humanAnimations = gltf.animations;
      this.humanTemplate.traverse((object) => {
        if (object instanceof THREE.Mesh) {
          object.castShadow = true;
          object.receiveShadow = true;
        }
      });
    } finally {
      URL.revokeObjectURL(url);
    }
  }

  clear() {
    for (const agent of this.detailed) {
      this.scene.remove(agent.object);
      agent.mixer?.stopAllAction();
    }
    this.detailed = [];
    this.simple = [];
    this.simpleMesh.count = 0;
    this.simpleMesh.instanceMatrix.needsUpdate = true;
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
      this.simple.push({
        route,
        segment: 0,
        speed: 1.05 + (i % 7) * 0.07,
        position: route[0].clone().add(this.lateralOffset(route, i)),
      });
    }
    this.simpleMesh.count = this.simple.length;
    this.flushInstances();
  }

  update(dt: number) {
    for (const agent of this.detailed) {
      agent.mixer?.update(dt);
      this.advance(agent.object.position, agent.route, agent, dt);
      this.faceNext(agent.object, agent.route, agent.segment);
    }
    for (const agent of this.simple) this.advance(agent.position, agent.route, agent, dt);
    this.flushInstances();
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
    const direction = next.clone().sub(object.position);
    direction.y = 0;
    if (direction.lengthSq() > 1e-6) object.rotation.y = Math.atan2(direction.x, direction.z);
  }

  private lateralOffset(route: CrowdRoute, index: number) {
    if (route.length < 2) return new THREE.Vector3();
    const direction = route[1].clone().sub(route[0]).setY(0).normalize();
    const side = new THREE.Vector3(-direction.z, 0, direction.x);
    const lane = ((index % 9) - 4) * 0.12;
    return side.multiplyScalar(lane);
  }

  private flushInstances() {
    for (let i = 0; i < this.simple.length; i++) {
      const agent = this.simple[i];
      const next = agent.route[Math.min(agent.segment + 1, agent.route.length - 1)] ?? agent.position;
      const direction = next.clone().sub(agent.position).setY(0);
      const rotation = new THREE.Quaternion();
      if (direction.lengthSq() > 1e-6) rotation.setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.atan2(direction.x, direction.z));
      this.matrix.compose(agent.position, rotation, new THREE.Vector3(1, 1, 1));
      this.simpleMesh.setMatrixAt(i, this.matrix);
    }
    this.simpleMesh.instanceMatrix.needsUpdate = true;
  }
}
