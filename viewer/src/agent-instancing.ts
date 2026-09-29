import * as THREE from "three";

export type AgentVisual = {
  position: THREE.Vector3;
  yaw: number;
  color?: number;
  scale?: number;
};

export type AgentLodStats = {
  total: number;
  near: number;
  mid: number;
  far: number;
};

/**
 * Shared high-count pedestrian renderer.
 *
 * Agents are split into progressively cheaper meshes so route demos and solver
 * playback use the same rendering path. The split is deterministic and avoids
 * one Object3D per pedestrian for the large majority of a crowd.
 */
export class InstancedAgentRenderer {
  private readonly near: THREE.InstancedMesh;
  private readonly mid: THREE.InstancedMesh;
  private readonly far: THREE.InstancedMesh;
  private readonly matrix = new THREE.Matrix4();
  private readonly quaternion = new THREE.Quaternion();
  private readonly scale = new THREE.Vector3();
  private readonly axis = new THREE.Vector3(0, 1, 0);
  private readonly color = new THREE.Color();
  private statsValue: AgentLodStats = { total: 0, near: 0, mid: 0, far: 0 };

  readonly maxAgents: number;
  readonly nearBudget: number;
  readonly midBudget: number;

  constructor(
    private readonly scene: THREE.Scene,
    maxAgents = 20_000,
    nearBudget = 256,
    midBudget = 2_048,
  ) {
    this.maxAgents = Math.max(1, maxAgents);
    this.nearBudget = Math.min(this.maxAgents, Math.max(0, nearBudget));
    this.midBudget = Math.min(
      this.maxAgents - this.nearBudget,
      Math.max(0, midBudget),
    );

    const material = () => new THREE.MeshStandardMaterial({ roughness: 0.9, vertexColors: true });
    this.near = new THREE.InstancedMesh(
      new THREE.CapsuleGeometry(0.20, 0.90, 3, 6),
      material(),
      Math.max(1, this.nearBudget),
    );
    this.mid = new THREE.InstancedMesh(
      new THREE.CylinderGeometry(0.19, 0.22, 1.28, 5, 1),
      material(),
      Math.max(1, this.midBudget),
    );
    this.far = new THREE.InstancedMesh(
      new THREE.ConeGeometry(0.24, 1.25, 4, 1),
      material(),
      Math.max(1, this.maxAgents - this.nearBudget - this.midBudget),
    );

    for (const [name, mesh] of [
      ["near", this.near],
      ["mid", this.mid],
      ["far", this.far],
    ] as const) {
      mesh.name = `IfcPath crowd ${name} LOD`;
      mesh.count = 0;
      // Computing one aggregate bounding volume every frame is more expensive
      // than drawing these compact meshes. Let the BIM renderer own model LOD;
      // crowd LOD is explicit here.
      mesh.frustumCulled = false;
      mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      scene.add(mesh);
    }
  }

  update(agents: readonly AgentVisual[]) {
    const total = Math.min(this.maxAgents, agents.length);
    const nearCount = Math.min(total, this.nearBudget);
    const midCount = Math.min(total - nearCount, this.midBudget);
    const farCount = total - nearCount - midCount;

    this.near.count = nearCount;
    this.mid.count = midCount;
    this.far.count = farCount;

    this.writeRange(this.near, agents, 0, nearCount);
    this.writeRange(this.mid, agents, nearCount, midCount);
    this.writeRange(this.far, agents, nearCount + midCount, farCount);

    this.statsValue = { total, near: nearCount, mid: midCount, far: farCount };
  }

  clear() {
    this.near.count = 0;
    this.mid.count = 0;
    this.far.count = 0;
    this.near.instanceMatrix.needsUpdate = true;
    this.mid.instanceMatrix.needsUpdate = true;
    this.far.instanceMatrix.needsUpdate = true;
    this.statsValue = { total: 0, near: 0, mid: 0, far: 0 };
  }

  setVisible(visible: boolean) {
    this.near.visible = visible;
    this.mid.visible = visible;
    this.far.visible = visible;
  }

  get stats(): AgentLodStats {
    return this.statsValue;
  }

  dispose() {
    for (const mesh of [this.near, this.mid, this.far]) {
      this.scene.remove(mesh);
      mesh.geometry.dispose();
      const material = mesh.material;
      if (Array.isArray(material)) material.forEach((item) => item.dispose());
      else material.dispose();
    }
  }

  private writeRange(
    mesh: THREE.InstancedMesh,
    agents: readonly AgentVisual[],
    offset: number,
    count: number,
  ) {
    for (let index = 0; index < count; index++) {
      const agent = agents[offset + index];
      this.quaternion.setFromAxisAngle(this.axis, agent.yaw);
      const scalar = agent.scale ?? 1;
      this.scale.set(scalar, scalar, scalar);
      this.matrix.compose(agent.position, this.quaternion, this.scale);
      mesh.setMatrixAt(index, this.matrix);
      this.color.setHex(agent.color ?? 0x607080);
      mesh.setColorAt(index, this.color);
    }
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
  }
}
