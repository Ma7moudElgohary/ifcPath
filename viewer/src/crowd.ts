import * as THREE from "three";
export class CrowdLayer{
 private agents:THREE.Object3D[]=[];
 constructor(private scene:THREE.Scene){}
 /** Placeholder primitive until a CC0 GLB is supplied. The simulation API never depends on a specific human mesh. */
 spawn(position:THREE.Vector3){const mesh=new THREE.Mesh(new THREE.CapsuleGeometry(.22,.9,4,8),new THREE.MeshStandardMaterial());mesh.position.copy(position);this.scene.add(mesh);this.agents.push(mesh)}
 update(_dt:number){}
}