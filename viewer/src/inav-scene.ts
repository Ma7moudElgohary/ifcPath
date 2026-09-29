import * as THREE from "three";
type Cell={vertices_m:[number,number,number][],terrain?:string};
export class InavScene{
 private group=new THREE.Group();
 constructor(private scene:THREE.Scene){scene.add(this.group)}
 load(model:{cells?:Cell[]}){this.group.clear();const positions:number[]=[];
  for(const c of model.cells??[])for(const v of c.vertices_m)positions.push(v[0],v[2],-v[1]);
  const g=new THREE.BufferGeometry();g.setAttribute("position",new THREE.Float32BufferAttribute(positions,3));
  const m=new THREE.MeshBasicMaterial({transparent:true,opacity:.32,side:THREE.DoubleSide,vertexColors:false});
  this.group.add(new THREE.Mesh(g,m));this.group.add(new THREE.LineSegments(new THREE.WireframeGeometry(g),new THREE.LineBasicMaterial()));
 }
}