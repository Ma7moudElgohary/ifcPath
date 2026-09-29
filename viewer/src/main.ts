import * as THREE from "three";
import "./style.css";
import { InavScene } from "./inav-scene";
import { CrowdLayer } from "./crowd";

const host=document.querySelector<HTMLDivElement>("#app")!;
host.innerHTML=`<canvas id="view"></canvas><div class="hud"><b>IfcPath</b><input id="inav" type="file" accept=".inav,.json"/><input id="ifc" type="file" accept=".ifc"/><span id="status">Load an INAV model</span></div>`;
const canvas=document.querySelector<HTMLCanvasElement>("#view")!;
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
const scene=new THREE.Scene(); scene.background=new THREE.Color(0xf2f3f5);
const camera=new THREE.PerspectiveCamera(55,1,0.01,5000); camera.position.set(12,-18,14);
scene.add(new THREE.HemisphereLight(0xffffff,0x555555,2));
const nav=new InavScene(scene); const crowd=new CrowdLayer(scene);
const status=document.querySelector<HTMLSpanElement>("#status")!;
document.querySelector<HTMLInputElement>("#inav")!.onchange=async e=>{
 const file=(e.target as HTMLInputElement).files?.[0]; if(!file)return;
 const model=JSON.parse(await file.text()); nav.load(model); status.textContent=`${model.cells?.length??0} nav cells`;
};
let last=performance.now();
function frame(now:number){const dt=Math.min((now-last)/1000,.05);last=now;crowd.update(dt);renderer.setSize(innerWidth,innerHeight,false);camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.render(scene,camera);requestAnimationFrame(frame)}
requestAnimationFrame(frame);
