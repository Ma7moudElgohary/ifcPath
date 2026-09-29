import * as THREE from "three";
import * as OBC from "@thatopen/components";
import "./style.css";
import { InavScene } from "./inav-scene";
import { CrowdLayer } from "./crowd";

const host=document.querySelector<HTMLDivElement>("#app")!;
host.innerHTML=`<div id="viewport"></div><div class="hud"><b>IfcPath</b><label>IFC <input id="ifc" type="file" accept=".ifc"/></label><label>INAV <input id="inav" type="file" accept=".inav,.json"/></label><span id="status">Load IFC + INAV</span></div>`;
const viewport=document.querySelector<HTMLDivElement>("#viewport")!;
const status=document.querySelector<HTMLSpanElement>("#status")!;

const components=new OBC.Components();
const worlds=components.get(OBC.Worlds);
const world=worlds.create<OBC.SimpleScene,OBC.OrthoPerspectiveCamera,OBC.SimpleRenderer>();
world.scene=new OBC.SimpleScene(components); world.scene.setup();
world.renderer=new OBC.SimpleRenderer(components,viewport);
world.camera=new OBC.OrthoPerspectiveCamera(components);
world.camera.controls.setLookAt(12,12,12,0,0,0);
components.init();

const fragments=components.get(OBC.FragmentsManager);
fragments.init(await OBC.FragmentsManager.getWorker());
world.camera.controls.addEventListener("update",()=>fragments.core.update());
fragments.list.onItemSet.add(({value:model})=>{
 model.useCamera(world.camera.three);
 world.scene.three.add(model.object);
 fragments.core.update(true);
});
const ifcLoader=components.get(OBC.IfcLoader);
await ifcLoader.setup({autoSetWasm:true});

const nav=new InavScene(world.scene.three);
const crowd=new CrowdLayer(world.scene.three);

document.querySelector<HTMLInputElement>("#ifc")!.onchange=async e=>{
 const file=(e.target as HTMLInputElement).files?.[0]; if(!file)return;
 status.textContent="Importing IFC…";
 const bytes=new Uint8Array(await file.arrayBuffer());
 await ifcLoader.load(bytes,true,file.name.replace(/\.ifc$/i,""));
 status.textContent="IFC loaded";
};
document.querySelector<HTMLInputElement>("#inav")!.onchange=async e=>{
 const file=(e.target as HTMLInputElement).files?.[0]; if(!file)return;
 const model=JSON.parse(await file.text()); nav.load(model);
 status.textContent=`${model.cells?.length??0} navigation cells`;
};
let last=performance.now();
world.renderer.onBeforeUpdate.add(()=>{const now=performance.now();crowd.update(Math.min((now-last)/1000,.05));last=now;});
