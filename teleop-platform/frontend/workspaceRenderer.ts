import * as THREE from 'three';
import URDFViewer from './vendor/urdf-viewer-element.js';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';

export type GripperPose={valid:boolean;joints_rad:Record<string,number>|null;fallback:boolean;received_ns?:number;raw?:number|null};
export type WorkspaceScene = {setJoints:(q:number[])=>void;setGripper?:(pose:GripperPose)=>void;layers:(points:boolean,collision:boolean)=>void;reset:()=>void;dispose:()=>void};

// Complete upstream viewer: scene, rendering, URDF loading and OrbitControls.
// This adapter adds project data, never movement commands.
export async function createWorkspaceScene(host:HTMLDivElement,robotUrl:string,mapUrl:string,modelId:string,onError:(message:string)=>void,fingerUnionUrl?:string):Promise<WorkspaceScene> {
  if(!customElements.get('e6-urdf-viewer'))customElements.define('e6-urdf-viewer',URDFViewer);
  const viewer=document.createElement('e6-urdf-viewer') as URDFViewer;
  viewer.style.cssText='width:100%;height:100%;outline:none';
  viewer.noAutoRecenter=true;viewer.ignoreLimits=true; // Show actual feedback, even out-of-limit readings.
  viewer.camera.near=.001;viewer.camera.far=10;viewer.camera.fov=40;
  viewer.controls.rotateSpeed=1;viewer.controls.zoomSpeed=1;viewer.controls.panSpeed=1;
  viewer.controls.minDistance=.1;viewer.controls.maxDistance=3;
  viewer.plane.visible=false;viewer.ambientLight.intensity=2;
  // No environment plane: disable the upstream scene's large-world shadows,
  // whose default scale causes self-shadow acne on this meter-scale arm.
  viewer.renderer.shadowMap.enabled=false;
  viewer.world.add(new THREE.AxesHelper(.15));
  const reset=()=>{viewer.camera.position.set(.85,.8,1.1);viewer.controls.target.set(0,.23,0);viewer.controls.update();viewer.redraw();};
  let disposed=false,joints:number[]=[];let cloud:THREE.Points|undefined;
  let gripper:GripperPose={valid:false,joints_rad:null,fallback:true};
  let unionObject:THREE.Mesh|undefined;
  let showPoints=true,showCollision=false;
  const abort=new AbortController();
  // Same URDF collision shapes as FCL. Distinguish rigid geometry from the
  // nominal full-stroke finger boxes; box edges make overlap inspectable.
  const rigidMaterial=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:.10,depthWrite:false});
  const sweepMaterial=new THREE.MeshBasicMaterial({color:0xfbbf24,transparent:true,opacity:.13,depthWrite:false});
  const rigidEdges=new THREE.LineBasicMaterial({color:0x38bdf8,transparent:true,opacity:.85});
  const sweepEdges=new THREE.LineBasicMaterial({color:0xfbbf24,transparent:true,opacity:.9});
  const outlined=new WeakSet<THREE.Object3D>();
  const styleCollisions=()=>{
    viewer.robot?.traverse((node:THREE.Object3D)=>{
      if(!(node as THREE.Object3D&{isURDFCollider?:boolean}).isURDFCollider)return;
      const finger=node.name.startsWith('finger_');
      if(viewer.robot?.links.op_unknown_sweep&&finger){
        let parent:THREE.Object3D|null=node;let isFallback=false;
        while(parent){if(parent.name==='op_unknown_sweep')isFallback=true;parent=parent.parent;}
        node.visible=showCollision&&(isFallback?gripper.fallback:!gripper.fallback);
      }
      // Exact union asset has the same occupied volume; hide only the repeated
      // surfaces/internal edges of its constituent boxes after it is loaded.
      if(finger&&unionObject){node.visible=false;return;}
      node.traverse(child=>{
        if(!(child instanceof THREE.Mesh))return;
        child.material=finger?sweepMaterial:rigidMaterial;
        if(child.geometry.type==='BoxGeometry'&&!outlined.has(child)){
          child.add(new THREE.LineSegments(new THREE.EdgesGeometry(child.geometry),finger?sweepEdges:rigidEdges));
          outlined.add(child);
        }
      });
    });
  };
  const loaded=()=>{
    if(disposed)return;
    if(!viewer.robot){onError('Robot geometry unavailable');return;}
    viewer.robot.visible=joints.length===6;
    viewer.robot.links.grasp_tcp?.add(new THREE.AxesHelper(.05));
    viewer.setJointValues(Object.fromEntries(joints.map((v,i)=>[`joint${i+1}`,v])));
    if(gripper.valid&&gripper.joints_rad)viewer.setJointValues(gripper.joints_rad);
    viewer.showCollision=showCollision;styleCollisions();viewer.redraw();
    if(fingerUnionUrl&&!unionObject){
      new STLLoader().load(fingerUnionUrl,geometry=>{
        if(disposed){geometry.dispose();return;}
        const parent=viewer.robot?.links.omnipicker;
        if(!parent){geometry.dispose();return;}
        unionObject=new THREE.Mesh(geometry,sweepMaterial);
        unionObject.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry,1),sweepEdges));
        parent.add(unionObject);unionObject.visible=showCollision;styleCollisions();viewer.redraw();
      },undefined,()=>{/* Original URDF boxes remain visible on overlay failure. */});
    }
  };
  const failed=(event:Event)=>{if(!disposed)onError('Robot model unavailable: '+String((event as CustomEvent).detail));};
  viewer.addEventListener('geometry-loaded',loaded);viewer.addEventListener('urdf-error',failed);
  host.appendChild(viewer);reset();viewer.urdf=robotUrl;
  // This endpoint changes across model releases. Never reuse a previous map
  // from the browser cache while pairing it with fresh model metadata.
  fetch(mapUrl,{signal:abort.signal,cache:'no-store'}).then(async response=>{
    if(!response.ok)throw new Error('Reachability sampling unavailable');return response.json();
  }).then(data=>{
    if(disposed)return;
    if(data.model_id!==modelId)throw new Error('Reachability model mismatch');
    const points=data.points_m as number[][];
    if(!Array.isArray(points)||!points.every(p=>p.length===3&&p.every(Number.isFinite)))throw new Error('Invalid workspace map');
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(points.flat(),3));
    cloud=new THREE.Points(geometry,new THREE.PointsMaterial({color:0x22c55e,size:.005,transparent:true,opacity:.17,depthWrite:false}));
    cloud.visible=showPoints;viewer.world.add(cloud);viewer.redraw();
  }).catch(error=>{if(!disposed)onError(String(error));});
  return {
    setJoints:q=>{if(q.length!==6||!q.every(Number.isFinite))return;joints=q;if(viewer.robot)viewer.robot.visible=true;viewer.setJointValues(Object.fromEntries(q.map((v,i)=>[`joint${i+1}`,v])));viewer.redraw();},
    setGripper:pose=>{gripper=pose;if(pose.valid&&pose.joints_rad&&Object.entries(pose.joints_rad).every(([name,value])=>name.startsWith('op_')&&Number.isFinite(value)))viewer.setJointValues(pose.joints_rad);styleCollisions();viewer.redraw();},
    layers:(points,collision)=>{showPoints=points;showCollision=collision;if(cloud)cloud.visible=points;viewer.showCollision=collision;if(unionObject)unionObject.visible=collision;styleCollisions();viewer.redraw();},reset,
    dispose:()=>{if(disposed)return;disposed=true;abort.abort();viewer.removeEventListener('geometry-loaded',loaded);viewer.removeEventListener('urdf-error',failed);viewer.remove();[rigidMaterial,sweepMaterial,rigidEdges,sweepEdges].forEach(m=>m.dispose());}
  };
}
