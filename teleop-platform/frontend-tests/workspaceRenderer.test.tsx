import {afterEach,beforeEach,describe,expect,it,vi} from 'vitest';
import * as THREE from 'three';
const loading=vi.hoisted(()=>({load:vi.fn()}));
vi.mock('three/examples/jsm/loaders/STLLoader.js',()=>({STLLoader:class {load=loading.load;}}));
vi.mock('../frontend/vendor/urdf-viewer-element.js',()=>({default:class extends HTMLElement {
  world=new THREE.Group();robot:any=new THREE.Group();camera:any={};plane:any={};ambientLight:any={};
  controls:any={target:{set:vi.fn()},update:vi.fn()};renderer:any={shadowMap:{}};
  constructor(){super();this.camera.position={set:vi.fn()};
    const link=new THREE.Group(),collider:any=new THREE.Group();collider.isURDFCollider=true;collider.name='finger_test';
    collider.add(new THREE.Mesh(new THREE.BoxGeometry()));link.add(collider);this.robot.add(link);
    this.robot.links={omnipicker:link};this.world.add(this.robot);
  }
  set showCollision(value:boolean){this.robot.traverse((n:any)=>{if(n.isURDFCollider)n.visible=value;});}
  setJointValues=vi.fn(); redraw(){}
  disconnectedCallback(){this.world.traverse((n:any)=>{n.geometry?.dispose();});}
}}));
import {createWorkspaceScene} from '../frontend/workspaceRenderer';
let host:HTMLDivElement;
beforeEach(()=>{loading.load.mockReset();host=document.createElement('div');document.body.append(host);
  vi.stubGlobal('fetch',vi.fn(async()=>({ok:true,json:async()=>({model_id:'m',points_m:[]})})));});
afterEach(()=>{host.remove();vi.unstubAllGlobals();});
async function ready(){
  const scene=await createWorkspaceScene(host,'robot.urdf','map.json','m',vi.fn(),'union.stl');
  scene.setJoints([0,0,0,0,0,0]);scene.layers(false,true);
  const viewer:any=host.firstElementChild;viewer.dispatchEvent(new Event('geometry-loaded'));
  return {scene,viewer,link:viewer.robot.links.omnipicker,collider:viewer.robot.links.omnipicker.children[0]};
}
describe('collision union rendering lifecycle',()=>{
  it('binds local moving boxes to measured pose and swaps to fallback without moving stale fingers',async()=>{
    const {scene,viewer}=await ready();
    const links:any={};
    for(const name of ['op_hand_narrow1_Link','op_unknown_sweep']){
      const link=new THREE.Group();link.name=name;
      const box:any=new THREE.Group();box.name='finger_'+name;box.isURDFCollider=true;
      box.add(new THREE.Mesh(new THREE.BoxGeometry()));link.add(box);viewer.robot.add(link);links[name]=link;
    }
    Object.assign(viewer.robot.links,links);
    const measured={op_hand_narrow1_joint:.2,op_hand_narrow2_joint:-.1};
    scene.setGripper!({valid:true,fallback:false,joints_rad:measured});
    expect(viewer.setJointValues).toHaveBeenLastCalledWith(measured);
    expect(links.op_hand_narrow1_Link.children[0].visible).toBe(true);
    expect(links.op_unknown_sweep.children[0].visible).toBe(false);
    viewer.setJointValues.mockClear();scene.setGripper!({valid:false,fallback:true,joints_rad:null});
    expect(viewer.setJointValues).not.toHaveBeenCalled();
    expect(links.op_hand_narrow1_Link.children[0].visible).toBe(false);
    expect(links.op_unknown_sweep.children[0].visible).toBe(true);
    scene.layers(false,false);expect(links.op_unknown_sweep.children[0].visible).toBe(false);
    scene.dispose();
  });
  it('keeps URDF bounds visible if the optional union fails',async()=>{
    const {scene,link,collider}=await ready();loading.load.mock.calls[0][3]();
    expect(fetch).toHaveBeenCalledWith('map.json',expect.objectContaining({cache:'no-store'}));
    expect(link.children).toHaveLength(1);expect(collider.visible).toBe(true);
    scene.layers(false,false);expect(collider.visible).toBe(false);
    scene.layers(false,true);expect(collider.visible).toBe(true);scene.dispose();
  });
  it('replaces repeated surfaces only after success and obeys layer switches',async()=>{
    const {scene,link,collider}=await ready();const geometry=new THREE.BoxGeometry();const dispose=vi.spyOn(geometry,'dispose');
    loading.load.mock.calls[0][1](geometry);const union=link.children[1];
    expect(collider.visible).toBe(false);expect(union.visible).toBe(true);
    scene.layers(false,false);expect(union.visible).toBe(false);
    scene.layers(false,true);expect(union.visible).toBe(true);expect(collider.visible).toBe(false);
    scene.dispose();expect(dispose).toHaveBeenCalled();expect(host.childElementCount).toBe(0);
  });
  it('releases an in-flight mesh after unmount without attaching it',async()=>{
    const {scene,link}=await ready();scene.dispose();const geometry=new THREE.BoxGeometry();const dispose=vi.spyOn(geometry,'dispose');
    loading.load.mock.calls[0][1](geometry);expect(dispose).toHaveBeenCalledTimes(1);expect(link.children).toHaveLength(1);
  });
});
