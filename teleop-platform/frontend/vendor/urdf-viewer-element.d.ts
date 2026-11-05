import {Object3D,Scene,PerspectiveCamera,WebGLRenderer,HemisphereLight,Mesh} from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {URDFRobot} from 'urdf-loader/src/URDFClasses';
export default class URDFViewer extends HTMLElement {
  scene:Scene;world:Object3D;camera:PerspectiveCamera;renderer:WebGLRenderer;
  controls:OrbitControls;plane:Mesh;ambientLight:HemisphereLight;
  robot:URDFRobot|null;noAutoRecenter:boolean;ignoreLimits:boolean;showCollision:boolean;urdf:string;
  setJointValues(values:Record<string,number>):void;
  redraw():void;
}
