import { createRenderer } from './render/three_setup.js';
import { CameraControls } from './render/camera_controls.js';
import { Controls } from './ui/controls.js';
import { loadPart1Variant1 } from './scenes/part1_variant1.js';

const container = document.getElementById('canvas-container');
const rendererData = createRenderer(container);
const cameraControls = new CameraControls(rendererData.camera);
const controls = new Controls();

let currentScene = null;
let currentPart = 'part1';
let currentSceneKey = 'variant1';
let currentIntegrator = 'global';
let speedFactor = 1;

function loadScene(part, sceneKey, integrator){
  if(currentScene && currentScene.dispose) currentScene.dispose();
  currentPart = part;
  currentSceneKey = sceneKey;
  currentIntegrator = integrator;
  if(part === 'part1' && sceneKey === 'variant1'){
    currentScene = loadPart1Variant1(rendererData, integrator);
    controls.setStatsElements(currentScene.statElems);
  } else {
    // placeholder scene (not implemented)
    currentScene = null;
    controls.setStatsElements(null);
  }
}

controls.onChange((part,scene,integrator,speed,damping)=>{
  speedFactor = speed;

  if(part !== currentPart || scene !== currentSceneKey || integrator !== currentIntegrator){
    loadScene(part,scene,integrator,damping);
  }

  currentScene.setDamping(damping);
});

controls.onReset((part,scene)=>{
  speedFactor = controls.speed;
  loadScene(part,scene,currentIntegrator);
});

controls.setParts({
  part1: {label:'Part 1', scenes: {variant1:'Variant 1'}},
  part2: {label:'Part 2', scenes: {placeholder:'(not implemented)'}},
  part3: {label:'Part 3', scenes: {placeholder:'(not implemented)'}},
  part4: {label:'Part 4', scenes: {placeholder:'(not implemented)'}}
});

controls.setIntegrators({
  global: 'Global coords',
  localNoGyro: 'Local coords no gyro',
  localExplicitGyro: 'Local coords explicit gyro',
  localImplicitGyro: 'Local coords implicit gyro'
});

loadScene('part1','variant1','global');

function animate(t){
  requestAnimationFrame(animate);
  cameraControls.update(1/60);
  const dt = 1/60 * speedFactor;
  if(currentScene && currentScene.step) currentScene.step(dt);
  rendererData.renderer.render(rendererData.scene, rendererData.camera);
}
requestAnimationFrame(animate);
