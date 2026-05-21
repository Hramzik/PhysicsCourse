import { createRenderer } from './render/three_setup.js';
import { CameraControls } from './render/camera_controls.js';
import { Controls } from './ui/controls.js';
import { loadPart1Scene1 } from './scenes/part1_scene1.js';
import { loadPart2Scene1Spring } from './scenes/part2_scene1_spring.js';

const container = document.getElementById('canvas-container');
const rendererData = createRenderer(container);
const cameraControls = new CameraControls(rendererData.camera);
const controls = new Controls();

let currentScene = null;
let currentPart = 'part1';
let currentSceneKey = 'scene1';
let currentIntegrator = 'global';
let speedFactor = 1;

function loadScene(part, sceneKey, integrator){
  // 1. Dispose old scene FIRST, before any UI changes
  if(currentScene && currentScene.dispose) currentScene.dispose();
  currentScene = null;

  // 2. Update state
  currentPart = part;
  currentSceneKey = sceneKey;
  currentIntegrator = integrator;

  // 3. Sync UI silently (no onChange callback fired)
  controls.setSelection(part, sceneKey, integrator);

  // 4. Create new scene
  if(part === 'part1' && sceneKey === 'scene1'){
    currentScene = loadPart1Scene1(rendererData, integrator);
    controls.setStatsElements(currentScene.statElems);
  } else if(part === 'part2' && sceneKey === 'spring'){
    currentScene = loadPart2Scene1Spring(rendererData, integrator);
    controls.setStatsElements(currentScene.statElems);
  } else {
    // placeholder scene (not implemented)
    currentScene = null;
    controls.setStatsElements(null);
  }

  // 5. Push current UI state into the freshly created scene
  if(currentScene && currentScene.setDamping){
    currentScene.setDamping(controls.damping);
  }
}

controls.onChange((part, scene, integrator, speed, damping)=>{
  speedFactor = speed;

  if(part !== currentPart || scene !== currentSceneKey || integrator !== currentIntegrator){
    loadScene(part, scene, integrator);
  }

  if(currentScene) currentScene.setDamping(damping);
});

// Reset scene: keep current integrator, just reload the scene
controls.onReset((part, scene, integrator)=>{
  speedFactor = controls.speed;
  loadScene(part, scene, integrator);
});

// Per-part integrator maps — must be set BEFORE setParts so that
// _populateScenes() → _populateIntegrators() finds the map ready.
controls.setIntegrators({
  part1: {
    global: 'Global coords',
    localNoGyro: 'Local coords no gyro',
    localExplicitGyro: 'Local coords explicit gyro',
    localImplicitGyro: 'Local coords implicit gyro'
  },
  part2: {
    externalForce: 'External force',
    softConstraint: 'Soft constraint (Buddha)'
  },
  part3: {},
  part4: {}
});

controls.setParts({
  part1: {label:'Part 1', scenes: {scene1:'Scene 1'}},
  part2: {label:'Part 2', scenes: {spring:'Spring'}},
  part3: {label:'Part 3', scenes: {placeholder:'(not implemented)'}},
  part4: {label:'Part 4', scenes: {placeholder:'(not implemented)'}}
});

loadScene('part1', 'scene1', 'global');

function animate(t){
  requestAnimationFrame(animate);
  cameraControls.update(1/60);
  const dt = 1/60 * speedFactor;
  if(currentScene && currentScene.step) currentScene.step(dt);
  rendererData.renderer.render(rendererData.scene, rendererData.camera);
}
requestAnimationFrame(animate);
