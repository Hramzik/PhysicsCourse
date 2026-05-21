import { createRenderer } from './render/three_setup.js';
import { CameraControls } from './render/camera_controls.js';
import { Controls } from './ui/controls.js';
import { loadPart1Scene1 } from './scenes/part1_scene1.js';
import { loadPart2Scene1Spring } from './scenes/part2_scene1_spring.js';
import { loadPart2Scene2TwoBodies } from './scenes/part2_scene2_two_bodies.js';
import { loadPart3Scene1Boxes } from './scenes/part3_scene1_boxes.js';
import { loadPart3Scene2Many } from './scenes/part3_scene2_many.js';
import { loadPart4Scene1Many } from './scenes/part4_scene1_many.js';

const container = document.getElementById('canvas-container');
const rendererData = createRenderer(container);
const cameraControls = new CameraControls(rendererData.camera);
const controls = new Controls();

let currentScene = null;
let currentPart = 'part1';
let currentSceneKey = 'scene1';
let currentIntegrator = 'global';
let currentMethod = null;
let speedFactor = 1;

// Default method per (part, scene)
const DEFAULT_METHODS = {
  part2: { twoBodies: 'xpbd' },
  part3: { boxes: 'xpbd', many: 'xpbd' },
  part4: { many: 'sap' }
};

// Default damping per (part, scene, method). When a scene is loaded with a
// specific method that has an entry here, the global damping slider is
// programmatically set to that value (UI + internal state).
const METHOD_DEFAULT_DAMPING = {
  part2: {
    twoBodies: {
      xpbd: 0,
      si_baumgarte: 0.9,
      si_ngs: 0,
      si_soft: 0.3
    }
  },
  part3: {
    boxes: { xpbd: 0.05, si: 0.05 },
    many:  { xpbd: 0.05 }
  },
  part4: {
    many: { sap: 0.05, lbvh: 0.05 }
  }
};

function getDefaultMethod(part, sceneKey){
  return (DEFAULT_METHODS[part] && DEFAULT_METHODS[part][sceneKey]) || null;
}

function getMethodDefaultDamping(part, sceneKey, method){
  if(!method) return null;
  const partMap = METHOD_DEFAULT_DAMPING[part];
  if(!partMap) return null;
  const sceneMap = partMap[sceneKey];
  if(!sceneMap) return null;
  return (method in sceneMap) ? sceneMap[method] : null;
}

function loadScene(part, sceneKey, integrator, method){
  // 1. Dispose old scene FIRST
  if(currentScene && currentScene.dispose) currentScene.dispose();
  currentScene = null;

  // 2. Determine method (use default if not given and scene has methods)
  const defaultMethod = getDefaultMethod(part, sceneKey);
  if(method == null) method = defaultMethod;

  // 3. Apply per-method default damping (silently updates slider + state)
  const methodDamping = getMethodDefaultDamping(part, sceneKey, method);
  if(methodDamping != null){
    controls.setDamping(methodDamping);
  }

  // 4. Update state
  currentPart = part;
  currentSceneKey = sceneKey;
  currentIntegrator = integrator;
  currentMethod = method;

  // 5. Sync UI silently (no onChange callback fired)
  controls.setSelection(part, sceneKey, integrator, method);

  // 5. Create new scene
  if(part === 'part1' && sceneKey === 'scene1'){
    currentScene = loadPart1Scene1(rendererData, integrator);
    controls.setStatsElements(currentScene.statElems);
  } else if(part === 'part2' && sceneKey === 'spring'){
    currentScene = loadPart2Scene1Spring(rendererData, integrator);
    controls.setStatsElements(currentScene.statElems);
  } else if(part === 'part2' && sceneKey === 'twoBodies'){
    currentScene = loadPart2Scene2TwoBodies(rendererData, method);
    controls.setStatsElements(currentScene.statElems);
  } else if(part === 'part3' && sceneKey === 'boxes'){
    currentScene = loadPart3Scene1Boxes(rendererData, method);
    controls.setStatsElements(currentScene.statElems);
  } else if(part === 'part3' && sceneKey === 'many'){
    currentScene = loadPart3Scene2Many(rendererData, method);
    controls.setStatsElements(currentScene.statElems);
  } else if(part === 'part4' && sceneKey === 'many'){
    currentScene = loadPart4Scene1Many(rendererData, method);
    controls.setStatsElements(currentScene.statElems);
  } else {
    currentScene = null;
    controls.setStatsElements(null);
  }

  // 6. Push current UI state into the freshly created scene
  if(currentScene && currentScene.setDamping){
    currentScene.setDamping(controls.damping);
  }
  if(currentScene && currentScene.setFriction){
    currentScene.setFriction(controls.friction);
  }
}

controls.onChange((part, scene, integrator, speed, damping)=>{
  speedFactor = speed;

  if(part !== currentPart || scene !== currentSceneKey || integrator !== currentIntegrator){
    // Scene/part/integrator changed → reload. Use the method appropriate to
    // the new (part, scene). If we're staying in the same scene, keep current
    // method; otherwise pick the default.
    const sameScene = (part === currentPart && scene === currentSceneKey);
    const method = sameScene ? currentMethod : getDefaultMethod(part, scene);
    loadScene(part, scene, integrator, method);
  }

  if(currentScene) currentScene.setDamping(damping);
});

controls.onMethodChange((part, scene, method)=>{
  // Reload current scene with new method
  loadScene(part, scene, currentIntegrator, method);
});

controls.onFrictionChange((value)=>{
  if(currentScene && currentScene.setFriction) currentScene.setFriction(value);
});

// Reset scene: keep current integrator and method
controls.onReset((part, scene, integrator, method)=>{
  speedFactor = controls.speed;
  loadScene(part, scene, integrator, method);
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

// Per-(part, scene) method maps. When a scene has methods defined,
// the method selector is shown and the integrator selector is hidden.
controls.setMethods({
  part2: {
    twoBodies: {
      xpbd: 'XPBD',
      si_baumgarte: 'SI + Baumgarte',
      si_ngs: 'SI + NGS',
      si_soft: 'SI + Soft (Buddha)'
    }
  },
  part3: {
    boxes: {
      xpbd: 'XPBD',
      si: 'SI + NGS'
    },
    many: {
      xpbd: 'XPBD'
    }
  },
  part4: {
    many: {
      sap:  'Sweep and Prune',
      lbvh: 'LBVH'
    }
  }
});

controls.setParts({
  part1: {label:'Part 1', scenes: {scene1:'Scene 1'}},
  part2: {label:'Part 2', scenes: {spring:'Spring', twoBodies:'Two bodies (distance)'}},
  part3: {label:'Part 3', scenes: {boxes:'10 boxes (gravity)', many:'~1000 cubes (spatial grid)'}},
  part4: {label:'Part 4', scenes: {many:'~400 varied boxes (SAP vs LBVH)'}}
});

loadScene('part1', 'scene1', 'global', null);

function animate(t){
  requestAnimationFrame(animate);
  cameraControls.update(1/60);
  const dt = 1/60 * speedFactor;
  if(currentScene && currentScene.step) currentScene.step(dt);
  rendererData.renderer.render(rendererData.scene, rendererData.camera);
}
requestAnimationFrame(animate);
