import { createRenderer } from './render/three_setup.js';
import { Controls } from './ui/controls.js';
import { loadPart1Variant1 } from './scenes/part1_variant1.js';

const container = document.getElementById('canvas-container');
const rendererData = createRenderer(container);
const controls = new Controls();

let currentScene = null;

function loadScene(part, sceneKey){
  if(currentScene && currentScene.dispose) currentScene.dispose();
  if(part === 'part1' && sceneKey === 'variant1'){
    currentScene = loadPart1Variant1(rendererData);
    controls.setStatsElements(currentScene.statElems);
  } else {
    // placeholder scene (not implemented)
    currentScene = null;
    controls.setStatsElements(null);
  }
}

controls.onChange((part,scene)=>{
  loadScene(part,scene);
});

controls.setParts({
  part1: {label:'Part 1', scenes: {variant1:'Variant 1'}},
  part2: {label:'Part 2', scenes: {placeholder:'(not implemented)'}},
  part3: {label:'Part 3', scenes: {placeholder:'(not implemented)'}},
  part4: {label:'Part 4', scenes: {placeholder:'(not implemented)'}}
});

loadScene('part1','variant1');

function animate(t){
  requestAnimationFrame(animate);
  const dt = Math.min(1/30, 0.016); // fixed-ish step for demo
  if(currentScene && currentScene.step) currentScene.step(dt);
  rendererData.renderer.render(rendererData.scene, rendererData.camera);
}
requestAnimationFrame(animate);
