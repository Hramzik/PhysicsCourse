import { RigidBody3D } from '../engine/rigidbody3d.js';
import { integrateInGlobalCoords } from '../engine/integrator.js';
import { createArrow, updateArrow } from '../render/arrow_helper.js';

export function loadPart1Variant1(rendererData){
  const scene = rendererData.scene;

  const boxSize = [1,0.4,0.1];
  const body = new RigidBody3D({mass:1, size:boxSize, angularVelocity:[1.6, 0, 0]});
  const initialL = body.angularMomentum().clone();

  const geom = new THREE.BoxGeometry(...boxSize);
  const mat = new THREE.MeshStandardMaterial({color:0x88aaff});
  const mesh = new THREE.Mesh(geom, mat);
  scene.add(mesh);

  const initialArrow = createArrow(scene, 0xffff00, 1);
  const currentArrow = createArrow(scene, 0x00ff00, 1);
  const initialArrowOrigin = new THREE.Vector3(body.position.x, body.position.y + 0.1, body.position.z);
  updateArrow(initialArrow, initialL, initialArrowOrigin);
  updateArrow(currentArrow, body.angularMomentum(), body.position);

  const statElems = {
    L0: document.getElementById('L0'),
    Lcur: document.getElementById('Lcur'),
    energy: document.getElementById('energy')
  };

  const L0 = body.angularMomentum().length();
  statElems.L0.textContent = L0.toFixed(3);

  return {
    step(dt){
      integrateInGlobalCoords(body, dt);
      mesh.quaternion.copy(body.quaternion);
      const currentL = body.angularMomentum();
      updateArrow(currentArrow, currentL, body.position);
      statElems.Lcur.textContent = currentL.length().toFixed(3);
      const energy = 0.5 * body.angularVelocity.lengthSq() * averageInertia(body);
      statElems.energy.textContent = energy.toFixed(4);
    },
    statElems,
    dispose(){
      try{ scene.remove(mesh); }catch(e){}
      try{ geom.dispose(); }catch(e){}
      try{ mat.dispose(); }catch(e){}
      try{ scene.remove(initialArrow); }catch(e){}
      try{ scene.remove(currentArrow); }catch(e){}
    }
  };
}

function averageInertia(body){
  const e = body.inertiaLocal.elements;
  return (e[0]+e[4]+e[8])/3;
}
