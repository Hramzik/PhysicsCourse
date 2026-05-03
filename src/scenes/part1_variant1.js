import { RigidBody3D } from '../engine/rigidbody3d.js';
import { integrateInGlobalCoords, integrateInLocalCoordsNoGyro, integrateInLocalCoordsExplicitGyro } from '../engine/integrator.js';
import { createArrow, updateArrow } from '../render/arrow_helper.js';

export function loadPart1Variant1(rendererData, integrator){
  const scene = rendererData.scene;

  const boxSize = [1,0.4,0.1];
  const body = new RigidBody3D({mass:1, size:boxSize});
  body.setAngularVelocityGlobal(new THREE.Vector3(0, 0.1, 1.6));
  const initialL = body.getAngularMomentum().clone();

  const geom = new THREE.BoxGeometry(...boxSize);
  const mat = new THREE.MeshStandardMaterial({color:0x88aaff});
  const mesh = new THREE.Mesh(geom, mat);
  scene.add(mesh);

  const initialArrow = createArrow(scene, 0xffff00, 1);
  const currentArrow = createArrow(scene, 0x00ff00, 1);
  const initialArrowOrigin = new THREE.Vector3(body.position.x, body.position.y + 0.1, body.position.z);
  updateArrow(initialArrow, initialL, initialArrowOrigin);
  updateArrow(currentArrow, body.getAngularMomentum(), body.position);

  const statElems = {
    L0: document.getElementById('L0'),
    Lcur: document.getElementById('Lcur'),
    energy: document.getElementById('energy')
  };

  const L0 = body.getAngularMomentum().length();
  statElems.L0.textContent = L0.toFixed(3);

  const integrators = {
    global: integrateInGlobalCoords,
    localNoGyro: integrateInLocalCoordsNoGyro,
    localExplicitGyro: integrateInLocalCoordsExplicitGyro
  };
  const stepIntegrator = integrators[integrator] || integrateInGlobalCoords;
  let currentDamping = 0;

  return {
    step(dt){
      stepIntegrator(body, dt, currentDamping);
      mesh.quaternion.copy(body.quaternion);
      const currentL = body.getAngularMomentum();
      updateArrow(currentArrow, currentL, body.position);
      statElems.Lcur.textContent = currentL.length().toFixed(3);
      const energy = 0.5 * body.getAngularVelocityGlobal().lengthSq() * averageInertia(body);
      statElems.energy.textContent = energy.toFixed(4);
    },
    setDamping(value){ currentDamping = value; },
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
