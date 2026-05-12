import { RigidBody3D } from '../engine/rigidbody3d.js';
import { integrateRigidBodyWithForcesImplicitGyro } from '../engine/integrator.js';
import { multiplyMatrix3Scalar } from '../engine/utils.js';

export function loadPart2Scene1SpringForce(rendererData){
  const scene = rendererData.scene;

  const boxSize = [1, 0.4, 0.2];
  const body = new RigidBody3D({mass:1, size:boxSize, position:[0, 0.2, 0]});
  // body.inertiaLocal = multiplyMatrix3Scalar(body.inertiaLocal, 100);
  // body.setAngularVelocityGlobal(new THREE.Vector3(0, 0.4, 0.1));
  // body.linearVelocity.set(0.6, 0, 0.2);

  const geom = new THREE.BoxGeometry(...boxSize);
  const mat = new THREE.MeshStandardMaterial({color:0x88ffaa});
  const mesh = new THREE.Mesh(geom, mat);
  scene.add(mesh);

  const anchorWorld = new THREE.Vector3(0, 1.6, 0);
  const anchorGeom = new THREE.SphereGeometry(0.05, 16, 12);
  const anchorMat = new THREE.MeshStandardMaterial({color:0xffaa33});
  const anchorMesh = new THREE.Mesh(anchorGeom, anchorMat);
  anchorMesh.position.copy(anchorWorld);
  scene.add(anchorMesh);

  const localAttach = new THREE.Vector3(0.5, 0.2, 0.1);
  const restLength = 1.1;
  const k = 18;
  const c = 1.8;
  const gravity = new THREE.Vector3(0, -9.8, 0);

  const springLine = createSpringLine(scene, anchorWorld, body.position);

  const statElems = {
    springLen: document.getElementById('springLen'),
    springForce: document.getElementById('springForce'),
    springEnergy: document.getElementById('springEnergy'),
    totalEnergy: document.getElementById('totalEnergy')
  };

  let currentDamping = 0;

  return {
    step(dt){
      body.clearForcesAndTorque();
      body.addForceGlobal(gravity.clone().multiplyScalar(body.mass));

      const attachWorld = getWorldPoint(body, localAttach);
      const d = new THREE.Vector3().subVectors(attachWorld, anchorWorld);
      const len = d.length();
      let force = new THREE.Vector3(0,0,0);
      if(len > 1e-6){
        const dir = d.clone().multiplyScalar(1 / len);
        const stretch = len - restLength;
        const pointVel = getPointVelocity(body, attachWorld);
        const relVel = pointVel.dot(dir);
        const fMag = -k * stretch - c * relVel;
        force = dir.multiplyScalar(fMag);
        body.applyForceGlobalAtPointGlobal(attachWorld, force);
      }

      integrateRigidBodyWithForcesImplicitGyro(body, dt, currentDamping);

      mesh.position.copy(body.position);
      mesh.quaternion.copy(body.quaternion);

      const attachWorldNow = getWorldPoint(body, localAttach);
      const lenNow = attachWorldNow.distanceTo(anchorWorld);
      updateSpringLine(springLine, anchorWorld, attachWorldNow);

      if(statElems.springLen){
        statElems.springLen.textContent = lenNow.toFixed(3);
        statElems.springForce.textContent = force.length().toFixed(3);
        const springEnergy = 0.5 * k * Math.pow(Math.max(0, lenNow - restLength), 2);
        statElems.springEnergy.textContent = springEnergy.toFixed(4);
        statElems.totalEnergy.textContent = (springEnergy + kineticEnergy(body)).toFixed(4);
      }
    },
    setDamping(value){ currentDamping = value; },
    statElems,
    dispose(){
      try{ scene.remove(mesh); }catch(e){}
      try{ geom.dispose(); }catch(e){}
      try{ mat.dispose(); }catch(e){}
      try{ scene.remove(anchorMesh); }catch(e){}
      try{ anchorGeom.dispose(); }catch(e){}
      try{ anchorMat.dispose(); }catch(e){}
      try{ scene.remove(springLine); }catch(e){}
      try{ springLine.geometry.dispose(); }catch(e){}
      try{ springLine.material.dispose(); }catch(e){}
    }
  };
}

function getWorldPoint(body, localPoint){
  const R = body.getRotationMatrix(body.quaternion);
  return localPoint.clone().applyMatrix3(R).add(body.position);
}

function getPointVelocity(body, worldPoint){
  const r = new THREE.Vector3().subVectors(worldPoint, body.position);
  const w = body.getAngularVelocityGlobal();
  return body.linearVelocity.clone().add(w.clone().cross(r));
}

function kineticEnergy(body){
  const linear = 0.5 * body.mass * body.linearVelocity.lengthSq();
  const w = body.getAngularVelocityGlobal();
  const Iw = w.clone().applyMatrix3(body.getInertiaGlobal());
  const angular = 0.5 * w.dot(Iw);
  return linear + angular;
}

function createSpringLine(scene, from, to){
  const points = [from.clone(), to.clone()];
  const geom = new THREE.BufferGeometry().setFromPoints(points);
  const mat = new THREE.LineBasicMaterial({color:0xffee77});
  const line = new THREE.Line(geom, mat);
  scene.add(line);
  return line;
}

function updateSpringLine(line, from, to){
  const pos = line.geometry.attributes.position.array;
  pos[0] = from.x; pos[1] = from.y; pos[2] = from.z;
  pos[3] = to.x; pos[4] = to.y; pos[5] = to.z;
  line.geometry.attributes.position.needsUpdate = true;
}
