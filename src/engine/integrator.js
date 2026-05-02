import { convertToQuaternion } from './utils.js';

// Semi-implicit Euler for free rigid-body rotation with constant angular momentum L
export function integrateInGlobalCoords(body, dt){
  const I_global = body.getInertiaGlobal();
  const I_global_inv = new THREE.Matrix3().copy(I_global).invert();
  const w = body.L.clone().applyMatrix3(I_global_inv);
  body.angularVelocity.copy(w);

  const wQuat = convertToQuaternion(w);
  const delta = new THREE.Quaternion(
    wQuat.x * dt * 0.5,
    wQuat.y * dt * 0.5,
    wQuat.z * dt * 0.5,
    wQuat.w * dt * 0.5 + 1
  );
  body.quaternion.copy(delta.multiply(body.quaternion));
  body.quaternion.normalize();
}
