import { convertToQuaternion, add, multiplyScalar } from './utils.js';

// Semi-implicit Euler for free rigid-body rotation with constant angular momentum L
export function integrateInGlobalCoords(body, dt, damping){
  const I_global = body.getInertiaGlobal();
  const I_global_inv = new THREE.Matrix3().copy(I_global).invert();
  const w = body.L_start.clone().applyMatrix3(I_global_inv);
  body.setAngularVelocityGlobal(w);

  const wQuat = convertToQuaternion(w);
  const delta = new THREE.Quaternion(
    wQuat.x * dt * 0.5,
    wQuat.y * dt * 0.5,
    wQuat.z * dt * 0.5,
    wQuat.w * dt * 0.5 + 1
  );
  body.quaternion.copy(delta.multiply(body.quaternion));
  body.quaternion.normalize();

  // Damping
  body.L_start.multiplyScalar(1 - damping * dt);
}

export function integrateInLocalCoordsNoGyro(body, dt, damping){
  const w = body.angularVelocityLocal;
  const q = body.quaternion;

  const wQuat = convertToQuaternion(w);
  const dq = wQuat.multiply(q);
  q.copy(add(q, multiplyScalar(dq, dt * 0.5)));
  q.normalize();

  // Damping
  w.multiplyScalar(1 - damping * dt);
}

export function integrateInLocalCoordsExplicitGyro(body, dt, damping){
  const w = body.angularVelocityLocal.clone();
  const q = body.quaternion;

  const Iw = w.clone().applyMatrix3(body.inertiaLocal);
  const minusWxIw = w.clone().cross(Iw).negate();
  const gyro = minusWxIw.applyMatrix3(body.inertiaLocalInversed).multiplyScalar(dt);
  w.add(gyro);
  body.angularVelocityLocal.copy(w);

  const wQuat = convertToQuaternion(w);
  const dq = wQuat.multiply(q);
  q.copy(add(q, multiplyScalar(dq, dt * 0.5)));
  q.normalize();

  // Damping
  body.angularVelocityLocal.multiplyScalar(1 - damping * dt);
}
