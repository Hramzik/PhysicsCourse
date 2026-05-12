import { convertToQuaternion, add, multiplyScalar, skew, addMatrix3, multiplyMatrix3Scalar, identityMatrix3 } from './utils.js';

// Semi-implicit Euler for free rigid-body rotation with constant angular momentum L
export function integrateInGlobalCoords(body, dt, damping){
  const I_global = body.getInertiaGlobal();
  const I_global_inv = new THREE.Matrix3().copy(I_global).invert();
  const w = body.L_start.clone().applyMatrix3(I_global_inv);
  body.setAngularVelocityGlobal(w);

  const wQuat = convertToQuaternion(w);
  const onePlusWTdiv2 = new THREE.Quaternion(
    wQuat.x * dt * 0.5,
    wQuat.y * dt * 0.5,
    wQuat.z * dt * 0.5,
    wQuat.w * dt * 0.5 + 1
  );
  body.quaternion.copy(body.quaternion.multiply(onePlusWTdiv2));
  body.quaternion.normalize();

  // Damping
  body.L_start.multiplyScalar(1 - damping * dt);
}

export function integrateInLocalCoordsNoGyro(body, dt, damping){
  const w = body.angularVelocityLocal;
  const q = body.quaternion;

  const wQuat = convertToQuaternion(w);
  const wxq = wQuat.multiply(q);
  q.copy(add(q, multiplyScalar(wxq, dt * 0.5)));
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
  const wxq = wQuat.multiply(q);
  q.copy(add(q, multiplyScalar(wxq, dt * 0.5)));
  q.normalize();

  // Damping
  body.angularVelocityLocal.multiplyScalar(1 - damping * dt);
}

export function integrateInLocalCoordsImplicitGyro(body, dt, damping){
  const w_n = body.angularVelocityLocal.clone();
  const q = body.quaternion;

  // Single Newton--Raphson iteration to solve for u = w_{n+1}:
  // F(u) = u - w_n + dt * I^{-1} * (u x (I u)) = 0
  // J(u) = I3 + dt * I^{-1} * ( [Iu]_x + [u]_x * I )

  // initial guess: w_n
  // const u = new THREE.Vector3(0, 0, 0);
  const u = w_n.clone();

  const Iu = u.clone().applyMatrix3(body.inertiaLocal);
  const uxIu = u.clone().cross(Iu); // u x (I u)
  const F = u.clone().sub(w_n).add(uxIu.clone().applyMatrix3(body.inertiaLocalInversed).multiplyScalar(dt));

  // Build Jacobian J = I3 + dt * I^{-1} * ( skew(Iu) + skew(u) * I )
  const skewIu = skew(Iu);
  const skewU = skew(u);
  const skewUxI = skewU.clone().multiply(body.inertiaLocal);
  const skewIuPlusSkewUxI = addMatrix3(skewIu, skewUxI);

  const rightSumPart = multiplyMatrix3Scalar(body.inertiaLocalInversed.clone().multiply(skewIuPlusSkewUxI), dt);

  // J = I3 + rightSumPart
  const J = addMatrix3(identityMatrix3(), rightSumPart);

  const Jinv = new THREE.Matrix3().copy(J).invert();

  // single Newton step: u1 = u0 - J^{-1} * F(u0)
  const JxF = F.clone().applyMatrix3(Jinv);
  const u1 = u.clone().sub(JxF);
  body.angularVelocityLocal.copy(u1);

  const wNew = body.angularVelocityLocal;
  const wQuat = convertToQuaternion(wNew);
  const wxq = wQuat.multiply(q);
  q.copy(add(q, multiplyScalar(wxq, dt * 0.5)));
  q.normalize();

  // Damping
  body.angularVelocityLocal.multiplyScalar(1 - damping * dt);
}

// Symplectic Euler with external forces and implicit gyro term
// I dont know why it looks so bad, all i did was add torque term
export function integrateRigidBodyWithForcesImplicitGyro(body, dt, damping){
  const accelGlobal = body.getForceGlobal().multiplyScalar(1 / body.mass);
  body.linearVelocity.addScaledVector(accelGlobal, dt);
  body.linearVelocity.multiplyScalar(1 - damping * dt);
  body.position.addScaledVector(body.linearVelocity, dt);

  const w_n = body.angularVelocityLocal.clone();

  // Single Newton-Raphson iteration for u = w_{n+1}:
  // F(u) = u - w_n - dt * I^{-1} * tau + dt * I^{-1} * (u x (I u)) = 0
  // J(u) = I3 + dt * I^{-1} * ( [Iu]_x + [u]_x * I )
  // (J(u) is the same)

  const u = w_n.clone();

  const Iu = u.clone().applyMatrix3(body.inertiaLocal);
  const uxIu = u.clone().cross(Iu);
  const dtxIinvTau = body.torqueLocal.clone().applyMatrix3(body.inertiaLocalInversed).multiplyScalar(dt);

  const F = u.clone()
    .sub(w_n)
    .sub(dtxIinvTau)
    .add(uxIu.clone().applyMatrix3(body.inertiaLocalInversed).multiplyScalar(dt));

  const skewIu = skew(Iu);
  const skewU = skew(u);
  const skewUxI = skewU.clone().multiply(body.inertiaLocal);
  const skewIuPlusSkewUxI = addMatrix3(skewIu, skewUxI);
  const rightSumPart = multiplyMatrix3Scalar(body.inertiaLocalInversed.clone().multiply(skewIuPlusSkewUxI), dt);
  const J = addMatrix3(identityMatrix3(), rightSumPart);
  const Jinv = new THREE.Matrix3().copy(J).invert();

  const JxF = F.clone().applyMatrix3(Jinv);
  const u1 = u.clone().sub(JxF);
  body.angularVelocityLocal.copy(u1);

  const wQuat = convertToQuaternion(body.angularVelocityLocal);
  const wxq = wQuat.multiply(body.quaternion);
  body.quaternion.copy(add(body.quaternion, multiplyScalar(wxq, dt * 0.5)));
  body.quaternion.normalize();

  // Damping
  body.angularVelocityLocal.multiplyScalar(1 - damping * dt);
}
