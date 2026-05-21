// Part 2 Scene 2 — Two free rigid bodies coupled by a distance constraint
// between off-center attachment points. Four solver methods are supported:
//
//   - 'xpbd'        : XPBD position-based dynamics
//                     (Müller et al. 2020, "Detailed Rigid Body Simulation
//                      with Extended Position-Based Dynamics")
//   - 'si_baumgarte': Sequential Impulses + Baumgarte stabilization
//                     (Catto 2005, GDC "Iterative Dynamics")
//   - 'si_ngs'      : Sequential Impulses (no bias) + Nonlinear Gauss-Seidel
//                     position projection (Catto, solver2d notes)
//   - 'si_soft'     : Sequential Impulses with soft (Buddha-spring) constraint
//                     (Catto 2011, GDC "Soft Constraints")
//
// Distance constraint:
//   C(x,q) = |p2_w − p1_w| − L,   p_i_w = x_i + R_i · r_i_local
//   n = (p2_w − p1_w) / |p2_w − p1_w|
//   Jacobian:  J = [−n, −(r1×n), n, (r2×n)]
//   Effective mass denominator:
//     K = 1/m1 + 1/m2 + (r1×n)ᵀ I1⁻¹ (r1×n) + (r2×n)ᵀ I2⁻¹ (r2×n)
//   m_eff = 1/K.

import { RigidBody3D } from '../engine/rigidbody3d.js';

// ─── Constants ──────────────────────────────────────────────────────────────
// Buddha spring (si_soft) tuning. ω = 2π·f. With m_eff ≈ 1 this gives
// k ≈ 18, c ≈ 1.8 — visibly compliant but not floppy.
const BUDDHA_FREQUENCY_HZ = 0.675;
// Two off-center bodies create stronger coupling than the single-anchor spring
// in Scene 1; with ζ=0.212 the system slowly pumps energy through warm-start.
// Bumped to 0.5 to remain stable at slider=0.
const BUDDHA_DAMPING_RATIO = 0.5;
const SOFT_DAMPING_GAIN = 10;
// Warm-start decay. Disabled (0) — NGS works without warm-start and is stable;
// for Baumgarte/Soft, carrying λ across frames with off-center attachments and
// large angular velocities introduces stale impulses that pump energy.
// Additionally, the previous implementation pre-applied λ_warm as a hard
// impulse BEFORE the loop AND penalised it inside the soft formula via γ·λ_acc,
// effectively double-counting and injecting work each frame.
const SI_WARM_START_DECAY = 0;

// Baumgarte bias factor. β/h is multiplied by C to feed position error into
// the velocity solver as bias. β=0.2 over-pumps with warm-start ⇒ use 0.1.
const BAUMGARTE_BETA = 0.1;

// NGS position-solver Baumgarte (slop=0). Per Catto/solver2d.
const NGS_BAUMGARTE = 0.2;

// SI velocity iterations.
const SI_VELOCITY_ITERATIONS = 8;
const SI_POSITION_ITERATIONS = 4; // for NGS

// XPBD substeps per dt.
const XPBD_SUBSTEPS = 20;
// Distance constraint compliance (α). α=0 → hard constraint.
const XPBD_COMPLIANCE = 0.0;

// ─── Scene ──────────────────────────────────────────────────────────────────
export function loadPart2Scene2TwoBodies(rendererData, method = 'xpbd') {
  const scene = rendererData.scene;

  // Two boxes of different size to make the off-center attachment asymmetric
  const size1 = [0.6, 0.4, 0.4];
  const size2 = [0.8, 0.3, 0.5];

  const body1 = new RigidBody3D({ mass: 1.0, size: size1, position: [-0.9, 0.2, 0] });
  const body2 = new RigidBody3D({ mass: 1.5, size: size2, position: [ 0.9, 0.2, 0] });

  // Off-center local attachment points (NOT at center of mass)
  const r1Local = new THREE.Vector3( 0.3,  0.2,  0.2);   // top-right-front corner area on body1
  const r2Local = new THREE.Vector3(-0.4, -0.15, -0.25); // bottom-left-back on body2

  // Rest length = current world distance at t=0 (so constraint starts satisfied)
  const initAttach1 = getWorldPoint(body1, r1Local);
  const initAttach2 = getWorldPoint(body2, r2Local);
  const restLength = initAttach1.distanceTo(initAttach2);

  // Initial excitation
  body1.linearVelocity.set(0, 0, 1.2);
  body1.angularVelocityLocal.set(0, 2.5, 0);
  body2.linearVelocity.set(0, 0, -1.0);
  body2.angularVelocityLocal.set(1.5, 0, 0);

  // ─── Meshes ────────────────────────────────────────────────────────────
  const geom1 = new THREE.BoxGeometry(...size1);
  const mat1 = new THREE.MeshStandardMaterial({ color: 0x88ccff });
  const mesh1 = new THREE.Mesh(geom1, mat1);
  scene.add(mesh1);

  const geom2 = new THREE.BoxGeometry(...size2);
  const mat2 = new THREE.MeshStandardMaterial({ color: 0xffaa88 });
  const mesh2 = new THREE.Mesh(geom2, mat2);
  scene.add(mesh2);

  // Small markers for attachment points
  const markerGeom = new THREE.SphereGeometry(0.04, 12, 10);
  const markerMat1 = new THREE.MeshStandardMaterial({ color: 0xffee55 });
  const markerMat2 = new THREE.MeshStandardMaterial({ color: 0xffee55 });
  const marker1 = new THREE.Mesh(markerGeom, markerMat1);
  const marker2 = new THREE.Mesh(markerGeom, markerMat2);
  scene.add(marker1);
  scene.add(marker2);

  // Constraint visualization
  const lineGeom = new THREE.BufferGeometry().setFromPoints([initAttach1.clone(), initAttach2.clone()]);
  const lineMat = new THREE.LineBasicMaterial({ color: 0xffffff });
  const constraintLine = new THREE.Line(lineGeom, lineMat);
  scene.add(constraintLine);

  // ─── State ─────────────────────────────────────────────────────────────
  let currentDamping = 0;
  // Warm-start: accumulated impulse from previous frame (SI methods only).
  let lambdaWarm = 0;

  const statElems = {
    springLen: document.getElementById('springLen'),
    springForce: document.getElementById('springForce'),
    springEnergy: document.getElementById('springEnergy'),
    totalEnergy: document.getElementById('totalEnergy')
  };

  // ─── Helper: compute constraint quantities at the current pose ────────
  // Returns {n, C, r1World, r2World, attach1, attach2, K, m_eff, w1, w2}
  function computeConstraintGeom(b1, b2) {
    const attach1 = getWorldPoint(b1, r1Local);
    const attach2 = getWorldPoint(b2, r2Local);
    const d = new THREE.Vector3().subVectors(attach2, attach1);
    const len = d.length();
    if (len < 1e-9) {
      return null;
    }
    const n = d.clone().multiplyScalar(1 / len);
    const C = len - restLength;
    const r1 = new THREE.Vector3().subVectors(attach1, b1.position);
    const r2 = new THREE.Vector3().subVectors(attach2, b2.position);

    const I1invW = new THREE.Matrix3().copy(b1.getInertiaGlobal()).invert();
    const I2invW = new THREE.Matrix3().copy(b2.getInertiaGlobal()).invert();

    const r1xn = r1.clone().cross(n);
    const r2xn = r2.clone().cross(n);
    const w1 = 1 / b1.mass + r1xn.dot(r1xn.clone().applyMatrix3(I1invW));
    const w2 = 1 / b2.mass + r2xn.dot(r2xn.clone().applyMatrix3(I2invW));
    const K = w1 + w2;
    const m_eff = K > 1e-9 ? 1 / K : 0;

    return { n, C, len, r1, r2, attach1, attach2, I1invW, I2invW, r1xn, r2xn, w1, w2, K, m_eff };
  }

  // ─── Apply a linear impulse λ·n (sign convention: body1 receives -, body2 +)
  function applyConstraintImpulse(b1, b2, info, lambda) {
    const { n, r1, r2 } = info;
    const imp = n.clone().multiplyScalar(lambda);

    // Body1: receive -imp
    b1.linearVelocity.addScaledVector(imp, -1 / b1.mass);
    addAngularImpulse(b1, r1, imp.clone().multiplyScalar(-1));

    // Body2: receive +imp
    b2.linearVelocity.addScaledVector(imp, 1 / b2.mass);
    addAngularImpulse(b2, r2, imp);
  }

  // Δω_local = I_local⁻¹ · Rᵀ · (r × impulseWorld)
  function addAngularImpulse(body, rWorld, impulseWorld) {
    const torqueImpulseWorld = rWorld.clone().cross(impulseWorld);
    const R = body.getRotationMatrix(body.quaternion);
    const Rt = new THREE.Matrix3().copy(R).transpose();
    const torqueImpulseLocal = torqueImpulseWorld.applyMatrix3(Rt);
    const deltaOmegaLocal = torqueImpulseLocal.applyMatrix3(body.inertiaLocalInversed);
    body.angularVelocityLocal.add(deltaOmegaLocal);
  }

  // ─── Position-level correction (XPBD / NGS) ──────────────────────────
  // Apply position-only correction dλ along n. Body1 gets -, body2 gets +.
  function applyPositionCorrection(b1, b2, info, dlambda) {
    const { n, r1, r2, I1invW, I2invW } = info;
    const p = n.clone().multiplyScalar(dlambda);

    // Linear position update
    b1.position.addScaledVector(p, -1 / b1.mass);
    b2.position.addScaledVector(p,  1 / b2.mass);

    // Angular position update.
    // δθ_world_1 = -I1_world⁻¹ · (r1 × p)
    // δθ_world_2 = +I2_world⁻¹ · (r2 × p)
    const dTheta1 = r1.clone().cross(p).applyMatrix3(I1invW).multiplyScalar(-1);
    const dTheta2 = r2.clone().cross(p).applyMatrix3(I2invW);

    applyAngularDelta(b1, dTheta1);
    applyAngularDelta(b2, dTheta2);
  }

  // Update quaternion by world-space angular delta:
  //   q ← normalize(q + ½ * [dθ, 0] · q)
  function applyAngularDelta(body, dThetaWorld) {
    const q = body.quaternion;
    const dq = new THREE.Quaternion(
      0.5 * (dThetaWorld.x * q.w + dThetaWorld.y * q.z - dThetaWorld.z * q.y),
      0.5 * (-dThetaWorld.x * q.z + dThetaWorld.y * q.w + dThetaWorld.z * q.x),
      0.5 * (dThetaWorld.x * q.y - dThetaWorld.y * q.x + dThetaWorld.z * q.w),
      0.5 * (-dThetaWorld.x * q.x - dThetaWorld.y * q.y - dThetaWorld.z * q.z)
    );
    q.set(q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w);
    q.normalize();
  }

  // ─── Velocity-level relative velocity along n ────────────────────────
  function relativeVelAlongN(b1, b2, info) {
    const { n, r1, r2 } = info;
    const w1w = b1.getAngularVelocityGlobal();
    const w2w = b2.getAngularVelocityGlobal();
    const v1p = b1.linearVelocity.clone().add(w1w.cross(r1));
    const v2p = b2.linearVelocity.clone().add(w2w.cross(r2));
    return v2p.sub(v1p).dot(n);
  }

  // ─── Simple semi-implicit integration of pose (no forces) ─────────────
  function integratePose(body, dt) {
    body.position.addScaledVector(body.linearVelocity, dt);
    const wLocal = body.angularVelocityLocal;
    const q = body.quaternion;
    // q ← q + 0.5 * [wLocal, 0] * q · dt    (local-frame variant: q · [wLocal,0])
    // Use the same convention as integrator.js (q' = 0.5 * wQuat * q · dt)
    const wQuat = new THREE.Quaternion(wLocal.x, wLocal.y, wLocal.z, 0);
    const dq = wQuat.multiply(q);
    q.set(q.x + 0.5 * dt * dq.x,
          q.y + 0.5 * dt * dq.y,
          q.z + 0.5 * dt * dq.z,
          q.w + 0.5 * dt * dq.w);
    q.normalize();
  }

  function applyLinearDamping(body, dt) {
    if (currentDamping > 0) {
      body.linearVelocity.multiplyScalar(1 - currentDamping * dt);
      body.angularVelocityLocal.multiplyScalar(1 - currentDamping * dt);
    }
  }

  // ─── Method: XPBD ─────────────────────────────────────────────────────
  // Algorithm 2 of Müller et al. 2020. Run XPBD_SUBSTEPS substeps, each
  // with 1 constraint iteration. After each substep, derive velocity from
  // position change.
  function stepXPBD(dt) {
    const h = dt / XPBD_SUBSTEPS;
    const alphaTilde = XPBD_COMPLIANCE / (h * h);

    for (let s = 0; s < XPBD_SUBSTEPS; s++) {
      // 1. Save previous state
      const x1Prev = body1.position.clone();
      const x2Prev = body2.position.clone();
      const q1Prev = body1.quaternion.clone();
      const q2Prev = body2.quaternion.clone();

      // 2. Predict pose with current velocity (no external forces)
      integratePose(body1, h);
      integratePose(body2, h);

      // 3. Solve distance constraint at the position level (1 iteration)
      const info = computeConstraintGeom(body1, body2);
      if (info && info.K > 1e-9) {
        const lambda = 0; // single-iteration: λ starts at 0 each substep
        const dlambda = (-info.C - alphaTilde * lambda) / (info.K + alphaTilde);
        applyPositionCorrection(body1, body2, info, dlambda);
      }

      // 4. Update velocities from position change
      body1.linearVelocity.copy(body1.position).sub(x1Prev).multiplyScalar(1 / h);
      body2.linearVelocity.copy(body2.position).sub(x2Prev).multiplyScalar(1 / h);

      setAngularVelFromQuatDiff(body1, q1Prev, h);
      setAngularVelFromQuatDiff(body2, q2Prev, h);

      applyLinearDamping(body1, h);
      applyLinearDamping(body2, h);
    }

    lambdaWarm = 0; // XPBD doesn't accumulate λ across frames
  }

  // ω_world from q_diff: q_new * q_prev⁻¹.
  // ω = 2 · (q_new * q_prev⁻¹).xyz / h   (with sign flip if w<0)
  function setAngularVelFromQuatDiff(body, qPrev, h) {
    const qInv = qPrev.clone().conjugate();
    const dq = body.quaternion.clone().multiply(qInv);
    const sign = dq.w >= 0 ? 1 : -1;
    const omegaWorld = new THREE.Vector3(dq.x, dq.y, dq.z).multiplyScalar(2 * sign / h);
    body.setAngularVelocityGlobal(omegaWorld);
  }

  // ─── Methods: SI + Baumgarte / NGS / Soft ─────────────────────────────
  function stepSI(dt) {
    // 1. (no external forces) — damping applied at end
    // 2. Build constraint info at the start-of-step pose
    let info = computeConstraintGeom(body1, body2);
    if (!info || info.K < 1e-9) {
      // Bodies coincident: skip constraint, integrate freely
      integratePose(body1, dt);
      integratePose(body2, dt);
      applyLinearDamping(body1, dt);
      applyLinearDamping(body2, dt);
      lambdaWarm = 0;
      return { lambdaTotal: 0 };
    }

    let lambdaAcc = 0;
    let lambdaTotal = 0;

    if (method === 'si_baumgarte') {
      // Warm-start
      applyConstraintImpulse(body1, body2, info, lambdaWarm);
      lambdaAcc = lambdaWarm;
      lambdaTotal += lambdaWarm;

      const bias = -(BAUMGARTE_BETA / dt) * info.C;
      for (let it = 0; it < SI_VELOCITY_ITERATIONS; it++) {
        const Jv = relativeVelAlongN(body1, body2, info);
        const lambda = info.m_eff * (-Jv + bias);
        applyConstraintImpulse(body1, body2, info, lambda);
        lambdaAcc += lambda;
        lambdaTotal += lambda;
      }
      lambdaWarm = lambdaAcc * SI_WARM_START_DECAY;
    }
    else if (method === 'si_ngs') {
      // Velocity solver WITHOUT bias.
      // (No warm-start — pseudo-velocities don't accumulate similarly.)
      for (let it = 0; it < SI_VELOCITY_ITERATIONS; it++) {
        const Jv = relativeVelAlongN(body1, body2, info);
        const lambda = info.m_eff * (-Jv);
        applyConstraintImpulse(body1, body2, info, lambda);
        lambdaTotal += lambda;
      }
      lambdaWarm = 0;
    }
    else if (method === 'si_soft') {
      // Warm-start
      applyConstraintImpulse(body1, body2, info, lambdaWarm);
      lambdaAcc = lambdaWarm;
      lambdaTotal += lambdaWarm;

      const omega = 2 * Math.PI * BUDDHA_FREQUENCY_HZ;
      const zeta = BUDDHA_DAMPING_RATIO + currentDamping * SOFT_DAMPING_GAIN;
      const k = info.m_eff * omega * omega;
      const c = 2 * info.m_eff * zeta * omega;
      const gamma = 1 / (c + dt * k);
      const beta = dt * k / (c + dt * k);
      const m_soft = 1 / (1 / info.m_eff + gamma);

      for (let it = 0; it < SI_VELOCITY_ITERATIONS; it++) {
        const Jv = relativeVelAlongN(body1, body2, info);
        const lambda = -m_soft * (Jv + (beta / dt) * info.C + gamma * lambdaAcc);
        applyConstraintImpulse(body1, body2, info, lambda);
        lambdaAcc += lambda;
        lambdaTotal += lambda;
      }
      lambdaWarm = lambdaAcc * SI_WARM_START_DECAY;
    }

    // 3. Integrate pose
    integratePose(body1, dt);
    integratePose(body2, dt);

    // 4. NGS position projection (after pose update)
    if (method === 'si_ngs') {
      for (let it = 0; it < SI_POSITION_ITERATIONS; it++) {
        const pinfo = computeConstraintGeom(body1, body2);
        if (!pinfo || pinfo.K < 1e-9) break;
        // pseudo-position impulse:  Δλ = -m_eff · NGS_BAUMGARTE · C
        const dlambda = -pinfo.m_eff * NGS_BAUMGARTE * pinfo.C;
        applyPositionCorrection(body1, body2, pinfo, dlambda);
      }
    }

    // 5. Damping. For si_soft, ζ already damps motion *along* the constraint,
    // but bodies are free to translate/rotate transversally — apply linear
    // damping to both linear and angular velocity uniformly here.
    applyLinearDamping(body1, dt);
    applyLinearDamping(body2, dt);

    return { lambdaTotal };
  }

  // ─── Main step ─────────────────────────────────────────────────────────
  return {
    step(dt) {
      let lambdaForStats = 0;
      if (method === 'xpbd') {
        stepXPBD(dt);
      } else {
        const res = stepSI(dt);
        lambdaForStats = res.lambdaTotal;
      }

      // Sync meshes
      mesh1.position.copy(body1.position);
      mesh1.quaternion.copy(body1.quaternion);
      mesh2.position.copy(body2.position);
      mesh2.quaternion.copy(body2.quaternion);

      const a1 = getWorldPoint(body1, r1Local);
      const a2 = getWorldPoint(body2, r2Local);
      marker1.position.copy(a1);
      marker2.position.copy(a2);
      updateConstraintLine(constraintLine, a1, a2);

      const len = a1.distanceTo(a2);

      if (statElems.springLen) {
        statElems.springLen.textContent = `${len.toFixed(4)} (rest ${restLength.toFixed(3)}, err ${(len - restLength).toExponential(2)})`;
        statElems.springForce.textContent = (method === 'xpbd')
          ? '—'
          : (Math.abs(lambdaForStats) / dt).toFixed(3);
        statElems.springEnergy.textContent = '—';
        const ke = kineticEnergy(body1) + kineticEnergy(body2);
        statElems.totalEnergy.textContent = ke.toFixed(4);
      }
    },

    setDamping(value) { currentDamping = value; },

    statElems,

    dispose() {
      try { scene.remove(mesh1); } catch(e){}
      try { geom1.dispose(); } catch(e){}
      try { mat1.dispose(); } catch(e){}
      try { scene.remove(mesh2); } catch(e){}
      try { geom2.dispose(); } catch(e){}
      try { mat2.dispose(); } catch(e){}
      try { scene.remove(marker1); } catch(e){}
      try { scene.remove(marker2); } catch(e){}
      try { markerGeom.dispose(); } catch(e){}
      try { markerMat1.dispose(); } catch(e){}
      try { markerMat2.dispose(); } catch(e){}
      try { scene.remove(constraintLine); } catch(e){}
      try { lineGeom.dispose(); } catch(e){}
      try { lineMat.dispose(); } catch(e){}
    }
  };
}

// ─── Helpers ────────────────────────────────────────────────────────────────

function getWorldPoint(body, localPoint) {
  const R = body.getRotationMatrix(body.quaternion);
  return localPoint.clone().applyMatrix3(R).add(body.position);
}

function kineticEnergy(body) {
  const linear = 0.5 * body.mass * body.linearVelocity.lengthSq();
  const w = body.getAngularVelocityGlobal();
  const Iw = w.clone().applyMatrix3(body.getInertiaGlobal());
  const angular = 0.5 * w.dot(Iw);
  return linear + angular;
}

function updateConstraintLine(line, from, to) {
  const pos = line.geometry.attributes.position.array;
  pos[0] = from.x; pos[1] = from.y; pos[2] = from.z;
  pos[3] = to.x;   pos[4] = to.y;   pos[5] = to.z;
  line.geometry.attributes.position.needsUpdate = true;
}
