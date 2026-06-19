import { RigidBody3D } from '../engine/rigidbody3d.js';
import { integrateRigidBodyWithForcesImplicitGyro } from '../engine/integrator.js';

// Buddha spring constants: frequencyHz and dampingRatio are tuned so that
// at the initial effective mass (m_eff ≈ 1) we get k ≈ 18, c ≈ 1.8.
// ω = 2π·f  →  k = m_eff·ω²,  c = 2·m_eff·ζ·ω
// f = sqrt(18/(4π²)) ≈ 0.675 Hz,  ζ = 1.8/(2·sqrt(18)) ≈ 0.212
const BUDDHA_FREQUENCY_HZ = 0.675;
const BUDDHA_DAMPING_RATIO = 0.212;

export function loadPart2Scene1Spring(rendererData, method = 'externalForce') {
  const scene = rendererData.scene;

  const boxSize = [1, 0.4, 0.2];
  const body = new RigidBody3D({ mass: 1, size: boxSize, position: [0, 0.2, 0] });

  const geom = new THREE.BoxGeometry(...boxSize);
  const mat = new THREE.MeshStandardMaterial({ color: 0x88ffaa });
  const mesh = new THREE.Mesh(geom, mat);
  scene.add(mesh);

  const anchorWorld = new THREE.Vector3(0, 1.6, 0);
  const anchorGeom = new THREE.SphereGeometry(0.05, 16, 12);
  const anchorMat = new THREE.MeshStandardMaterial({ color: 0xffaa33 });
  const anchorMesh = new THREE.Mesh(anchorGeom, anchorMat);
  anchorMesh.position.copy(anchorWorld);
  scene.add(anchorMesh);

  const localAttach = new THREE.Vector3(0.0, 0.2, 0.1);
  const restLength = 1.1;
  // External force spring constants
  const k_ext = 18;
  const c_ext = 1.8;
  const gravity = new THREE.Vector3(0, -9.8, 0);

  const springLine = createSpringLine(scene, anchorWorld, body.position);

  const statElems = {
    springLen: document.getElementById('springLen'),
    springForce: document.getElementById('springForce'),
    springEnergy: document.getElementById('springEnergy'),
    totalEnergy: document.getElementById('totalEnergy')
  };

  let currentDamping = 0;
  // Warm-start: accumulated constraint impulse from the previous frame
  let lambdaWarm = 0;

  // ─── External-force spring ────────────────────────────────────────────────
  function applyExternalForceSpring(dt) {
    body.clearForcesAndTorque();
    body.addForceGlobal(gravity.clone().multiplyScalar(body.mass));

    const attachWorld = getWorldPoint(body, localAttach);
    const d = new THREE.Vector3().subVectors(attachWorld, anchorWorld);
    const len = d.length();
    let force = new THREE.Vector3(0, 0, 0);
    if (len > 1e-6) {
      const dir = d.clone().multiplyScalar(1 / len);
      const stretch = len - restLength;
      const pointVel = getPointVelocity(body, attachWorld);
      const relVel = pointVel.dot(dir);
      const fMag = -k_ext * stretch - c_ext * relVel;
      force = dir.multiplyScalar(fMag);
      body.applyForceGlobalAtPointGlobal(attachWorld, force);
    }

    integrateRigidBodyWithForcesImplicitGyro(body, dt, currentDamping);
    return { force, attachWorld };
  }

  // ─── Soft constraint (Buddha spring) ─────────────────────────────────────
  // Implements the sequential-impulse soft constraint from:
  // Erin Catto, "Soft Constraints", GDC 2011
  //
  // Constraint: C = |attachWorld - anchorWorld| - restLength = 0
  // Jacobian (1D): J = [n, r×n]  where n = (attach - anchor)/|…|
  //
  // Effective mass:  1/m_eff = 1/m + (r×n)ᵀ · I_world⁻¹ · (r×n)
  // Spring/damper from harmonic oscillator:
  //   ω = 2π·f,  k = m_eff·ω²,  c = 2·m_eff·ζ·ω
  // Softness (impulse form, h = dt):
  //   γ_imp = 1/(c + h·k)
  //   β     = h·k/(c + h·k)
  //   m_soft = 1/(1/m_eff + γ_imp)
  // Impulse:
  //   Jv = (v_lin + ω_world × r) · n
  //   λ  = −m_soft · (Jv + (β/h)·C + γ_imp·λ_acc)
  //   λ_acc += λ
  function applySoftConstraintSpring(dt) {
    // 1. Apply gravity impulse directly to linear velocity
    body.linearVelocity.addScaledVector(gravity, dt);

    const attachWorld = getWorldPoint(body, localAttach);
    const d = new THREE.Vector3().subVectors(attachWorld, anchorWorld);
    const len = d.length();

    // lambdaFrame: total impulse applied this step (for stats)
    let lambdaFrame = 0;
    // lambdaAccFrame: accumulated impulse within this step's iterations
    // initialised from warm-start of previous frame
    let lambdaAccFrame = lambdaWarm;

    if (len > 1e-6) {
      const n = d.clone().multiplyScalar(1 / len);   // constraint axis
      const C = len - restLength;                      // position error
      const r = new THREE.Vector3().subVectors(attachWorld, body.position);

      // Effective mass
      const I_world_inv = new THREE.Matrix3().copy(body.getInertiaGlobal()).invert();
      const rCrossN = r.clone().cross(n);              // r × n
      const I_inv_rCrossN = rCrossN.clone().applyMatrix3(I_world_inv);
      const angularTerm = rCrossN.dot(I_inv_rCrossN); // (r×n)ᵀ I⁻¹ (r×n)
      const m_eff = 1 / (1 / body.mass + angularTerm);

      // Spring/damper constants from harmonic oscillator.
      // UI damping slider is mapped to an additive damping ratio with a
      // stronger gain so user sees the effect clearly: slider 0..0.9
      // maps roughly to ζ_extra 0..9 (well into the over-damped region).
      const DAMPING_GAIN = 10;
      const omega = 2 * Math.PI * BUDDHA_FREQUENCY_HZ;
      const zeta = BUDDHA_DAMPING_RATIO + currentDamping * DAMPING_GAIN;
      const k_soft = m_eff * omega * omega;
      const c_soft = 2 * m_eff * zeta * omega;

      // Softness parameters (impulse form)
      // γ_imp = 1/(c + h·k),  β = h·k/(c + h·k),  m_soft = 1/(1/m_eff + γ_imp)
      const gamma_imp = 1 / (c_soft + dt * k_soft);
      const beta = dt * k_soft / (c_soft + dt * k_soft);
      const m_soft = 1 / (1 / m_eff + gamma_imp);

      // Iterative impulse solver (sequential impulses, 4 iterations)
      const ITERATIONS = 4;
      for (let iter = 0; iter < ITERATIONS; iter++) {
        // Relative velocity at attachment point along constraint axis
        const w_world = body.getAngularVelocityGlobal();
        const v_attach = body.linearVelocity.clone().add(w_world.clone().cross(r));
        const Jv = v_attach.dot(n);

        // Soft constraint impulse: λ = −m_soft·(Jv + (β/h)·C + γ_imp·λ_acc)
        const lambda = -m_soft * (Jv + (beta / dt) * C + gamma_imp * lambdaAccFrame);
        lambdaAccFrame += lambda;
        lambdaFrame += lambda;

        // Apply linear impulse: Δv = (λ/m)·n
        body.linearVelocity.addScaledVector(n, lambda / body.mass);

        // Apply angular impulse in local coords: Δω_local = I_local⁻¹ · Rᵀ · (r × (λ·n))
        const impulseWorld = n.clone().multiplyScalar(lambda);
        const torqueImpulseWorld = r.clone().cross(impulseWorld);
        const R = body.getRotationMatrix(body.quaternion);
        const Rt = new THREE.Matrix3().copy(R).transpose();
        const torqueImpulseLocal = torqueImpulseWorld.clone().applyMatrix3(Rt);
        const deltaOmegaLocal = torqueImpulseLocal.clone().applyMatrix3(body.inertiaLocalInversed);
        body.angularVelocityLocal.add(deltaOmegaLocal);
      }

      // Save accumulated impulse as warm-start for next frame (with slight decay)
      lambdaWarm = lambdaAccFrame * 0.95;
    } else {
      lambdaWarm = 0;
    }

    // Integrate pose (position + quaternion) with zero forces/torques.
    // Pass damping=0 to the integrator: damping is already applied through
    // the constraint's ζ above, double-damping would feel wrong.
    body.clearForcesAndTorque();
    integrateRigidBodyWithForcesImplicitGyro(body, dt, 0);

    return { lambda: lambdaFrame, dt };
  }

  // ─── Main step ────────────────────────────────────────────────────────────
  return {
    step(dt) {
      let force = new THREE.Vector3(0, 0, 0);
      let attachWorldNow;
      let lambdaForStats = 0;

      if (method === 'softConstraint') {
        const res = applySoftConstraintSpring(dt);
        lambdaForStats = res.lambda;
        attachWorldNow = getWorldPoint(body, localAttach);
      } else {
        // externalForce (default)
        const res = applyExternalForceSpring(dt);
        force = res.force;
        attachWorldNow = getWorldPoint(body, localAttach);
      }

      mesh.position.copy(body.position);
      mesh.quaternion.copy(body.quaternion);

      const lenNow = attachWorldNow.distanceTo(anchorWorld);
      updateSpringLine(springLine, anchorWorld, attachWorldNow);

      if (statElems.springLen) {
        statElems.springLen.textContent = lenNow.toFixed(3);
        if (method === 'softConstraint') {
          // force ≈ |λ| / dt
          statElems.springForce.textContent = (Math.abs(lambdaForStats) / dt).toFixed(3);
        } else {
          statElems.springForce.textContent = force.length().toFixed(3);
        }
        const springEnergy = 0.5 * k_ext * Math.pow(Math.max(0, lenNow - restLength), 2);
        statElems.springEnergy.textContent = springEnergy.toFixed(4);
        statElems.totalEnergy.textContent = (springEnergy + kineticEnergy(body)).toFixed(4);
      }
    },
    setDamping(value) { currentDamping = value; },
    statElems,
    dispose() {
      try { scene.remove(mesh); } catch (e) { }
      try { geom.dispose(); } catch (e) { }
      try { mat.dispose(); } catch (e) { }
      try { scene.remove(anchorMesh); } catch (e) { }
      try { anchorGeom.dispose(); } catch (e) { }
      try { anchorMat.dispose(); } catch (e) { }
      try { scene.remove(springLine); } catch (e) { }
      try { springLine.geometry.dispose(); } catch (e) { }
      try { springLine.material.dispose(); } catch (e) { }
    }
  };
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

function getWorldPoint(body, localPoint) {
  const R = body.getRotationMatrix(body.quaternion);
  return localPoint.clone().applyMatrix3(R).add(body.position);
}

function getPointVelocity(body, worldPoint) {
  const r = new THREE.Vector3().subVectors(worldPoint, body.position);
  const w = body.getAngularVelocityGlobal();
  return body.linearVelocity.clone().add(w.clone().cross(r));
}

function kineticEnergy(body) {
  const linear = 0.5 * body.mass * body.linearVelocity.lengthSq();
  const w = body.getAngularVelocityGlobal();
  const Iw = w.clone().applyMatrix3(body.getInertiaGlobal());
  const angular = 0.5 * w.dot(Iw);
  return linear + angular;
}

function createSpringLine(scene, from, to) {
  const points = [from.clone(), to.clone()];
  const geom = new THREE.BufferGeometry().setFromPoints(points);
  const mat = new THREE.LineBasicMaterial({ color: 0xffee77 });
  const line = new THREE.Line(geom, mat);
  scene.add(line);
  return line;
}

function updateSpringLine(line, from, to) {
  const pos = line.geometry.attributes.position.array;
  pos[0] = from.x; pos[1] = from.y; pos[2] = from.z;
  pos[3] = to.x; pos[4] = to.y; pos[5] = to.z;
  line.geometry.attributes.position.needsUpdate = true;
}
