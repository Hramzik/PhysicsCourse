// Contact solver primitives shared between XPBD and Sequential Impulses.
//
// Convention (matches collision.js):
//   contact.normal points FROM A TO B (A pushed in -normal, B pushed in +normal).
//   For static contacts (bodyB === null), normal points from body INTO solid;
//   bodyA is pushed in -normal (out of solid).
//
// Helper functions operate in world-space; per-body inertia tensors are
// recomputed each call from the body's current orientation.

// ─── Effective mass for a contact along a given axis ───────────────────────
// w_i = 1/m_i + ((r_i × axis)ᵀ · I_i⁻¹ · (r_i × axis))
// K   = w_A + w_B (w_B = 0 when bodyB is null/static)
function effectiveMassAlong(bodyA, rA, bodyB, rB, axis) {
  const rAxa = rA.clone().cross(axis);
  const IA_inv = new THREE.Matrix3().copy(bodyA.getInertiaGlobal()).invert();
  const wA = 1 / bodyA.mass + rAxa.dot(rAxa.clone().applyMatrix3(IA_inv));

  let wB = 0;
  let IB_inv = null;
  if (bodyB) {
    const rBxa = rB.clone().cross(axis);
    IB_inv = new THREE.Matrix3().copy(bodyB.getInertiaGlobal()).invert();
    wB = 1 / bodyB.mass + rBxa.dot(rBxa.clone().applyMatrix3(IB_inv));
  }
  const K = wA + wB;
  return { K, m_eff: K > 1e-9 ? 1 / K : 0, IA_inv, IB_inv };
}

// Effective mass for the normal direction (most callers). Also returns the
// world-space arms rA, rB so the impulse application can reuse them.
export function effectiveMass(contact) {
  const { bodyA, bodyB, point, normal } = contact;
  const rA = new THREE.Vector3().subVectors(point, bodyA.position);
  const rB = bodyB ? new THREE.Vector3().subVectors(point, bodyB.position) : null;
  const m = effectiveMassAlong(bodyA, rA, bodyB, rB, normal);
  return { K: m.K, m_eff: m.m_eff, rA, rB, IA_inv: m.IA_inv, IB_inv: m.IB_inv };
}

// ─── Relative velocity at the contact point ────────────────────────────────
// v_rel = (v_B + ω_B × r_B) − (v_A + ω_A × r_A).
// Caller projects onto whatever axis it cares about.
export function relativeVelocity(contact, ctx) {
  const { bodyA, bodyB } = contact;
  const wA_w = bodyA.getAngularVelocityGlobal();
  const vA = bodyA.linearVelocity.clone().add(wA_w.cross(ctx.rA));
  if (!bodyB) return vA.multiplyScalar(-1); // (0 + 0) - vA = -vA
  const wB_w = bodyB.getAngularVelocityGlobal();
  const vB = bodyB.linearVelocity.clone().add(wB_w.cross(ctx.rB));
  return vB.sub(vA);
}

export function relativeNormalVelocity(contact, ctx) {
  return relativeVelocity(contact, ctx).dot(contact.normal);
}

// ─── Apply a linear impulse along an arbitrary world-space vector ──────────
// Sign convention: positive vector pushes B in +direction, A in -direction.
function applyImpulseAlong(contact, ctx, axis, lambda) {
  const { bodyA, bodyB } = contact;
  const imp = axis.clone().multiplyScalar(lambda);

  bodyA.linearVelocity.addScaledVector(imp, -1 / bodyA.mass);
  addAngularImpulse(bodyA, ctx.rA, imp.clone().multiplyScalar(-1));

  if (bodyB) {
    bodyB.linearVelocity.addScaledVector(imp, 1 / bodyB.mass);
    addAngularImpulse(bodyB, ctx.rB, imp);
  }
}

export function applyContactImpulse(contact, ctx, lambda) {
  applyImpulseAlong(contact, ctx, contact.normal, lambda);
}

// Apply a tangential impulse expressed as a world-space vector (impulseVec is
// the full vector, not scalar × axis). A receives -impulseVec, B receives +impulseVec.
export function applyTangentialImpulse(contact, ctx, impulseVec) {
  const { bodyA, bodyB } = contact;
  bodyA.linearVelocity.addScaledVector(impulseVec, -1 / bodyA.mass);
  addAngularImpulse(bodyA, ctx.rA, impulseVec.clone().multiplyScalar(-1));
  if (bodyB) {
    bodyB.linearVelocity.addScaledVector(impulseVec, 1 / bodyB.mass);
    addAngularImpulse(bodyB, ctx.rB, impulseVec);
  }
}

// ─── Apply position correction along an arbitrary direction ────────────────
function applyPositionCorrectionAlong(contact, ctx, IA_inv, IB_inv, axis, dlambda) {
  const { bodyA, bodyB } = contact;
  const p = axis.clone().multiplyScalar(dlambda);

  bodyA.position.addScaledVector(p, -1 / bodyA.mass);
  const dThetaA = ctx.rA.clone().cross(p).applyMatrix3(IA_inv).multiplyScalar(-1);
  applyAngularDelta(bodyA, dThetaA);

  if (bodyB) {
    bodyB.position.addScaledVector(p, 1 / bodyB.mass);
    const dThetaB = ctx.rB.clone().cross(p).applyMatrix3(IB_inv);
    applyAngularDelta(bodyB, dThetaB);
  }
}

// Normal position correction (legacy public name).
export function applyContactPositionCorrection(contact, ctx, dlambda) {
  applyPositionCorrectionAlong(contact, ctx, ctx.IA_inv, ctx.IB_inv,
                               contact.normal, dlambda);
}

// Tangential position correction. `dpVec` is the full Δ-vector (world space)
// to APPLY to the contact-point displacement (i.e., we want body to move so
// that the relative tangential motion shrinks by `dpVec`).
// Returns the actual signed-length applied so caller can update λ_t.
function applyTangentialPositionCorrection(contact, ctx, dpVec) {
  const len = dpVec.length();
  if (len < 1e-12) return 0;
  const axis = dpVec.clone().multiplyScalar(1 / len);
  const t = effectiveMassAlong(contact.bodyA, ctx.rA, contact.bodyB, ctx.rB, axis);
  if (t.K < 1e-9) return 0;
  const dlambda = len / t.K;
  applyPositionCorrectionAlong(contact, ctx, t.IA_inv, t.IB_inv, axis, dlambda);
  return dlambda; // scalar magnitude along `axis`
}

// ─── Internal helpers ───────────────────────────────────────────────────────
function addAngularImpulse(body, rWorld, impulseWorld) {
  const torqueImpulseWorld = rWorld.clone().cross(impulseWorld);
  const R = body.getRotationMatrix(body.quaternion);
  const Rt = new THREE.Matrix3().copy(R).transpose();
  const torqueImpulseLocal = torqueImpulseWorld.applyMatrix3(Rt);
  const dOmegaLocal = torqueImpulseLocal.applyMatrix3(body.inertiaLocalInversed);
  body.angularVelocityLocal.add(dOmegaLocal);
}

function applyAngularDelta(body, dThetaWorld) {
  const q = body.quaternion;
  // dq = 0.5 * [dθ, 0] * q
  const dqx =  0.5 * ( dThetaWorld.x * q.w + dThetaWorld.y * q.z - dThetaWorld.z * q.y);
  const dqy =  0.5 * (-dThetaWorld.x * q.z + dThetaWorld.y * q.w + dThetaWorld.z * q.x);
  const dqz =  0.5 * ( dThetaWorld.x * q.y - dThetaWorld.y * q.x + dThetaWorld.z * q.w);
  const dqw =  0.5 * (-dThetaWorld.x * q.x - dThetaWorld.y * q.y - dThetaWorld.z * q.z);
  q.set(q.x + dqx, q.y + dqy, q.z + dqz, q.w + dqw);
  q.normalize();
}

// Rotate a local-space vector by a quaternion (world = q · v · q⁻¹).
function rotateVectorByQuat(v, q) {
  // Build R from q and apply (3×3 multiply is cheap and reuses existing code).
  const x = q.x, y = q.y, z = q.z, w = q.w;
  const xx = x*x, yy = y*y, zz = z*z;
  const xy = x*y, xz = x*z, yz = y*z;
  const wx = w*x, wy = w*y, wz = w*z;
  return new THREE.Vector3(
    (1 - 2*(yy + zz)) * v.x + 2*(xy - wz)     * v.y + 2*(xz + wy)     * v.z,
    2*(xy + wz)       * v.x + (1 - 2*(xx + zz)) * v.y + 2*(yz - wx)     * v.z,
    2*(xz - wy)       * v.x + 2*(yz + wx)     * v.y + (1 - 2*(xx + yy)) * v.z
  );
}

// ─── Pose integration helpers (no forces) ───────────────────────────────────
export function integratePosePredict(body, dt) {
  body.position.addScaledVector(body.linearVelocity, dt);
  const wLocal = body.angularVelocityLocal;
  const q = body.quaternion;
  const wx = wLocal.x, wy = wLocal.y, wz = wLocal.z;
  // q' = 0.5 * dt * wQuat * q   where wQuat = [wx, wy, wz, 0]
  // wQuat * q:
  const px =  wx * q.w + wy * q.z - wz * q.y;
  const py = -wx * q.z + wy * q.w + wz * q.x;
  const pz =  wx * q.y - wy * q.x + wz * q.w;
  const pw = -wx * q.x - wy * q.y - wz * q.z;
  q.set(q.x + 0.5 * dt * px,
        q.y + 0.5 * dt * py,
        q.z + 0.5 * dt * pz,
        q.w + 0.5 * dt * pw);
  q.normalize();
}

// Derive linear/angular velocity from position change after a substep.
// Used by XPBD: v = (x_new - x_prev)/h,  ω_world = 2*(q_new * q_prev⁻¹).xyz / h
export function setVelocityFromPoseDelta(body, prev, dt) {
  body.linearVelocity.copy(body.position).sub(prev.position).multiplyScalar(1 / dt);

  const qInv = prev.quaternion.clone().conjugate();
  const dq = body.quaternion.clone().multiply(qInv);
  const sign = dq.w >= 0 ? 1 : -1;
  const omegaWorld = new THREE.Vector3(dq.x, dq.y, dq.z).multiplyScalar(2 * sign / dt);
  body.setAngularVelocityGlobal(omegaWorld);
}

export function savePose(body) {
  return { position: body.position.clone(), quaternion: body.quaternion.clone() };
}

// ─── Apply linear/angular damping (multiplicative) ──────────────────────────
export function applyDamping(body, dt, damping) {
  if (damping > 0) {
    const k = 1 - damping * dt;
    body.linearVelocity.multiplyScalar(k);
    body.angularVelocityLocal.multiplyScalar(k);
  }
}

// ─── XPBD contact solver (with PBD-style friction) ──────────────────────────
// For each substep:
//   1) save prev pose, apply gravity, predict pose
//   2) re-detect contacts
//   3) Iterative position projection (positionIterations iters):
//      Each iteration, for each contact:
//        a) Recompute the *current* penetration from world-space anchor points
//           that were captured right after detection:
//             pA_now = pos_A + R(q_A)·rA_local
//             pB_now = pos_B + R(q_B)·rB_local  (or null for static)
//             gap    = (pA_now − pA_anchor) · n  − (pB_now − pB_anchor) · n
//             C      = penetration_init + gap   (≤ 0 means separated)
//           This is essential for multi-point manifolds: contact #1 reduces
//           penetration, contact #2 sees the smaller residual, and so on —
//           Gauss-Seidel convergence. Without it, N coplanar contacts each
//           "see" the full penetration and we get N× over-correction.
//        b) Apply normal correction dλ_n = C / K (clamped, λ_n ≥ 0).
//        c) Static-friction position sub-pass (PBDBodies §3.5).
//   4) Derive velocities from pose delta (substep-level).
//   5) Velocity pass: restitution gate + dynamic friction (§3.6).
//   6) Damping.
//
// detectContacts(bodies, planes) returns a fresh array of Contact objects.
// Stick threshold (m/s) — below this tangential speed we zero v_t completely
// to avoid micro-jitter in resting stacks. Configurable per-call via options.vStick.
const V_STICK_DEFAULT = 0.01;
let V_STICK = V_STICK_DEFAULT;

// Restitution gate (m/s) — below this *separating* normal speed we zero v_n.
// Resolves the "bodies never settle" issue in stacks: the position-projection
// pass pushes overlapping bodies apart by `penetration/h` per substep, which
// is mathematically equivalent to giving them a separating normal velocity.
// Even with restitution = 0, this small positive v_n re-opens a gap, gravity
// closes it, and the cycle repeats — the body floats. By snapping the
// post-correction v_n to zero when it is below ~2·|g|·h, we suppress this
// numerical bounce while still allowing genuine impacts to rebound (or, with
// restitution = 0, to dissipate normally through the velocity solver).
const V_REST_GATE_DEFAULT = 0.5;
let V_REST_GATE = V_REST_GATE_DEFAULT;

// Max linear position correction per single iteration (m). Protects against
// huge λ-spikes when penetration is enormous (a body teleported into another).
// Acts as Fix B — bounds dλ_n so the resulting linear displacement of the
// lighter body never exceeds POS_MAX_MOVE_PER_ITER.
const POS_MAX_MOVE_DEFAULT = 0.2;

export function xpbdSolveContacts(bodies, planes, detectContacts, dt, options = {}) {
  const substeps = options.substeps || 10;
  const positionIterations = options.positionIterations || 4;
  const posRelaxation = options.posRelaxation != null ? options.posRelaxation : 1.0;
  const posMaxMove = options.posMaxMove != null ? options.posMaxMove : POS_MAX_MOVE_DEFAULT;
  const gravity = options.gravity || new THREE.Vector3(0, -9.8, 0);
  const damping = options.damping || 0;
  const muS = options.muS != null ? options.muS : 0;
  const muD = options.muD != null ? options.muD : 0;
  V_STICK = options.vStick != null ? options.vStick : V_STICK_DEFAULT;
  V_REST_GATE = options.vRestGate != null ? options.vRestGate : V_REST_GATE_DEFAULT;
  const h = dt / substeps;
  const invH = 1 / h;
  const invH2 = 1 / (h * h);

  let totalContacts = 0;

  for (let s = 0; s < substeps; s++) {
    // 1. Apply gravity impulse + save prev poses + predict.
    // Attach prev pose directly on the body so contact-side code can fetch it
    // in O(1) (avoids bodies.indexOf in the friction pass — critical for n=1000).
    const prev = new Array(bodies.length);
    for (let i = 0; i < bodies.length; i++) {
      bodies[i].linearVelocity.addScaledVector(gravity, h);
      const p = savePose(bodies[i]);
      prev[i] = p;
      bodies[i]._prevPose = p;
      integratePosePredict(bodies[i], h);
    }

    // 2. Detect contacts in the predicted pose
    const contacts = detectContacts(bodies, planes);
    totalContacts += contacts.length;

    // ── Cache per-contact data: effectiveMass (rotation-dependent, but inertia
    // changes negligibly over one substep) and world anchor points at the
    // post-detection pose. Anchors are needed to re-evaluate penetration each
    // iteration (without them, N coplanar contacts would each correct the full
    // initial penetration → N× over-correction → bodies launch upward).
    const ctxs = new Array(contacts.length);
    const anchorsA = new Array(contacts.length);
    const anchorsB = new Array(contacts.length);
    for (let c = 0; c < contacts.length; c++) {
      const contact = contacts[c];
      ctxs[c] = effectiveMass(contact);
      anchorsA[c] = contact.bodyA.position.clone()
        .add(rotateVectorByQuat(contact.rA_local, contact.bodyA.quaternion));
      if (contact.bodyB) {
        anchorsB[c] = contact.bodyB.position.clone()
          .add(rotateVectorByQuat(contact.rB_local, contact.bodyB.quaternion));
      } else {
        anchorsB[c] = null;
      }
    }

    // 3. Iterative position projection (Fix A)
    for (let it = 0; it < positionIterations; it++) {
      for (let c = 0; c < contacts.length; c++) {
        const contact = contacts[c];
        const ctx = ctxs[c];
        if (ctx.K < 1e-9) continue;
        const bA = contact.bodyA, bB = contact.bodyB;

        // (a) Current penetration via world anchors.
        const pA_now = bA.position.clone()
          .add(rotateVectorByQuat(contact.rA_local, bA.quaternion));
        let gap;
        if (bB) {
          const pB_now = bB.position.clone()
            .add(rotateVectorByQuat(contact.rB_local, bB.quaternion));
          // gap = (Δpos_A − Δpos_B) · n   (= dA_in_-n + dB_in_+n with proper sign)
          gap = pA_now.sub(anchorsA[c]).sub(pB_now.sub(anchorsB[c])).dot(contact.normal);
        } else {
          gap = pA_now.sub(anchorsA[c]).dot(contact.normal);
        }
        // gap < 0 means bodies have moved apart along +normal (penetration shrunk).
        // C = remaining penetration.
        const C = contact.penetration + gap;
        if (C <= 1e-9) continue;

        // (b) Normal correction with relaxation + per-iter clamp (Fix B).
        let dlambda = (C / ctx.K) * posRelaxation;
        if (contact.lambdaAcc + dlambda < 0) dlambda = -contact.lambdaAcc;
        // Cap linear displacement of the lighter body to posMaxMove.
        const minMass = bB ? Math.min(bA.mass, bB.mass) : bA.mass;
        const maxLambdaByMove = posMaxMove * minMass;
        if (dlambda > maxLambdaByMove) dlambda = maxLambdaByMove;
        contact.lambdaAcc += dlambda;
        applyContactPositionCorrection(contact, ctx, dlambda);

        // (c) Static friction position sub-pass (PBDBodies §3.5)
        if (muS > 0 && contact.lambdaAcc > 0) {
          const prevA = bA._prevPose;
          const pA_w = bA.position.clone().add(rotateVectorByQuat(contact.rA_local, bA.quaternion));
          const pA_p = prevA.position.clone()
                        .add(rotateVectorByQuat(contact.rA_local, prevA.quaternion));
          const dpA = pA_w.sub(pA_p);

          let dp;
          if (bB) {
            const prevB = bB._prevPose;
            const pB_w = bB.position.clone().add(rotateVectorByQuat(contact.rB_local, bB.quaternion));
            const pB_p = prevB.position.clone()
                          .add(rotateVectorByQuat(contact.rB_local, prevB.quaternion));
            const dpB = pB_w.sub(pB_p);
            dp = dpA.sub(dpB);
          } else {
            dp = dpA;
          }
          const dn = dp.dot(contact.normal);
          const dp_t = dp.sub(contact.normal.clone().multiplyScalar(dn));
          const lenT = dp_t.length();
          if (lenT > 1e-9) {
            const axisT = dp_t.clone().multiplyScalar(1 / lenT);
            const tCtx = effectiveMassAlong(bA, ctx.rA, bB, ctx.rB, axisT);
            if (tCtx.K > 1e-9) {
              const dlT = lenT / tCtx.K;
              const cone = muS * contact.lambdaAcc;
              const tentative = contact.lambdaTAcc.clone().addScaledVector(axisT, dlT);
              if (tentative.length() <= cone) {
                applyPositionCorrectionAlong(
                  contact, ctx, tCtx.IA_inv, tCtx.IB_inv,
                  axisT, -dlT
                );
                contact.lambdaTAcc.copy(tentative);
              }
            }
          }
        }
      }
    }

    // 5. Derive velocity from pose change
    for (let i = 0; i < bodies.length; i++) {
      setVelocityFromPoseDelta(bodies[i], prev[i], h);
    }

    // 6. Velocity pass: restitution gate + dynamic friction (PBDBodies §3.6)
    //    For each active contact (λ_n > 0):
    //      (a) Restitution gate: if 0 < v_n < V_REST_GATE, apply normal impulse
    //          J_n = -m_eff·v_n along contact.normal to snap separation to zero.
    //          v_n here is v_rel·normal; normal points A→B, so positive v_n
    //          means B is moving away from A (separating). Negative v_n means
    //          approaching — left alone (no restitution) so impact damping
    //          comes from the next position pass.
    //      (b) Tangential dynamic friction: cap |Δv_t| by Coulomb h·μ·|f_n|;
    //          if |v_t| < V_STICK, zero v_t completely.
    for (let c = 0; c < contacts.length; c++) {
      const contact = contacts[c];
      if (contact.lambdaAcc <= 0) continue;
      const ctx = effectiveMass(contact);
      if (ctx.K < 1e-9) continue;

      // (a) Normal restitution gate — zero out small *separating* normal vel.
      if (V_REST_GATE > 0) {
        const vRelN = relativeVelocity(contact, ctx);
        const vnN = vRelN.dot(contact.normal);
        if (vnN > 0 && vnN < V_REST_GATE) {
          applyContactImpulse(contact, ctx, -ctx.m_eff * vnN);
        }
      }

      if (muD > 0 || V_STICK > 0) {
        const vRel = relativeVelocity(contact, ctx);
        const vn = vRel.dot(contact.normal);
        const vt = vRel.sub(contact.normal.clone().multiplyScalar(vn));
        const vtLen = vt.length();
        if (vtLen < 1e-9) continue;

        // Stick: snap tangential velocity to zero when very small.
        if (vtLen < V_STICK) {
          const axisT = vt.multiplyScalar(1 / vtLen);
          const tCtx = effectiveMassAlong(contact.bodyA, ctx.rA, contact.bodyB, ctx.rB, axisT);
          if (tCtx.K < 1e-9) continue;
          const J = axisT.multiplyScalar(-tCtx.m_eff * vtLen);
          applyTangentialImpulse(contact, ctx, J);
          continue;
        }

        if (muD <= 0) continue;

        const fn = contact.lambdaAcc * invH2;         // normal force magnitude
        const maxDvMag = h * muD * fn;                // max |Δv| from Coulomb cone
        const dvMag = Math.min(maxDvMag, vtLen);
        const axisT = vt.multiplyScalar(1 / vtLen);   // unit tangential direction
        const tCtx = effectiveMassAlong(contact.bodyA, ctx.rA, contact.bodyB, ctx.rB, axisT);
        if (tCtx.K < 1e-9) continue;
        // Apply impulse J_t = m_eff_t · (−axisT · dvMag).
        const J = axisT.multiplyScalar(-tCtx.m_eff * dvMag);
        applyTangentialImpulse(contact, ctx, J);
      }
    }

    // 7. Damping
    for (let i = 0; i < bodies.length; i++) {
      applyDamping(bodies[i], h, damping);
    }
  }

  return { totalContacts };
}

// ─── SI contact solver (with box-friction) ─────────────────────────────────
// One step of dt:
//   1) apply gravity impulse to all linear velocities
//   2) detect contacts (in the pre-integrate pose)
//   3) velocity solver: N iterations
//        a) normal: λ_step = m_eff · (−v·n);  λ_n_acc ≥ 0
//        b) for each tangent t1, t2: box-friction
//             λ_t_step = m_eff_t · (−v·t)
//             clamp λ_t_acc + λ_t_step to ±μ·λ_n_acc
//   4) integrate pose
//   5) NGS position projection
//   6) damping
export function siSolveContacts(bodies, planes, detectContacts, dt, options = {}) {
  const velIterations = options.velIterations || 8;
  const posIterations = options.posIterations || 4;
  const slop = options.slop || 0.005;
  const beta = options.beta || 0.2;
  const gravity = options.gravity || new THREE.Vector3(0, -9.8, 0);
  const damping = options.damping || 0;
  const mu = options.muS != null ? options.muS : 0; // SI uses a single μ

  // 1. Apply gravity to linear velocities
  for (let i = 0; i < bodies.length; i++) {
    bodies[i].linearVelocity.addScaledVector(gravity, dt);
  }

  // 2. Detect contacts at current pose
  const contacts = detectContacts(bodies, planes);

  // Precompute per-contact ctx and tangent basis. Stored aligned with contacts[].
  const ctxs = new Array(contacts.length);
  const tangents = new Array(contacts.length);  // {t1, t2, K1, K2, lambdaT1, lambdaT2}
  for (let i = 0; i < contacts.length; i++) {
    ctxs[i] = effectiveMass(contacts[i]);
    const n = contacts[i].normal;
    const t1 = buildTangentBasis(n);
    const t2 = n.clone().cross(t1).normalize();
    const K1 = effectiveMassAlong(contacts[i].bodyA, ctxs[i].rA, contacts[i].bodyB, ctxs[i].rB, t1);
    const K2 = effectiveMassAlong(contacts[i].bodyA, ctxs[i].rA, contacts[i].bodyB, ctxs[i].rB, t2);
    tangents[i] = { t1, t2, K1, K2, lambdaT1: 0, lambdaT2: 0 };
  }

  // 3. Velocity solver
  for (let it = 0; it < velIterations; it++) {
    for (let i = 0; i < contacts.length; i++) {
      const contact = contacts[i];
      const ctx = ctxs[i];
      if (ctx.K < 1e-9) continue;

      // (a) Normal
      const vrel0 = relativeVelocity(contact, ctx);
      const vn = vrel0.dot(contact.normal);
      let lambda = -ctx.m_eff * vn;
      const newAcc = Math.max(0, contact.lambdaAcc + lambda);
      lambda = newAcc - contact.lambdaAcc;
      contact.lambdaAcc = newAcc;
      applyContactImpulse(contact, ctx, lambda);

      // (b) Friction along t1 and t2 (box-friction)
      if (mu > 0) {
        const cone = mu * contact.lambdaAcc;
        const tng = tangents[i];

        // t1
        const vrel1 = relativeVelocity(contact, ctx);
        const vt1 = vrel1.dot(tng.t1);
        let l1 = -tng.K1.m_eff * vt1;
        const new1 = clampTo(tng.lambdaT1 + l1, cone);
        l1 = new1 - tng.lambdaT1;
        tng.lambdaT1 = new1;
        applyTangentialImpulse(contact, ctx, tng.t1.clone().multiplyScalar(l1));

        // t2
        const vrel2 = relativeVelocity(contact, ctx);
        const vt2 = vrel2.dot(tng.t2);
        let l2 = -tng.K2.m_eff * vt2;
        const new2 = clampTo(tng.lambdaT2 + l2, cone);
        l2 = new2 - tng.lambdaT2;
        tng.lambdaT2 = new2;
        applyTangentialImpulse(contact, ctx, tng.t2.clone().multiplyScalar(l2));
      }
    }
  }

  // 4. Integrate pose
  for (let i = 0; i < bodies.length; i++) {
    integratePosePredict(bodies[i], dt);
  }

  // 5. NGS position projection. Recompute contacts each pass.
  for (let it = 0; it < posIterations; it++) {
    const posContacts = detectContacts(bodies, planes);
    for (let i = 0; i < posContacts.length; i++) {
      const contact = posContacts[i];
      const ctx = effectiveMass(contact);
      if (ctx.K < 1e-9) continue;
      const correction = Math.max(0, contact.penetration - slop);
      if (correction <= 0) continue;
      const dlambda = ctx.m_eff * beta * correction;
      applyContactPositionCorrection(contact, ctx, dlambda);
    }
  }

  // 6. Damping
  for (let i = 0; i < bodies.length; i++) {
    applyDamping(bodies[i], dt, damping);
  }

  return { totalContacts: contacts.length };
}

// ─── Tangent basis: build a unit vector orthogonal to n ────────────────────
export function buildTangentBasis(n) {
  // Pick the world axis least aligned with n, then Gram–Schmidt.
  if (Math.abs(n.x) < 0.5) {
    return new THREE.Vector3(1, 0, 0)
      .sub(n.clone().multiplyScalar(n.x))
      .normalize();
  } else {
    return new THREE.Vector3(0, 1, 0)
      .sub(n.clone().multiplyScalar(n.y))
      .normalize();
  }
}

function clampTo(x, lim) {
  if (x >  lim) return  lim;
  if (x < -lim) return -lim;
  return x;
}
