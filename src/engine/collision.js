// Collision detection for axis-aligned static planes vs OBB (box) and OBB-OBB.
//
// All operations return a Contact:
//   { bodyA, bodyB, point,
//     rA_local, rB_local,   // contact point in each body's local frame
//                           //   (rB_local is null for static contacts)
//     normal,               // unit, points A→B (or body→solid for static)
//     penetration,          // > 0 means overlap
//     lambdaAcc,            // normal Lagrange accumulator
//     lambdaTAcc,           // tangential accumulator as Vector3 (world space) }
//
// Convention reminder: applyContactImpulse(λ>0) pushes A in -normal and B in +normal,
// i.e. it separates the pair (or for static: pushes A out of the solid).
//
// rA_local / rB_local are used by the friction sub-passes which need to
// reconstruct the contact point at substep-start (using prev pose) and at the
// current pose to compute Δp = (p_A − p_A_prev) − (p_B − p_B_prev).

// ─── Box helpers ─────────────────────────────────────────────────────────────

// Returns 8 world-space vertices of an OBB.
export function getBoxVertices(body, out) {
  const [sx, sy, sz] = body.size;
  const hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;
  const R = body.getRotationMatrix(body.quaternion);
  const p = body.position;
  const e = R.elements;
  // R columns are local axes in world.
  const ax = e[0], ay = e[1], az = e[2];
  const bx = e[3], by = e[4], bz = e[5];
  const cx = e[6], cy = e[7], cz = e[8];

  if (!out) out = new Array(8);
  let k = 0;
  for (let i = 0; i < 8; i++) {
    const sxg = (i & 1) ? hx : -hx;
    const syg = (i & 2) ? hy : -hy;
    const szg = (i & 4) ? hz : -hz;
    const vx = p.x + ax * sxg + bx * syg + cx * szg;
    const vy = p.y + ay * sxg + by * syg + cy * szg;
    const vz = p.z + az * sxg + bz * syg + cz * szg;
    out[k++] = new THREE.Vector3(vx, vy, vz);
  }
  return out;
}

// AABB (world) for an OBB.
export function getBoxAABB(body) {
  const [sx, sy, sz] = body.size;
  const hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;
  const R = body.getRotationMatrix(body.quaternion);
  const e = R.elements;
  // |R| * half-extents
  const rx = Math.abs(e[0]) * hx + Math.abs(e[3]) * hy + Math.abs(e[6]) * hz;
  const ry = Math.abs(e[1]) * hx + Math.abs(e[4]) * hy + Math.abs(e[7]) * hz;
  const rz = Math.abs(e[2]) * hx + Math.abs(e[5]) * hy + Math.abs(e[8]) * hz;
  const p = body.position;
  return { min: [p.x - rx, p.y - ry, p.z - rz], max: [p.x + rx, p.y + ry, p.z + rz] };
}

export function aabbOverlap(a, b) {
  return a.max[0] >= b.min[0] && a.min[0] <= b.max[0] &&
         a.max[1] >= b.min[1] && a.min[1] <= b.max[1] &&
         a.max[2] >= b.min[2] && a.min[2] <= b.max[2];
}

// Express a world-space point in a body's local frame: r_local = Rᵀ · (p - x).
function worldPointToLocal(body, worldPoint) {
  const R = body.getRotationMatrix(body.quaternion);
  const Rt = new THREE.Matrix3().copy(R).transpose();
  return worldPoint.clone().sub(body.position).applyMatrix3(Rt);
}

// ─── Box vs Static Plane ────────────────────────────────────────────────────
// Plane: { point: Vector3, normal: Vector3 } (normal unit, points out of solid).
// Generates up to 4 contacts (vertices below the plane).
//
// Contact normal convention (matches A→B for OBB-OBB):
// the normal points FROM the body TOWARDS the solid, i.e., it is the direction
// in which body A is being pushed by the contact constraint *into* the wall.
// We store -plane.normal so that applyContactImpulse(λ>0) ⇒ body receives
// −λ·normal = +λ·plane.normal, pushing it OUT of the solid.
export function collideBoxPlane(body, plane, out) {
  const verts = getBoxVertices(body);
  const n = plane.normal;
  const px = plane.point.x, py = plane.point.y, pz = plane.point.z;
  for (let i = 0; i < 8; i++) {
    const v = verts[i];
    const d = (v.x - px) * n.x + (v.y - py) * n.y + (v.z - pz) * n.z;
    if (d < 0) {
      const contactNormal = n.clone().multiplyScalar(-1);  // from body INTO solid
      out.push({
        bodyA: body,
        bodyB: null,
        point: v.clone(),
        rA_local: worldPointToLocal(body, v),
        rB_local: null,
        normal: contactNormal,
        penetration: -d,
        lambdaAcc: 0,
        lambdaTAcc: new THREE.Vector3()
      });
    }
  }
}

// ─── Box vs Box via SAT + face clipping ─────────────────────────────────────
// 15 axes (3 + 3 + 9). For face-axis contacts, generates up to 4 contact
// points via Sutherland–Hodgman clipping of the incident face polygon against
// the side planes of the reference face — necessary for stable stacking
// (a single contact point creates a parasitic angular moment).
//
// For edge-edge axes a single contact point is generated (deepest incident
// vertex), which is geometrically correct for sharp-edge collisions.

export function collideBoxBox(bodyA, bodyB, out) {
  const RA = bodyA.getRotationMatrix(bodyA.quaternion);
  const RB = bodyB.getRotationMatrix(bodyB.quaternion);
  const ea = RA.elements, eb = RB.elements;

  const axesA = [
    new THREE.Vector3(ea[0], ea[1], ea[2]),
    new THREE.Vector3(ea[3], ea[4], ea[5]),
    new THREE.Vector3(ea[6], ea[7], ea[8])
  ];
  const axesB = [
    new THREE.Vector3(eb[0], eb[1], eb[2]),
    new THREE.Vector3(eb[3], eb[4], eb[5]),
    new THREE.Vector3(eb[6], eb[7], eb[8])
  ];

  const halfA = [bodyA.size[0] * 0.5, bodyA.size[1] * 0.5, bodyA.size[2] * 0.5];
  const halfB = [bodyB.size[0] * 0.5, bodyB.size[1] * 0.5, bodyB.size[2] * 0.5];

  const t = new THREE.Vector3().subVectors(bodyB.position, bodyA.position);

  // Build candidate axes with type tags:
  //   type 0 = face of A, 1 = face of B, 2 = edge-edge (cross axisA[i] × axisB[j])
  //   idx  = axis index within that group (0..2 for face, 0..8 for edge-edge)
  const axes = [];
  axes.push({ v: axesA[0], type: 0, idx: 0 });
  axes.push({ v: axesA[1], type: 0, idx: 1 });
  axes.push({ v: axesA[2], type: 0, idx: 2 });
  axes.push({ v: axesB[0], type: 1, idx: 0 });
  axes.push({ v: axesB[1], type: 1, idx: 1 });
  axes.push({ v: axesB[2], type: 1, idx: 2 });
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      const c = new THREE.Vector3().crossVectors(axesA[i], axesB[j]);
      if (c.lengthSq() > 1e-8) {
        c.normalize();
        axes.push({ v: c, type: 2, idx: i * 3 + j });
      }
    }
  }

  // Face-axis preference bias (PBDBodies / Box2D trick): we slightly prefer
  // face axes over edge-edge axes when overlaps are close, because face axes
  // give a much more stable contact manifold and edge-edge cross axes are
  // numerically noisy. Without this, in stacks a near-tie can flip between a
  // face and an edge axis between substeps and produce jitter.
  const FACE_BIAS = 1e-3;

  let minOverlap = Infinity;
  let best = null;
  for (let i = 0; i < axes.length; i++) {
    const ax = axes[i].v;
    const rA = Math.abs(axesA[0].dot(ax)) * halfA[0]
             + Math.abs(axesA[1].dot(ax)) * halfA[1]
             + Math.abs(axesA[2].dot(ax)) * halfA[2];
    const rB = Math.abs(axesB[0].dot(ax)) * halfB[0]
             + Math.abs(axesB[1].dot(ax)) * halfB[1]
             + Math.abs(axesB[2].dot(ax)) * halfB[2];
    const distOnAxis = Math.abs(t.dot(ax));
    const overlap = (rA + rB) - distOnAxis;
    if (overlap < 0) return; // separating axis found
    const effOverlap = overlap + (axes[i].type === 2 ? FACE_BIAS : 0);
    if (effOverlap < minOverlap) {
      minOverlap = effOverlap;
      best = axes[i];
    }
  }
  if (!best) return;

  // Orient normal from A to B
  const normal = best.v.clone();
  if (normal.dot(t) < 0) normal.multiplyScalar(-1);

  // Recover the true (non-biased) overlap as the penetration scalar.
  const rA0 = Math.abs(axesA[0].dot(normal)) * halfA[0]
           + Math.abs(axesA[1].dot(normal)) * halfA[1]
           + Math.abs(axesA[2].dot(normal)) * halfA[2];
  const rB0 = Math.abs(axesB[0].dot(normal)) * halfB[0]
           + Math.abs(axesB[1].dot(normal)) * halfB[1]
           + Math.abs(axesB[2].dot(normal)) * halfB[2];
  const truePen = (rA0 + rB0) - Math.abs(t.dot(normal));

  // ── Edge-edge case: single contact point ──────────────────────────────
  if (best.type === 2) {
    const vertsB = getBoxVertices(bodyB);
    let deepest = vertsB[0];
    let deepestDot = deepest.dot(normal);
    for (let i = 1; i < 8; i++) {
      const d = vertsB[i].dot(normal);
      if (d < deepestDot) { deepestDot = d; deepest = vertsB[i]; }
    }
    out.push({
      bodyA, bodyB,
      point: deepest.clone(),
      rA_local: worldPointToLocal(bodyA, deepest),
      rB_local: worldPointToLocal(bodyB, deepest),
      normal,
      penetration: truePen,
      lambdaAcc: 0,
      lambdaTAcc: new THREE.Vector3()
    });
    return;
  }

  // ── Face-axis case: face clipping for up to 4 contact points ──────────
  // Reference body owns the face whose axis is `best`; incident body provides
  // the polygon being clipped. Convention: refNormal points OUTWARD from the
  // reference body, INTO the incident body.
  let refBody, incBody, refAxes, incAxes, refHalfs, incHalfs, refAxisIdx;
  let refNormal;
  if (best.type === 0) {
    refBody = bodyA; incBody = bodyB;
    refAxes = axesA; incAxes = axesB;
    refHalfs = halfA; incHalfs = halfB;
    refAxisIdx = best.idx;
    refNormal = normal.clone();          // normal A→B  ≡ outward from A
  } else {
    refBody = bodyB; incBody = bodyA;
    refAxes = axesB; incAxes = axesA;
    refHalfs = halfB; incHalfs = halfA;
    refAxisIdx = best.idx;
    refNormal = normal.clone().negate(); // outward from B  ≡ -normal
  }

  // Sign of refNormal along refAxes[refAxisIdx]  (±1)
  const refAxisVec = refAxes[refAxisIdx];
  const sign = refAxisVec.dot(refNormal) >= 0 ? 1 : -1;

  // Reference face center & two in-face axes (and half-extents)
  const refFaceCenter = refBody.position.clone()
    .addScaledVector(refAxisVec, sign * refHalfs[refAxisIdx]);
  const i1 = (refAxisIdx + 1) % 3;
  const i2 = (refAxisIdx + 2) % 3;
  const refU = refAxes[i1], refV = refAxes[i2];
  const refHU = refHalfs[i1], refHV = refHalfs[i2];

  // Incident face: face on incBody whose outward normal is most anti-parallel
  // to refNormal (i.e. most aligned with -refNormal).
  let bestInc = -Infinity;
  let incAxisIdx = 0, incSign = 1;
  for (let k = 0; k < 3; k++) {
    const d = incAxes[k].dot(refNormal); // we want d ≈ -1 ideally
    if (-d > bestInc) { bestInc = -d; incAxisIdx = k; incSign = -1; }
    if ( d > bestInc) { bestInc =  d; incAxisIdx = k; incSign =  1; }
  }
  // incSign here = sign of inc face's outward normal along incAxes[incAxisIdx];
  // outward = incSign * incAxes[incAxisIdx], and we want this most anti-parallel
  // to refNormal, so flip the sign convention: outward direction is the one
  // that minimizes dot(outward, refNormal).
  // Recompute cleanly:
  {
    let minDot = Infinity;
    for (let k = 0; k < 3; k++) {
      const dp = incAxes[k].dot(refNormal);
      if (dp < minDot)  { minDot = dp;  incAxisIdx = k; incSign =  1; }
      if (-dp < minDot) { minDot = -dp; incAxisIdx = k; incSign = -1; }
    }
  }

  const incFaceCenter = incBody.position.clone()
    .addScaledVector(incAxes[incAxisIdx], incSign * incHalfs[incAxisIdx]);
  const j1 = (incAxisIdx + 1) % 3;
  const j2 = (incAxisIdx + 2) % 3;
  const incU = incAxes[j1], incV = incAxes[j2];
  const incHU = incHalfs[j1], incHV = incHalfs[j2];

  // 4 vertices of incident face (world space)
  let poly = [
    incFaceCenter.clone().addScaledVector(incU,  incHU).addScaledVector(incV,  incHV),
    incFaceCenter.clone().addScaledVector(incU,  incHU).addScaledVector(incV, -incHV),
    incFaceCenter.clone().addScaledVector(incU, -incHU).addScaledVector(incV, -incHV),
    incFaceCenter.clone().addScaledVector(incU, -incHU).addScaledVector(incV,  incHV),
  ];

  // Clip by the 4 side planes of the reference face (Sutherland–Hodgman).
  poly = clipPolyHalfspace(poly, refFaceCenter, refU,  1, refHU);
  poly = clipPolyHalfspace(poly, refFaceCenter, refU, -1, refHU);
  poly = clipPolyHalfspace(poly, refFaceCenter, refV,  1, refHV);
  poly = clipPolyHalfspace(poly, refFaceCenter, refV, -1, refHV);

  // Keep only points below the reference plane (penetrating).
  // sd = (p − refFaceCenter)·refNormal  (positive = outside refBody, negative = inside)
  // pen = -sd
  const surviving = [];
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const sd = p.clone().sub(refFaceCenter).dot(refNormal);
    if (sd < 0) surviving.push({ point: p, pen: -sd });
  }
  if (surviving.length === 0) {
    // Fallback: degenerate clip (shouldn't happen if SAT said overlap > 0).
    // Emit a single contact at the deepest incident vertex.
    let deepest = poly[0] || incFaceCenter;
    let deepestSd = Infinity;
    for (let i = 0; i < poly.length; i++) {
      const sd = poly[i].clone().sub(refFaceCenter).dot(refNormal);
      if (sd < deepestSd) { deepestSd = sd; deepest = poly[i]; }
    }
    if (deepestSd < 0) surviving.push({ point: deepest, pen: -deepestSd });
  }

  // Reduce to ≤ 4 contact points: keep the 4 with the largest penetration
  // (these contribute most to the position correction; clipping a quad by 4
  // half-planes gives ≤ 8 vertices).
  if (surviving.length > 4) {
    surviving.sort((a, b) => b.pen - a.pen);
    surviving.length = 4;
  }

  for (let i = 0; i < surviving.length; i++) {
    const p = surviving[i].point;
    out.push({
      bodyA, bodyB,
      point: p.clone(),
      rA_local: worldPointToLocal(bodyA, p),
      rB_local: worldPointToLocal(bodyB, p),
      normal: normal.clone(),
      penetration: surviving[i].pen,
      lambdaAcc: 0,
      lambdaTAcc: new THREE.Vector3()
    });
  }
}

// Sutherland–Hodgman clip of a convex polygon against a single half-space.
// The half-space is { p : sign * (p − planePoint) · axis  ≤ halfExtent }.
// Returns a new array of THREE.Vector3 (input is left unchanged).
function clipPolyHalfspace(poly, planePoint, axis, sign, halfExtent) {
  const out = [];
  const n = poly.length;
  if (n === 0) return out;
  for (let i = 0; i < n; i++) {
    const p1 = poly[i];
    const p2 = poly[(i + 1) % n];
    const d1 = sign * (p1.x - planePoint.x) * axis.x
             + sign * (p1.y - planePoint.y) * axis.y
             + sign * (p1.z - planePoint.z) * axis.z - halfExtent;
    const d2 = sign * (p2.x - planePoint.x) * axis.x
             + sign * (p2.y - planePoint.y) * axis.y
             + sign * (p2.z - planePoint.z) * axis.z - halfExtent;
    const inside1 = d1 <= 0;
    const inside2 = d2 <= 0;
    if (inside1) out.push(p1);
    if (inside1 !== inside2) {
      const t = d1 / (d1 - d2);
      out.push(new THREE.Vector3(
        p1.x + (p2.x - p1.x) * t,
        p1.y + (p2.y - p1.y) * t,
        p1.z + (p2.z - p1.z) * t
      ));
    }
  }
  return out;
}
