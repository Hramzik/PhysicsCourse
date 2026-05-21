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

// ─── Box vs Box via SAT ─────────────────────────────────────────────────────
// 15 axes (3 + 3 + 9). Returns at most one contact with the minimum-overlap
// axis as the separating direction.

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

  // Build 15 candidate axes.
  const axes = [];
  axes.push(axesA[0], axesA[1], axesA[2]);
  axes.push(axesB[0], axesB[1], axesB[2]);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      const c = new THREE.Vector3().crossVectors(axesA[i], axesB[j]);
      if (c.lengthSq() > 1e-8) {
        c.normalize();
        axes.push(c);
      }
    }
  }

  let minOverlap = Infinity;
  let bestAxis = null;
  for (let i = 0; i < axes.length; i++) {
    const ax = axes[i];
    const rA = Math.abs(axesA[0].dot(ax)) * halfA[0]
             + Math.abs(axesA[1].dot(ax)) * halfA[1]
             + Math.abs(axesA[2].dot(ax)) * halfA[2];
    const rB = Math.abs(axesB[0].dot(ax)) * halfB[0]
             + Math.abs(axesB[1].dot(ax)) * halfB[1]
             + Math.abs(axesB[2].dot(ax)) * halfB[2];
    const distOnAxis = Math.abs(t.dot(ax));
    const overlap = (rA + rB) - distOnAxis;
    if (overlap < 0) return; // separating axis found
    if (overlap < minOverlap) {
      minOverlap = overlap;
      bestAxis = ax;
    }
  }
  if (!bestAxis) return;

  // Orient normal from A to B
  const normal = bestAxis.clone();
  if (normal.dot(t) < 0) normal.multiplyScalar(-1);

  // Contact point: incident vertex on B (deepest in the direction of -normal).
  // Pick the vertex of B that is most along (-normal) from its center, i.e.
  // furthest along axis pointing from B into A.
  const vertsB = getBoxVertices(bodyB);
  let deepest = vertsB[0];
  let deepestDot = deepest.dot(normal);
  for (let i = 1; i < 8; i++) {
    const d = vertsB[i].dot(normal);
    if (d < deepestDot) { deepestDot = d; deepest = vertsB[i]; }
  }

  out.push({
    bodyA,
    bodyB,
    point: deepest.clone(),
    rA_local: worldPointToLocal(bodyA, deepest),
    rB_local: worldPointToLocal(bodyB, deepest),
    normal,
    penetration: minOverlap,
    lambdaAcc: 0,
    lambdaTAcc: new THREE.Vector3()
  });
}
