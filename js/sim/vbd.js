import * as THREE from '../../vendor/three/three.module.min.js';
import { DistanceConstraint, VolumeConstraint } from './constraints.js';

function clamp(x, a, b) {
  return Math.max(a, Math.min(b, x));
}

function stiffnessFromCompliance(compliance) {
  const c = Number(compliance);
  if (!Number.isFinite(c)) return 1;
  if (c <= 0) return 1e6;
  return clamp(1 / c, 1, 1e6);
}

function addOuter(H, v, s) {
  // H += s * (v v^T)
  const x = v.x;
  const y = v.y;
  const z = v.z;
  H[0] += s * x * x;
  H[1] += s * x * y;
  H[2] += s * x * z;
  H[3] += s * y * x;
  H[4] += s * y * y;
  H[5] += s * y * z;
  H[6] += s * z * x;
  H[7] += s * z * y;
  H[8] += s * z * z;
}

function solve3x3(H, b, out) {
  // Solve H x = b (3x3). Uses Gaussian elimination with partial pivoting.
  // H is row-major length 9. b/out are THREE.Vector3.
  const a00 = H[0], a01 = H[1], a02 = H[2];
  const a10 = H[3], a11 = H[4], a12 = H[5];
  const a20 = H[6], a21 = H[7], a22 = H[8];

  const m = [
    [a00, a01, a02, b.x],
    [a10, a11, a12, b.y],
    [a20, a21, a22, b.z],
  ];

  for (let col = 0; col < 3; col++) {
    // Pivot
    let pivotRow = col;
    let pivotAbs = Math.abs(m[col][col]);
    for (let r = col + 1; r < 3; r++) {
      const v = Math.abs(m[r][col]);
      if (v > pivotAbs) {
        pivotAbs = v;
        pivotRow = r;
      }
    }
    if (pivotAbs < 1e-12) return false;
    if (pivotRow !== col) {
      const tmp = m[col];
      m[col] = m[pivotRow];
      m[pivotRow] = tmp;
    }

    const piv = m[col][col];
    for (let c = col; c < 4; c++) m[col][c] /= piv;

    for (let r = 0; r < 3; r++) {
      if (r === col) continue;
      const f = m[r][col];
      if (Math.abs(f) < 1e-20) continue;
      for (let c = col; c < 4; c++) m[r][c] -= f * m[col][c];
    }
  }

  out.set(m[0][3], m[1][3], m[2][3]);
  return true;
}

function computeVolumeGradient(particles, triangles) {
  // Matches VolumeConstraint.project() gradient, but returns grads array.
  const grads = new Array(particles.length);
  for (let i = 0; i < grads.length; i++) grads[i] = new THREE.Vector3(0, 0, 0);

  const tmp = new THREE.Vector3();
  for (let t = 0; t < triangles.length; t++) {
    const [i0, i1, i2] = triangles[t];
    const x0 = particles[i0].x;
    const x1 = particles[i1].x;
    const x2 = particles[i2].x;

    tmp.crossVectors(x1, x2).multiplyScalar(1 / 6);
    grads[i0].add(tmp);

    tmp.crossVectors(x2, x0).multiplyScalar(1 / 6);
    grads[i1].add(tmp);

    tmp.crossVectors(x0, x1).multiplyScalar(1 / 6);
    grads[i2].add(tmp);
  }

  return grads;
}

function edgeKey(i, j) {
  return i < j ? i + ',' + j : j + ',' + i;
}

export function createVBDContext(group, dt, settings) {
  const particles = group.particles;
  const n = particles.length;

  const distanceConstraints = [];
  const volumeConstraints = [];

  for (const c of group.constraints) {
    if (c instanceof DistanceConstraint) distanceConstraints.push(c);
    else if (c instanceof VolumeConstraint) volumeConstraints.push(c);
  }

  const incident = Array.from({ length: n }, () => []);
  for (const c of distanceConstraints) {
    incident[c.i0].push(c);
    incident[c.i1].push(c);
  }

  // Save y = predicted position (implicit Euler target) once per time step.
  const y = new Float64Array(n * 3);
  for (let i = 0; i < n; i++) {
    const x = particles[i].x;
    y[i * 3 + 0] = x.x;
    y[i * 3 + 1] = x.y;
    y[i * 3 + 2] = x.z;
  }

  return { n, incident, distanceConstraints, volumeConstraints, y, dt, settingsSnapshot: null };
}

export function vbdIteration(group, dt, settings, ctx) {
  const particles = group.particles;
  const n = ctx.n;

  // Collision stiffness: tuned relative to inertia to be noticeable with few iterations.
  const mOverH2Ref = 1 / (Math.max(1e-8, particles.find(p => p.invMass !== 0)?.invMass ?? 1) * dt * dt);
  const kFloor = clamp(30 * mOverH2Ref, 1e3, 2e5);
  const kSelf = clamp(15 * mOverH2Ref, 1e3, 2e5);

  // Volume terms (compute once per group per iteration)
  const volumeTerms = [];
  for (const vc of ctx.volumeConstraints) {
    const triangles = vc.triangles;
    const V = VolumeConstraint.computeVolume(particles, triangles);
    const C = V - vc.restVolume;
    const grads = computeVolumeGradient(particles, triangles);

    const comp = typeof vc._getCompliance === 'function' ? vc._getCompliance(settings) : settings.complianceVolume;
    const k = stiffnessFromCompliance(comp);
    volumeTerms.push({ C, grads, k });
  }

  const neighborEdges = group.neighborEdges || null;

  const grad = new THREE.Vector3();
  const rhs = new THREE.Vector3();
  const dx = new THREE.Vector3();
  const nrm = new THREE.Vector3();

  for (let i = 0; i < n; i++) {
    const p = particles[i];
    if (p.invMass === 0) continue;

    const miOverH2 = 1 / (p.invMass * dt * dt);

    // Local gradient g and Hessian H for vertex i
    grad.set(0, 0, 0);
    const H = [
      miOverH2, 0, 0,
      0, miOverH2, 0,
      0, 0, miOverH2,
    ];

    // Inertia term: (m/2h^2) ||x - y||^2
    const yi = new THREE.Vector3(ctx.y[i * 3 + 0], ctx.y[i * 3 + 1], ctx.y[i * 3 + 2]);
    grad.addScaledVector(p.x.clone().sub(yi), miOverH2);

    // Edge spring energies
    const inc = ctx.incident[i];
    for (const e of inc) {
      const j = e.i0 === i ? e.i1 : e.i0;
      const pj = particles[j];

      const d = p.x.clone().sub(pj.x);
      const len = d.length();
      if (len < 1e-9) continue;

      const C = len - e.restLength;
      nrm.copy(d).multiplyScalar(1 / len);

      const comp = typeof e._getCompliance === 'function' ? e._getCompliance(settings) : settings.complianceEdges;
      const k = stiffnessFromCompliance(comp);
      // g += k * C * n
      grad.addScaledVector(nrm, k * C);
      // GN Hessian: k * (n n^T)
      addOuter(H, nrm, k);
    }

    // Volume penalty energies
    for (const t of volumeTerms) {
      const gi = t.grads[i];
      if (!gi) continue;

      const C = t.C;
      const k = t.k;
      grad.addScaledVector(gi, k * C);
      addOuter(H, gi, k);
    }

    // Floor collision penalty
    if (settings.enableFloorCollision) {
      const minY = settings.floorY + p.radius;
      const pen = minY - p.x.y;
      if (pen > 0) {
        // E = 1/2 k pen^2, grad = -k pen * up
        grad.y += -kFloor * pen;
        H[4] += kFloor;
      }
    }

    // Self-collision penalty (spheres)
    if (settings.enableSelfCollision) {
      for (let j = 0; j < n; j++) {
        if (j === i) continue;
        if (neighborEdges && neighborEdges.has(edgeKey(i, j))) continue;

        const pj = particles[j];
        const minDist = (p.radius + pj.radius) * settings.selfCollisionRadiusScale;

        const d = p.x.clone().sub(pj.x);
        const dist = d.length();
        if (dist <= 1e-9 || dist >= minDist) continue;

        const pen = minDist - dist;
        nrm.copy(d).multiplyScalar(1 / dist);

        // grad += -k * pen * n
        grad.addScaledVector(nrm, -kSelf * pen);
        addOuter(H, nrm, kSelf);
      }
    }

    // Newton step: H dx = -grad
    rhs.copy(grad).multiplyScalar(-1);

    // Small regularization to avoid singularities
    H[0] += 1e-8;
    H[4] += 1e-8;
    H[8] += 1e-8;

    if (!solve3x3(H, rhs, dx)) continue;

    // Optional step limiting (cheap alternative to line search)
    const maxStep = 0.35;
    const dLen = dx.length();
    if (dLen > maxStep) dx.multiplyScalar(maxStep / dLen);

    p.x.add(dx);
  }
}
