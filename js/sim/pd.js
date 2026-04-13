import { DistanceConstraint, VolumeConstraint } from './constraints.js';

function clamp(x, a, b) {
  return Math.max(a, Math.min(b, x));
}

function pdWeightFromCompliance(complianceEdges) {
  const c = Number(complianceEdges);
  if (!Number.isFinite(c)) return 1;

  // Map “compliance” (XPBD-style) to a PD weight
  const w = 1 / Math.max(1e-6, c);
  return clamp(w, 1, 1e6);
}

function choleskyDecompose(A, n) {
  const L = new Float64Array(n * n);

  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = A[i * n + j];
      for (let k = 0; k < j; k++) {
        sum -= L[i * n + k] * L[j * n + k];
      }

      if (i === j) {
        L[i * n + j] = Math.sqrt(Math.max(sum, 1e-12));
      } else {
        L[i * n + j] = sum / L[j * n + j];
      }
    }
  }

  return L;
}

function choleskySolve(L, n, b) {
  const y = new Float64Array(n);

  for (let i = 0; i < n; i++) {
    let sum = b[i];
    for (let k = 0; k < i; k++) {
      sum -= L[i * n + k] * y[k];
    }
    y[i] = sum / L[i * n + i];
  }

  const x = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let sum = y[i];
    for (let k = i + 1; k < n; k++) {
      sum -= L[k * n + i] * x[k];
    }
    x[i] = sum / L[i * n + i];
  }

  return x;
}

export function createPDContext(group, dt, settings) {
  const particles = group.particles;
  const n = particles.length;

  const edges = [];
  const volumes = [];
  for (const c of group.constraints) {
    if (c instanceof DistanceConstraint) edges.push(c);
    else if (c instanceof VolumeConstraint) volumes.push(c);
  }

  const xStar = new Float64Array(n * 3);
  const miOverH2 = new Float64Array(n);
  const pinned = new Uint8Array(n);

  const h2 = dt * dt;
  for (let i = 0; i < n; i++) {
    const p = particles[i];
    xStar[i * 3 + 0] = p.x.x;
    xStar[i * 3 + 1] = p.x.y;
    xStar[i * 3 + 2] = p.x.z;

    if (p.invMass === 0) {
      pinned[i] = 1;
      miOverH2[i] = 0;
    } else {
      miOverH2[i] = 1 / (p.invMass * h2);
    }
  }

  const w = pdWeightFromCompliance(settings.complianceEdges);

  const deg = new Int32Array(n);
  for (const e of edges) {
    deg[e.i0]++;
    deg[e.i1]++;
  }

  const A = new Float64Array(n * n);

  for (let i = 0; i < n; i++) {
    A[i * n + i] = miOverH2[i] + w * deg[i];
  }

  for (const e of edges) {
    const i = e.i0;
    const j = e.i1;
    A[i * n + j] -= w;
    A[j * n + i] -= w;
  }

  for (let i = 0; i < n; i++) {
    if (!pinned[i]) continue;
    for (let k = 0; k < n; k++) {
      A[i * n + k] = 0;
      A[k * n + i] = 0;
    }
    A[i * n + i] = 1;
  }

  const L = choleskyDecompose(A, n);

  return { n, edges, volumes, xStar, miOverH2, pinned, w, L };
}

export function pdIteration(group, dt, settings, ctx) {
  const particles = group.particles;
  const n = ctx.n;
  const w = ctx.w;

  const bx = new Float64Array(n);
  const by = new Float64Array(n);
  const bz = new Float64Array(n);

  for (let i = 0; i < n; i++) {
    if (ctx.pinned[i]) {
      const p = particles[i];
      bx[i] = p.x.x;
      by[i] = p.x.y;
      bz[i] = p.x.z;
      continue;
    }

    const mH2 = ctx.miOverH2[i];
    bx[i] = mH2 * ctx.xStar[i * 3 + 0];
    by[i] = mH2 * ctx.xStar[i * 3 + 1];
    bz[i] = mH2 * ctx.xStar[i * 3 + 2];
  }

  for (const e of ctx.edges) {
    const i = e.i0;
    const j = e.i1;

    const xi = particles[i].x;
    const xj = particles[j].x;

    const dx = xi.x - xj.x;
    const dy = xi.y - xj.y;
    const dz = xi.z - xj.z;

    const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-9) continue;

    const s = e.restLength / len;
    const ddx = dx * s;
    const ddy = dy * s;
    const ddz = dz * s;

    const iPinned = ctx.pinned[i] === 1;
    const jPinned = ctx.pinned[j] === 1;

    // Local projection contribution (d_ij) only for free variables
    if (!iPinned) {
      bx[i] += w * ddx;
      by[i] += w * ddy;
      bz[i] += w * ddz;
    }
    if (!jPinned) {
      bx[j] -= w * ddx;
      by[j] -= w * ddy;
      bz[j] -= w * ddz;
    }

    if (!iPinned && jPinned) {
      bx[i] += w * xj.x;
      by[i] += w * xj.y;
      bz[i] += w * xj.z;
    } else if (!jPinned && iPinned) {
      bx[j] += w * xi.x;
      by[j] += w * xi.y;
      bz[j] += w * xi.z;
    }
  }

  const xOut = choleskySolve(ctx.L, n, bx);
  const yOut = choleskySolve(ctx.L, n, by);
  const zOut = choleskySolve(ctx.L, n, bz);

  for (let i = 0; i < n; i++) {
    if (ctx.pinned[i]) continue;
    const p = particles[i];
    p.x.x = xOut[i];
    p.x.y = yOut[i];
    p.x.z = zOut[i];
  }

  for (const v of ctx.volumes) {
    v.project(group, dt, settings);
  }
}
