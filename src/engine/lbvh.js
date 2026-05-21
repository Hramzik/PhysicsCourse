// LBVH — Linear Bounding Volume Hierarchy.
//
// Full rebuild per frame:
//   1. Compute world AABB of all input AABBs (the scene bound).
//   2. Compute Morton code (30-bit, 10 bits per axis) for each body's center
//      normalised into [0,1]³ relative to the scene bound.
//   3. Sort body indices by Morton code (gives spatial locality).
//   4. Build the BVH by top-down split. At each node, find the longest
//      axis of the parent AABB and split sorted indices at the median.
//      (This is a simpler/more robust variant of Karras 2012; for ~few hundred
//      bodies it's plenty fast and easier to debug.)
//   5. Compute internal-node AABBs bottom-up.
//
// Pair query: self-traversal that walks pairs of subtrees and prunes by AABB.

export class LBVH {
  constructor() {
    this._aabbs = null;
    // Flat arrays for the tree nodes (interleaved-style but with parallel arrays).
    // For internal node k:
    //   nodeLeft[k], nodeRight[k]  — child indices (positive ⇒ internal,
    //                                negative ⇒ leaf: -(bodyIndex+1))
    //   nodeMin[k*3..k*3+2], nodeMax[…]  — AABB
    this.nodeLeft  = [];
    this.nodeRight = [];
    this.nodeMin   = [];
    this.nodeMax   = [];
    this.root      = -1;
  }

  rebuild(aabbs) {
    this._aabbs = aabbs;
    const n = aabbs.length;
    this.nodeLeft.length = 0;
    this.nodeRight.length = 0;
    this.nodeMin.length = 0;
    this.nodeMax.length = 0;
    if (n === 0) { this.root = -1; return; }
    if (n === 1) {
      this.root = this._makeLeaf(0);
      return;
    }

    // Scene bound
    let mnx = aabbs[0].min[0], mny = aabbs[0].min[1], mnz = aabbs[0].min[2];
    let mxx = aabbs[0].max[0], mxy = aabbs[0].max[1], mxz = aabbs[0].max[2];
    for (let i = 1; i < n; i++) {
      const a = aabbs[i];
      if (a.min[0] < mnx) mnx = a.min[0]; if (a.max[0] > mxx) mxx = a.max[0];
      if (a.min[1] < mny) mny = a.min[1]; if (a.max[1] > mxy) mxy = a.max[1];
      if (a.min[2] < mnz) mnz = a.min[2]; if (a.max[2] > mxz) mxz = a.max[2];
    }
    const invX = 1 / Math.max(1e-9, mxx - mnx);
    const invY = 1 / Math.max(1e-9, mxy - mny);
    const invZ = 1 / Math.max(1e-9, mxz - mnz);

    // Morton codes + indices
    const morton = new Uint32Array(n);
    const indices = new Array(n);
    for (let i = 0; i < n; i++) {
      const cx = 0.5 * (aabbs[i].min[0] + aabbs[i].max[0]);
      const cy = 0.5 * (aabbs[i].min[1] + aabbs[i].max[1]);
      const cz = 0.5 * (aabbs[i].min[2] + aabbs[i].max[2]);
      const ux = clampUnit((cx - mnx) * invX);
      const uy = clampUnit((cy - mny) * invY);
      const uz = clampUnit((cz - mnz) * invZ);
      morton[i] = morton3D(ux, uy, uz);
      indices[i] = i;
    }
    // Sort indices by morton code (stable enough; ties broken by index).
    indices.sort((i, j) => {
      const d = morton[i] - morton[j];
      return d !== 0 ? d : i - j;
    });

    // Build BVH top-down with median split along longest axis
    this.root = this._build(indices, 0, n - 1);
  }

  // Recursively build [lo..hi] inclusive of `indices` (sorted by morton).
  // Returns node id (positive = internal, negative = leaf).
  _build(indices, lo, hi) {
    if (lo === hi) return this._makeLeaf(indices[lo]);
    const aabbs = this._aabbs;
    // Compute parent bound
    let mnx = Infinity, mny = Infinity, mnz = Infinity;
    let mxx = -Infinity, mxy = -Infinity, mxz = -Infinity;
    for (let k = lo; k <= hi; k++) {
      const a = aabbs[indices[k]];
      if (a.min[0] < mnx) mnx = a.min[0]; if (a.max[0] > mxx) mxx = a.max[0];
      if (a.min[1] < mny) mny = a.min[1]; if (a.max[1] > mxy) mxy = a.max[1];
      if (a.min[2] < mnz) mnz = a.min[2]; if (a.max[2] > mxz) mxz = a.max[2];
    }
    // Determine split position. Because indices are already sorted by Morton
    // code, splitting at the median in the morton ordering already gives a
    // spatially coherent partition — that's the whole point of LBVH. We use
    // simple median split.
    const mid = (lo + hi) >> 1;
    const left  = this._build(indices, lo, mid);
    const right = this._build(indices, mid + 1, hi);
    const id = this.nodeLeft.length;
    this.nodeLeft.push(left);
    this.nodeRight.push(right);
    this.nodeMin.push(mnx, mny, mnz);
    this.nodeMax.push(mxx, mxy, mxz);
    return id;
  }

  _makeLeaf(bodyIndex) {
    // Encode leaf as negative number: -(bodyIndex+1)
    return -(bodyIndex + 1);
  }

  // Walks the tree and emits pairs of body indices whose AABBs overlap.
  getPairs() {
    const pairs = [];
    if (this.root < 0) return pairs;             // single leaf or empty
    this._descendSelf(this.root, pairs);
    return pairs;
  }

  // Self-traversal: walk all overlapping pairs WITHIN a subtree.
  _descendSelf(node, pairs) {
    if (node < 0) return;  // leaf has no internal pairs
    const L = this.nodeLeft[node];
    const R = this.nodeRight[node];
    this._descendSelf(L, pairs);
    this._descendSelf(R, pairs);
    this._descendCross(L, R, pairs);
  }

  // Cross-traversal: emit all overlapping body pairs where one is in left subtree
  // and the other in right subtree.
  _descendCross(a, b, pairs) {
    if (!this._overlap(a, b)) return;
    if (a < 0 && b < 0) {
      // Both leaves
      const i = -a - 1, j = -b - 1;
      const aabbs = this._aabbs;
      const ai = aabbs[i], aj = aabbs[j];
      // Recheck AABB overlap on the actual body AABBs (node-overlap is on the
      // ENCLOSING AABBs of subtrees — they were equal in this leaf-leaf case
      // but the check is essentially free and guards against rounding).
      if (ai.max[0] < aj.min[0] || ai.min[0] > aj.max[0]) return;
      if (ai.max[1] < aj.min[1] || ai.min[1] > aj.max[1]) return;
      if (ai.max[2] < aj.min[2] || ai.min[2] > aj.max[2]) return;
      if (i < j) pairs.push([i, j]); else pairs.push([j, i]);
      return;
    }
    if (a < 0) {
      // a is leaf, b is internal → descend b
      this._descendCross(a, this.nodeLeft[b], pairs);
      this._descendCross(a, this.nodeRight[b], pairs);
      return;
    }
    if (b < 0) {
      this._descendCross(this.nodeLeft[a],  b, pairs);
      this._descendCross(this.nodeRight[a], b, pairs);
      return;
    }
    // Both internal → 4 children combinations
    this._descendCross(this.nodeLeft[a],  this.nodeLeft[b],  pairs);
    this._descendCross(this.nodeLeft[a],  this.nodeRight[b], pairs);
    this._descendCross(this.nodeRight[a], this.nodeLeft[b],  pairs);
    this._descendCross(this.nodeRight[a], this.nodeRight[b], pairs);
  }

  // AABB overlap test between two nodes (leaf or internal).
  _overlap(a, b) {
    let amn, amx, bmn, bmx;
    if (a < 0) {
      const aabb = this._aabbs[-a - 1];
      amn = aabb.min; amx = aabb.max;
    } else {
      const o = a * 3;
      amn = [this.nodeMin[o], this.nodeMin[o+1], this.nodeMin[o+2]];
      amx = [this.nodeMax[o], this.nodeMax[o+1], this.nodeMax[o+2]];
    }
    if (b < 0) {
      const aabb = this._aabbs[-b - 1];
      bmn = aabb.min; bmx = aabb.max;
    } else {
      const o = b * 3;
      bmn = [this.nodeMin[o], this.nodeMin[o+1], this.nodeMin[o+2]];
      bmx = [this.nodeMax[o], this.nodeMax[o+1], this.nodeMax[o+2]];
    }
    return amx[0] >= bmn[0] && amn[0] <= bmx[0]
        && amx[1] >= bmn[1] && amn[1] <= bmx[1]
        && amx[2] >= bmn[2] && amn[2] <= bmx[2];
  }
}

// ─── Morton helpers (10 bits per axis → 30-bit code) ────────────────────────
function expandBits10(v) {
  // v in [0..1023]. Spread bits into every 3rd slot.
  v = (v | (v << 16)) & 0x030000FF;
  v = (v | (v <<  8)) & 0x0300F00F;
  v = (v | (v <<  4)) & 0x030C30C3;
  v = (v | (v <<  2)) & 0x09249249;
  return v;
}

function morton3D(x, y, z) {
  // Inputs in [0,1]; quantise to 10 bits.
  const ix = Math.min(1023, Math.max(0, Math.floor(x * 1024)));
  const iy = Math.min(1023, Math.max(0, Math.floor(y * 1024)));
  const iz = Math.min(1023, Math.max(0, Math.floor(z * 1024)));
  return (expandBits10(ix) << 2) | (expandBits10(iy) << 1) | expandBits10(iz);
}

function clampUnit(v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
