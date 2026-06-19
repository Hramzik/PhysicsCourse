// Sweep and Prune (SAP) broadphase.
//
// Algorithm (full rebuild each frame, O(N log N) for the sort):
//   1. For every body collect its AABB.
//   2. Sort body indices by aabb.min[0] (X axis).
//   3. Sweep in sorted order; maintain an "active" set whose aabb.max[0]
//      is ≥ the current candidate's min[0]. For every new candidate, pair it
//      with every body currently active (a candidate-vs-active O(N · k_avg)
//      step, where k_avg is the typical SAP overlap count along X).
//   4. For each candidate pair perform a full Y- and Z-overlap test before
//      emitting it.
//
// This is the "single-axis SAP" variant; it's the simplest robust form and
// already gives a large speed-up over O(N²) when bodies are not all in the
// same vertical column.

export class SAP {
  constructor() {
    this._sorted = [];   // body indices, sorted by min[0]
    this._active = [];   // currently active indices during the sweep
    this._aabbs  = null;
  }

  // aabbs: Array<{min:[x,y,z], max:[x,y,z]}>
  rebuild(aabbs) {
    this._aabbs = aabbs;
    const n = aabbs.length;
    // (Re)build the index array.
    if (this._sorted.length !== n) this._sorted = new Array(n);
    for (let i = 0; i < n; i++) this._sorted[i] = i;
    // Sort indices by min X (cheapest single comparison).
    const a = aabbs;
    this._sorted.sort((i, j) => a[i].min[0] - a[j].min[0]);
  }

  // Return all overlapping pairs [i, j] with i < j.
  getPairs() {
    const pairs = [];
    const a = this._aabbs;
    const sorted = this._sorted;
    const active = this._active;
    active.length = 0;

    for (let s = 0; s < sorted.length; s++) {
      const i = sorted[s];
      const ai = a[i];
      // Prune the active set: drop any whose max[0] < ai.min[0].
      let w = 0;
      for (let r = 0; r < active.length; r++) {
        const j = active[r];
        if (a[j].max[0] >= ai.min[0]) {
          active[w++] = j;
        }
      }
      active.length = w;

      // Test against everything still active (overlap on X is guaranteed by
      // construction; only verify Y and Z).
      for (let r = 0; r < active.length; r++) {
        const j = active[r];
        const aj = a[j];
        if (ai.max[1] < aj.min[1] || ai.min[1] > aj.max[1]) continue;
        if (ai.max[2] < aj.min[2] || ai.min[2] > aj.max[2]) continue;
        // Emit ordered pair (smaller index first) for caller-side consistency.
        if (i < j) pairs.push([i, j]); else pairs.push([j, i]);
      }
      active.push(i);
    }
    return pairs;
  }
}
