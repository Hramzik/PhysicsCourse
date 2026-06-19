// Spatial Hash Grid for 3D broadphase.
//
// Bodies are inserted by their world AABB; each grid cell stores the list of
// body indices that overlap it. getPairs() returns unique candidate pairs
// (i < j) that share at least one cell.
//
// Cell size should be ~ 2× typical body radius for best performance.
//
// Usage:
//   const grid = new SpatialGrid(cellSize);
//   grid.clear();
//   for (let i=0; i<bodies.length; i++) grid.insert(i, aabb_i);
//   const pairs = grid.getPairs();   // [[i,j], ...]

export class SpatialGrid {
  constructor(cellSize) {
    this.cellSize = cellSize;
    this.invCell = 1 / cellSize;
    // Map<cellKey, number[]>  cellKey is a string "ix,iy,iz"
    this.cells = new Map();
  }

  clear() {
    this.cells.clear();
  }

  // aabb: { min:[x,y,z], max:[x,y,z] }
  insert(bodyIndex, aabb) {
    const ix0 = Math.floor(aabb.min[0] * this.invCell);
    const iy0 = Math.floor(aabb.min[1] * this.invCell);
    const iz0 = Math.floor(aabb.min[2] * this.invCell);
    const ix1 = Math.floor(aabb.max[0] * this.invCell);
    const iy1 = Math.floor(aabb.max[1] * this.invCell);
    const iz1 = Math.floor(aabb.max[2] * this.invCell);

    for (let ix = ix0; ix <= ix1; ix++) {
      for (let iy = iy0; iy <= iy1; iy++) {
        for (let iz = iz0; iz <= iz1; iz++) {
          const key = ix + ',' + iy + ',' + iz;
          let list = this.cells.get(key);
          if (!list) { list = []; this.cells.set(key, list); }
          list.push(bodyIndex);
        }
      }
    }
  }

  // Return all unique candidate pairs (i, j) with i < j that share a cell.
  getPairs() {
    const pairs = [];
    const seen = new Set();
    for (const list of this.cells.values()) {
      if (list.length < 2) continue;
      for (let a = 0; a < list.length; a++) {
        for (let b = a + 1; b < list.length; b++) {
          const i = list[a], j = list[b];
          const lo = i < j ? i : j;
          const hi = i < j ? j : i;
          // Pack into a single number for the dedup set (assumes < 2^21 bodies)
          const key = lo * 2097152 + hi;
          if (seen.has(key)) continue;
          seen.add(key);
          pairs.push([lo, hi]);
        }
      }
    }
    return pairs;
  }
}
