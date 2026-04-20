export function detectContacts(particles, settings) {
  const contacts = [];
  const n = particles.length;
  if (n === 0) return contacts;

  if (settings.enableFloorCollision) {
    const floorY = settings.floorY;
    for (let i = 0; i < n; i++) {
      const p = particles[i];
      if (p.invMass === 0) continue;
      const minY = floorY + p.radius;
      const pen = minY - p.x.y;
      if (pen > 0) {
        contacts.push({ type: 'floor', i, j: null, xA: p.x.clone(), xB: null, n: { x: 0, y: 1, z: 0 }, penetration: pen });
      }
    }
  }

  if (!settings.enableSelfCollision) return contacts;

  let maxR = 0;
  for (const p of particles) if (p.radius > maxR) maxR = p.radius;
  const cellSize = Math.max(1e-3, (maxR * 2) * (settings.selfCollisionRadiusScale || 1));

  const hash = new Map();
  function cellKey(ix, iy, iz) { return ix + ',' + iy + ',' + iz; }
  function insert(i, x) {
    const ix = Math.floor(x.x / cellSize);
    const iy = Math.floor(x.y / cellSize);
    const iz = Math.floor(x.z / cellSize);
    const k = cellKey(ix, iy, iz);
    let arr = hash.get(k);
    if (!arr) { arr = []; hash.set(k, arr); }
    arr.push(i);
  }

  for (let i = 0; i < n; i++) insert(i, particles[i].x);

  // Search neighboring cells
  for (let i = 0; i < n; i++) {
    const pi = particles[i];
    if (pi.invMass === 0) continue;
    const ix = Math.floor(pi.x.x / cellSize);
    const iy = Math.floor(pi.x.y / cellSize);
    const iz = Math.floor(pi.x.z / cellSize);

    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (let dz = -1; dz <= 1; dz++) {
          const k = cellKey(ix + dx, iy + dy, iz + dz);
          const arr = hash.get(k);
          if (!arr) continue;
          for (const j of arr) {
            if (j <= i) continue;
            const pj = particles[j];
            if (pj.invMass === 0) continue;

            const minDist = (pi.radius + pj.radius) * (settings.selfCollisionRadiusScale || 1);
            const dxv = pi.x.x - pj.x.x;
            const dyv = pi.x.y - pj.x.y;
            const dzv = pi.x.z - pj.x.z;
            const dist2 = dxv*dxv + dyv*dyv + dzv*dzv;
            if (dist2 <= 1e-12) continue;
            const dist = Math.sqrt(dist2);
            if (dist >= minDist) continue;

            // contact normal from j -> i
            const nx = dxv / dist;
            const ny = dyv / dist;
            const nz = dzv / dist;
            const pen = minDist - dist;
            contacts.push({ type: 'pair', i, j, xA: pi.x.clone(), xB: pj.x.clone(), n: { x: nx, y: ny, z: nz }, penetration: pen });
          }
        }
      }
    }
  }

  return contacts;
}
