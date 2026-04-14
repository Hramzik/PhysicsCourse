import * as THREE from '../../vendor/three/three.module.min.js';

const UP = new THREE.Vector3(0, 1, 0);

export function projectFloorPositions(groups, floorY) {
  for (const group of groups) {
    for (const p of group.particles) {
      const r = p.radius;
      const minY = floorY + r;
      if (p.x.y < minY) p.x.y = minY;
    }
  }
}

export function applyFloorVelocity(groups, dt, floorY, restitution, friction) {
  const eps = 1e-5;
  for (const group of groups) {
    for (const p of group.particles) {
      if (p.invMass === 0) continue;

      const r = p.radius;
      const minY = floorY + r;
      if (p.x.y > minY + eps) continue;

      // Reconstruct velocity from Verlet
      const v = p.x.clone().sub(p.xPrev).multiplyScalar(1 / dt);
      const vn = v.dot(UP);
      const vt = v.clone().sub(UP.clone().multiplyScalar(vn));

      let vnOut = vn;
      if (vn < 0) vnOut = -restitution * vn;

      const vtLen = vt.length();
      if (vtLen > 1e-9) {
        const maxDrop = friction * (1 + restitution) * Math.abs(vn);
        const newLen = Math.max(0, vtLen - maxDrop);
        vt.multiplyScalar(newLen / vtLen);
      }

      const vOut = vt.addScaledVector(UP, vnOut);
      p.xPrev.copy(p.x).addScaledVector(vOut, -dt);
    }
  }
}

function edgeKey(i, j) {
  return i < j ? i + ',' + j : j + ',' + i;
}

export function projectSelfCollisions(groups, radiusScale) {
  for (const group of groups) {
    const particles = group.particles;
    const neighborEdges = group.neighborEdges || null; // Set of "i,j" keys

    for (let i = 0; i < particles.length; i++) {
      for (let j = i + 1; j < particles.length; j++) {
        if (neighborEdges && neighborEdges.has(edgeKey(i, j))) continue;

        const p0 = particles[i];
        const p1 = particles[j];

        const r0 = p0.radius * radiusScale;
        const r1 = p1.radius * radiusScale;
        const minDist = r0 + r1;

        const d = p0.x.clone().sub(p1.x);
        const dist = d.length();
        if (dist <= 1e-9 || dist >= minDist) continue;

        const n = d.multiplyScalar(1 / dist);
        const penetration = minDist - dist;

        const w0 = p0.invMass;
        const w1 = p1.invMass;
        const wSum = w0 + w1;
        if (wSum === 0) continue;

        // Position correction
        const corr0 = (w0 / wSum) * penetration;
        const corr1 = (w1 / wSum) * penetration;

        p0.x.addScaledVector(n, corr0);
        p1.x.addScaledVector(n, -corr1);
      }
    }
  }
}

export function xpbdIteration(groups, dt, settings) {
  for (let it = 0; it < settings.iterations; it++) {
    for (const group of groups) {
      for (const c of group.constraints) c.project(group, dt, settings);
    }

    if (settings.enableFloorCollision) {
      projectFloorPositions(groups, settings.floorY);
    }
    if (settings.enableSelfCollision) {
      projectSelfCollisions(groups, settings.selfCollisionRadiusScale);
    }
    if (settings.enableFloorCollision) {
      applyFloorVelocity(groups, dt, settings.floorY, settings.restitution, settings.friction);
    }
  }
}
