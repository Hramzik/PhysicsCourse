export function applyVelocityCorrections(particles, contacts, dt, settings) {
  if (!contacts || contacts.length === 0) return;

  for (const c of contacts) {
    if (c.type === 'floor') {
      const p = particles[c.i];
      if (p.invMass === 0) continue;
      const n = { x: 0, y: 1, z: 0 };
      const vn = ((p.x.x - p.xPrev.x)/dt) * n.x + ((p.x.y - p.xPrev.y)/dt) * n.y + ((p.x.z - p.xPrev.z)/dt) * n.z;
      let vnOut = vn;
      if (vn < 0) vnOut = -settings.restitution * vn;

      // tangential velocity
      const vx = (p.x.x - p.xPrev.x)/dt;
      const vy = (p.x.y - p.xPrev.y)/dt;
      const vz = (p.x.z - p.xPrev.z)/dt;
      const vt = { x: vx - n.x*vn, y: vy - n.y*vn, z: vz - n.z*vn };
      const vtLen = Math.hypot(vt.x, vt.y, vt.z);
      if (vtLen > 1e-9) {
        const maxDrop = settings.friction * (1 + settings.restitution) * Math.abs(vn);
        const newLen = Math.max(0, vtLen - maxDrop);
        const scale = newLen / vtLen;
        vt.x *= scale; vt.y *= scale; vt.z *= scale;
      }

      const vOut = { x: vt.x + n.x*vnOut, y: vt.y + n.y*vnOut, z: vt.z + n.z*vnOut };
      p.xPrev.x = p.x.x - vOut.x * dt;
      p.xPrev.y = p.x.y - vOut.y * dt;
      p.xPrev.z = p.x.z - vOut.z * dt;
    } else if (c.type === 'pair') {
      const ia = c.i, ib = c.j;
      const pa = particles[ia];
      const pb = particles[ib];
      if (pa.invMass === 0 && pb.invMass === 0) continue;

      const va = { x: (pa.x.x - pa.xPrev.x)/dt, y: (pa.x.y - pa.xPrev.y)/dt, z: (pa.x.z - pa.xPrev.z)/dt };
      const vb = { x: (pb.x.x - pb.xPrev.x)/dt, y: (pb.x.y - pb.xPrev.y)/dt, z: (pb.x.z - pb.xPrev.z)/dt };

      const n = c.n; // from b -> a
      const rel = { x: va.x - vb.x, y: va.y - vb.y, z: va.z - vb.z };
      const vn = rel.x*n.x + rel.y*n.y + rel.z*n.z;
      if (vn >= 0) continue; // separating

      const invMa = pa.invMass;
      const invMb = pb.invMass;
      const e = settings.restitution;
      const j = -(1 + e) * vn / (invMa + invMb);

      const impA = { x: j * n.x, y: j * n.y, z: j * n.z };
      const impB = { x: -j * n.x, y: -j * n.y, z: -j * n.z };

      // friction
      const vt = { x: rel.x - vn*n.x, y: rel.y - vn*n.y, z: rel.z - vn*n.z };
      const vtLen = Math.hypot(vt.x, vt.y, vt.z);
      let frictionFactor = 1;
      if (vtLen > 1e-9) {
        const maxDrop = settings.friction * (1 + settings.restitution) * Math.abs(vn);
        const newLen = Math.max(0, vtLen - maxDrop);
        frictionFactor = newLen / vtLen;
        vt.x *= frictionFactor; vt.y *= frictionFactor; vt.z *= frictionFactor;
      }

      const vaNew = { x: va.x + impA.x * invMa, y: va.y + impA.y * invMa, z: va.z + impA.z * invMa };
      const vbNew = { x: vb.x + impB.x * invMb, y: vb.y + impB.y * invMb, z: vb.z + impB.z * invMb };

      // friction
      const relAfter = { x: vaNew.x - vbNew.x, y: vaNew.y - vbNew.y, z: vaNew.z - vbNew.z };
      const vnAfter = relAfter.x*n.x + relAfter.y*n.y + relAfter.z*n.z;
      const vtAfter = { x: relAfter.x - vnAfter*n.x, y: relAfter.y - vnAfter*n.y, z: relAfter.z - vnAfter*n.z };
      const vtAfterLen = Math.hypot(vtAfter.x, vtAfter.y, vtAfter.z);
      let vtScaled = { x: vtAfter.x, y: vtAfter.y, z: vtAfter.z };
      if (vtAfterLen > 1e-9) {
        const maxDrop = settings.friction * (1 + settings.restitution) * Math.abs(vn);
        const newLen = Math.max(0, vtAfterLen - maxDrop);
        const scale = newLen / vtAfterLen;
        vtScaled.x = vtAfter.x * scale; vtScaled.y = vtAfter.y * scale; vtScaled.z = vtAfter.z * scale;
      }

      // distribute tangential change equally
      const deltaT = { x: vtScaled.x - vtAfter.x, y: vtScaled.y - vtAfter.y, z: vtScaled.z - vtAfter.z };
      const vaFinal = { x: vaNew.x + 0.5 * deltaT.x, y: vaNew.y + 0.5 * deltaT.y, z: vaNew.z + 0.5 * deltaT.z };
      const vbFinal = { x: vbNew.x - 0.5 * deltaT.x, y: vbNew.y - 0.5 * deltaT.y, z: vbNew.z - 0.5 * deltaT.z };

      pa.xPrev.x = pa.x.x - vaFinal.x * dt;
      pa.xPrev.y = pa.x.y - vaFinal.y * dt;
      pa.xPrev.z = pa.x.z - vaFinal.z * dt;

      pb.xPrev.x = pb.x.x - vbFinal.x * dt;
      pb.xPrev.y = pb.x.y - vbFinal.y * dt;
      pb.xPrev.z = pb.x.z - vbFinal.z * dt;
    }
  }
}
