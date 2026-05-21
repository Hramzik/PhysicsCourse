// Part 3 Scene 1 — ~10 OBBs in a static box (4 walls + floor), gravity.
// Two solver methods via method-select:
//   - 'xpbd' : XPBD with position projection (substeps, one-sided)
//   - 'si'   : Sequential Impulses (velocity solver, λ≥0) + NGS position projection.
//
// Broadphase: O(n²) AABB overlap test (acceptable for n=10).
// Narrowphase: SAT for box-box, vertex-plane for box vs static plane.

import { RigidBody3D } from '../engine/rigidbody3d.js';
import { collideBoxPlane, collideBoxBox, getBoxAABB, aabbOverlap } from '../engine/collision.js';
import { xpbdSolveContacts, siSolveContacts } from '../engine/contact_solver.js';

const NUM_BODIES = 10;
const BOX_HALF = 3.0;     // half-width of the container box
const BOX_HEIGHT = 6.0;   // top of container (open top)

export function loadPart3Scene1Boxes(rendererData, method = 'xpbd') {
  const scene = rendererData.scene;

  // ─── Static planes (floor + 4 walls) ──────────────────────────────────
  // Plane normal points OUT of the solid (into the open region).
  const planes = [
    // Floor
    { point: new THREE.Vector3(0, 0, 0), normal: new THREE.Vector3(0, 1, 0) },
    // +X wall (normal -X means open region is -X side)
    { point: new THREE.Vector3( BOX_HALF, 0, 0), normal: new THREE.Vector3(-1, 0, 0) },
    { point: new THREE.Vector3(-BOX_HALF, 0, 0), normal: new THREE.Vector3( 1, 0, 0) },
    { point: new THREE.Vector3(0, 0,  BOX_HALF), normal: new THREE.Vector3(0, 0, -1) },
    { point: new THREE.Vector3(0, 0, -BOX_HALF), normal: new THREE.Vector3(0, 0,  1) }
  ];

  // ─── Bodies ───────────────────────────────────────────────────────────
  const bodies = [];
  const meshes = [];
  const palette = [0x88ccff, 0xffaa88, 0x88ffaa, 0xffcc55, 0xff88cc,
                   0xaa88ff, 0x88ffee, 0xffee88, 0xcc88ff, 0xeeff88];

  // Deterministic pseudo-random so resets look the same
  let seed = 1;
  const rand = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 0xFFFFFFFF;
  };

  for (let i = 0; i < NUM_BODIES; i++) {
    const sx = 0.4 + rand() * 0.5;
    const sy = 0.4 + rand() * 0.5;
    const sz = 0.4 + rand() * 0.5;
    const px = (2 * rand() - 0.5) * (BOX_HALF * 2 - sx * 2);
    const py = 1.0 + i * 0.8 + rand() * 0.2;
    const pz = (rand() - 0.5) * (BOX_HALF * 2 - sz * 2);
    const q = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(rand() * Math.PI, rand() * Math.PI, rand() * Math.PI)
    );

    const body = new RigidBody3D({ mass: sx * sy * sz * 5, size: [sx, sy, sz], position: [px, py, pz], quaternion: q });
    bodies.push(body);

    const geom = new THREE.BoxGeometry(sx, sy, sz);
    const mat = new THREE.MeshStandardMaterial({ color: palette[i % palette.length] });
    const mesh = new THREE.Mesh(geom, mat);
    scene.add(mesh);
    meshes.push({ mesh, geom, mat });
  }

  // ─── Container visualization (wireframe box) ──────────────────────────
  const containerGroup = new THREE.Group();
  // Floor
  const floorGeom = new THREE.PlaneGeometry(BOX_HALF * 2, BOX_HALF * 2);
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x444455, side: THREE.DoubleSide });
  const floorMesh = new THREE.Mesh(floorGeom, floorMat);
  floorMesh.rotation.x = -Math.PI / 2;
  containerGroup.add(floorMesh);
  // Wireframe walls
  const wallEdges = new THREE.EdgesGeometry(new THREE.BoxGeometry(BOX_HALF * 2, BOX_HEIGHT, BOX_HALF * 2));
  const wallLineMat = new THREE.LineBasicMaterial({ color: 0x888899 });
  const wallLines = new THREE.LineSegments(wallEdges, wallLineMat);
  wallLines.position.y = BOX_HEIGHT * 0.5;
  containerGroup.add(wallLines);
  scene.add(containerGroup);

  // ─── State ────────────────────────────────────────────────────────────
  let currentDamping = 0;
  // Base Coulomb coefficients (scaled by the global friction slider).
  const BASE_MU_S = 0.5;
  const BASE_MU_D = 0.4;
  let frictionScale = 1;
  const statElems = {
    springLen: document.getElementById('springLen'),
    springForce: document.getElementById('springForce'),
    springEnergy: document.getElementById('springEnergy'),
    totalEnergy: document.getElementById('totalEnergy')
  };

  // ─── Broadphase + narrowphase contact detection ───────────────────────
  function detectContacts(bodies, planes) {
    const contacts = [];

    // Body vs static planes
    for (let i = 0; i < bodies.length; i++) {
      for (let p = 0; p < planes.length; p++) {
        collideBoxPlane(bodies[i], planes[p], contacts);
      }
    }

    // Body vs body (O(n²) broadphase via AABB, then SAT)
    const aabbs = new Array(bodies.length);
    for (let i = 0; i < bodies.length; i++) aabbs[i] = getBoxAABB(bodies[i]);

    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        if (!aabbOverlap(aabbs[i], aabbs[j])) continue;
        collideBoxBox(bodies[i], bodies[j], contacts);
      }
    }
    return contacts;
  }

  // ─── Step ──────────────────────────────────────────────────────────────
  return {
    step(dt) {
      const muS = BASE_MU_S * frictionScale;
      const muD = BASE_MU_D * frictionScale;
      let info;
      if (method === 'xpbd') {
        info = xpbdSolveContacts(bodies, planes, detectContacts, dt, {
          substeps: 10,
          damping: currentDamping,
          muS, muD
        });
      } else {
        info = siSolveContacts(bodies, planes, detectContacts, dt, {
          velIterations: 8,
          posIterations: 4,
          damping: currentDamping,
          muS  // SI uses a single μ for box-friction
        });
      }

      // Sync meshes
      for (let i = 0; i < bodies.length; i++) {
        meshes[i].mesh.position.copy(bodies[i].position);
        meshes[i].mesh.quaternion.copy(bodies[i].quaternion);
      }

      // Stats
      if (statElems.springLen) {
        statElems.springLen.textContent = '—';
        statElems.springForce.textContent = `${info.totalContacts} contacts`;
        statElems.springEnergy.textContent = '—';
        let ke = 0;
        for (let i = 0; i < bodies.length; i++) ke += kineticEnergy(bodies[i]);
        statElems.totalEnergy.textContent = ke.toFixed(2);
      }
    },

    setDamping(value) { currentDamping = value; },
    setFriction(scale) { frictionScale = scale; },

    statElems,

    dispose() {
      for (const m of meshes) {
        try { scene.remove(m.mesh); } catch(e){}
        try { m.geom.dispose(); } catch(e){}
        try { m.mat.dispose(); } catch(e){}
      }
      try { scene.remove(containerGroup); } catch(e){}
      try { floorGeom.dispose(); } catch(e){}
      try { floorMat.dispose(); } catch(e){}
      try { wallEdges.dispose(); } catch(e){}
      try { wallLineMat.dispose(); } catch(e){}
    }
  };
}

// ─── Helpers ────────────────────────────────────────────────────────────────
function kineticEnergy(body) {
  const linear = 0.5 * body.mass * body.linearVelocity.lengthSq();
  const w = body.getAngularVelocityGlobal();
  const Iw = w.clone().applyMatrix3(body.getInertiaGlobal());
  return linear + 0.5 * w.dot(Iw);
}
