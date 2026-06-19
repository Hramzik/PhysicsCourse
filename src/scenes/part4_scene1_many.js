// Part 4 Scene — many OBBs of *varying* size in a static box.
// Broadphase comparison: SAP vs LBVH (method selector).
// Narrowphase: SAT. Solver: XPBD with friction.

import { RigidBody3D } from '../engine/rigidbody3d.js';
import { collideBoxPlane, collideBoxBox, getBoxAABB } from '../engine/collision.js';
import { xpbdSolveContacts } from '../engine/contact_solver.js';
import { SAP } from '../engine/sap.js';
import { LBVH } from '../engine/lbvh.js';

const NUM_BODIES = 200;
const BOX_HALF = 6.0;
const BOX_HEIGHT = 12.0;

export function loadPart4Scene1Many(rendererData, method = 'sap') {
  const scene = rendererData.scene;

  // ─── Static planes (open-top box) ─────────────────────────────────────
  const planes = [
    { point: new THREE.Vector3(0, 0, 0), normal: new THREE.Vector3(0, 1, 0) },
    { point: new THREE.Vector3( BOX_HALF, 0, 0), normal: new THREE.Vector3(-1, 0, 0) },
    { point: new THREE.Vector3(-BOX_HALF, 0, 0), normal: new THREE.Vector3( 1, 0, 0) },
    { point: new THREE.Vector3(0, 0,  BOX_HALF), normal: new THREE.Vector3(0, 0, -1) },
    { point: new THREE.Vector3(0, 0, -BOX_HALF), normal: new THREE.Vector3(0, 0,  1) }
  ];

  // ─── Bodies of varying size ───────────────────────────────────────────
  const bodies = [];
  const meshes = [];

  let seed = 1;
  const rand = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 0xFFFFFFFF;
  };

  // Distribute bodies in a tall column above the box so they cascade in.
  // Vary size across [0.2, 1.0] uniformly per axis.
  for (let i = 0; i < NUM_BODIES; i++) {
    const sx = 0.2 + rand() * 0.8;
    const sy = 0.2 + rand() * 0.8;
    const sz = 0.2 + rand() * 0.8;
    const px = (rand() - 0.5) * (BOX_HALF * 2 - 1);
    const py = 1.0 + i * 0.12 + rand() * 0.05;
    const pz = (rand() - 0.5) * (BOX_HALF * 2 - 1);
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(
      rand() * Math.PI, rand() * Math.PI, rand() * Math.PI
    ));

    const body = new RigidBody3D({
      mass: sx * sy * sz * 5,
      size: [sx, sy, sz],
      position: [px, py, pz],
      quaternion: q
    });
    bodies.push(body);

    const geom = new THREE.BoxGeometry(sx, sy, sz);
    const hue = (i * 37) % 360;
    const c = new THREE.Color().setHSL(hue / 360, 0.55, 0.6);
    const mat = new THREE.MeshStandardMaterial({ color: c });
    const mesh = new THREE.Mesh(geom, mat);
    scene.add(mesh);
    meshes.push({ mesh, geom, mat });
  }

  // ─── Container visualization ──────────────────────────────────────────
  const containerGroup = new THREE.Group();
  const floorGeom = new THREE.PlaneGeometry(BOX_HALF * 2, BOX_HALF * 2);
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x444455, side: THREE.DoubleSide });
  const floorMesh = new THREE.Mesh(floorGeom, floorMat);
  floorMesh.rotation.x = -Math.PI / 2;
  containerGroup.add(floorMesh);
  const wallEdges = new THREE.EdgesGeometry(new THREE.BoxGeometry(BOX_HALF * 2, BOX_HEIGHT, BOX_HALF * 2));
  const wallLineMat = new THREE.LineBasicMaterial({ color: 0x888899 });
  const wallLines = new THREE.LineSegments(wallEdges, wallLineMat);
  wallLines.position.y = BOX_HEIGHT * 0.5;
  containerGroup.add(wallLines);
  scene.add(containerGroup);

  // ─── State ────────────────────────────────────────────────────────────
  let currentDamping = 0.05;
  const BASE_MU_S = 0.5;
  const BASE_MU_D = 0.4;
  let frictionScale = 1;

  const sap  = new SAP();
  const lbvh = new LBVH();

  const statElems = {
    springLen: document.getElementById('springLen'),
    springForce: document.getElementById('springForce'),
    springEnergy: document.getElementById('springEnergy'),
    totalEnergy: document.getElementById('totalEnergy')
  };

  // ─── Detect contacts via the selected broadphase ──────────────────────
  const aabbs = new Array(bodies.length);

  function detectContacts(bodies, planes) {
    const contacts = [];

    // Body vs static planes
    for (let i = 0; i < bodies.length; i++) {
      for (let p = 0; p < planes.length; p++) {
        collideBoxPlane(bodies[i], planes[p], contacts);
      }
    }

    // Build per-body AABBs
    for (let i = 0; i < bodies.length; i++) aabbs[i] = getBoxAABB(bodies[i]);

    // Broadphase
    let pairs;
    if (method === 'lbvh') {
      lbvh.rebuild(aabbs);
      pairs = lbvh.getPairs();
    } else {
      sap.rebuild(aabbs);
      pairs = sap.getPairs();
    }

    // Narrowphase: SAT for each candidate pair
    for (let k = 0; k < pairs.length; k++) {
      const i = pairs[k][0], j = pairs[k][1];
      collideBoxBox(bodies[i], bodies[j], contacts);
    }
    return contacts;
  }

  // ─── Step ─────────────────────────────────────────────────────────────
  return {
    step(dt) {
      const info = xpbdSolveContacts(bodies, planes, detectContacts, dt, {
        substeps: 6,
        positionIterations: 4,
        damping: currentDamping,
        muS: BASE_MU_S * frictionScale,
        muD: BASE_MU_D * frictionScale
      });

      for (let i = 0; i < bodies.length; i++) {
        meshes[i].mesh.position.copy(bodies[i].position);
        meshes[i].mesh.quaternion.copy(bodies[i].quaternion);
      }

      if (statElems.springLen) {
        statElems.springLen.textContent = `${bodies.length} bodies`;
        statElems.springForce.textContent = `${info.totalContacts} contacts`;
        statElems.springEnergy.textContent = method.toUpperCase();
        statElems.totalEnergy.textContent = '—';
      }
    },

    setDamping(value)  { currentDamping = value; },
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
