// Part 3 Scene 2 — ~1000 uniform cubes in a static box.
// Broadphase: Spatial Hash Grid. Narrowphase: SAT. Solver: XPBD.
// Rendering: single THREE.InstancedMesh.

import { RigidBody3D } from '../engine/rigidbody3d.js';
import {
  collideBoxPlane, collideBoxBox, getBoxAABB
} from '../engine/collision.js';
import { xpbdSolveContacts } from '../engine/contact_solver.js';
import { SpatialGrid } from '../engine/spatial_grid.js';

const NUM_BODIES = 1000;
const CUBE_SIZE = 0.25;
const BOX_HALF = 4.0;
const BOX_HEIGHT = 16.0;
const CELL_SIZE = CUBE_SIZE * 2.2; // > diagonal/2 of a rotated cube of this size

export function loadPart3Scene2Many(rendererData, method = 'xpbd') {
  const scene = rendererData.scene;

  // ─── Static planes (open-top box) ─────────────────────────────────────
  const planes = [
    { point: new THREE.Vector3(0, 0, 0), normal: new THREE.Vector3(0, 1, 0) },
    { point: new THREE.Vector3( BOX_HALF, 0, 0), normal: new THREE.Vector3(-1, 0, 0) },
    { point: new THREE.Vector3(-BOX_HALF, 0, 0), normal: new THREE.Vector3( 1, 0, 0) },
    { point: new THREE.Vector3(0, 0,  BOX_HALF), normal: new THREE.Vector3(0, 0, -1) },
    { point: new THREE.Vector3(0, 0, -BOX_HALF), normal: new THREE.Vector3(0, 0,  1) }
  ];

  // ─── Bodies: NUM_BODIES uniform cubes, placed in a grid above the box ─
  const bodies = [];
  const size = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE];

  let seed = 1;
  const rand = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 0xFFFFFFFF;
  };

  // Layout in a 10×10×10 stack
  const NX = 10, NY = 10, NZ = 10;
  const spacing = CUBE_SIZE * 1.5;
  for (let iy = 0; iy < NY; iy++) {
    for (let ix = 0; ix < NX; ix++) {
      for (let iz = 0; iz < NZ; iz++) {
        const px = (ix - NX * 0.5 + 0.5) * spacing + (rand() - 0.5) * 0.02;
        const py = 1.5 + iy * spacing;
        const pz = (iz - NZ * 0.5 + 0.5) * spacing + (rand() - 0.5) * 0.02;
        const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(
          rand() * 0.4, rand() * 0.4, rand() * 0.4
        ));
        const body = new RigidBody3D({
          mass: 1, size, position: [px, py, pz], quaternion: q
        });
        bodies.push(body);
        if (bodies.length >= NUM_BODIES) break;
      }
      if (bodies.length >= NUM_BODIES) break;
    }
    if (bodies.length >= NUM_BODIES) break;
  }

  // ─── Rendering: InstancedMesh ─────────────────────────────────────────
  const geom = new THREE.BoxGeometry(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
  const mat = new THREE.MeshStandardMaterial({ color: 0x88ccff });
  const instancedMesh = new THREE.InstancedMesh(geom, mat, bodies.length);
  instancedMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);

  // Per-instance colors for variety
  const colorAttr = new THREE.InstancedBufferAttribute(new Float32Array(bodies.length * 3), 3);
  for (let i = 0; i < bodies.length; i++) {
    const c = new THREE.Color().setHSL((i * 137.5 % 360) / 360, 0.55, 0.6);
    colorAttr.setXYZ(i, c.r, c.g, c.b);
  }
  instancedMesh.instanceColor = colorAttr;
  mat.vertexColors = false;
  // Three doesn't directly use instanceColor with MeshStandardMaterial without
  // onBeforeCompile, so fall back to a single color (still readable with 1000).
  scene.add(instancedMesh);

  // Container wireframe
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
  let currentDamping = 0.05; // tiny damping helps the stack settle
  const BASE_MU_S = 0.6;
  const BASE_MU_D = 0.5;
  let frictionScale = 1;
  const grid = new SpatialGrid(CELL_SIZE);

  const statElems = {
    springLen: document.getElementById('springLen'),
    springForce: document.getElementById('springForce'),
    springEnergy: document.getElementById('springEnergy'),
    totalEnergy: document.getElementById('totalEnergy')
  };

  // ─── Detect contacts via spatial grid ─────────────────────────────────
  const aabbs = new Array(bodies.length);

  function detectContacts(bodies, planes) {
    const contacts = [];

    // Body vs planes
    for (let i = 0; i < bodies.length; i++) {
      for (let p = 0; p < planes.length; p++) {
        collideBoxPlane(bodies[i], planes[p], contacts);
      }
    }

    // Spatial hash grid for body-body pairs
    grid.clear();
    for (let i = 0; i < bodies.length; i++) {
      aabbs[i] = getBoxAABB(bodies[i]);
      grid.insert(i, aabbs[i]);
    }
    const pairs = grid.getPairs();
    for (let k = 0; k < pairs.length; k++) {
      const i = pairs[k][0], j = pairs[k][1];
      // AABB pre-test (cheap)
      const a = aabbs[i], b = aabbs[j];
      if (a.max[0] < b.min[0] || a.min[0] > b.max[0] ||
          a.max[1] < b.min[1] || a.min[1] > b.max[1] ||
          a.max[2] < b.min[2] || a.min[2] > b.max[2]) continue;
      collideBoxBox(bodies[i], bodies[j], contacts);
    }

    return contacts;
  }

  const tmpMatrix = new THREE.Matrix4();

  // ─── Step ─────────────────────────────────────────────────────────────
  return {
    step(dt) {
      const info = xpbdSolveContacts(bodies, planes, detectContacts, dt, {
        substeps: 8,
        damping: currentDamping,
        muS: BASE_MU_S * frictionScale,
        muD: BASE_MU_D * frictionScale
      });

      // Update InstancedMesh transforms
      for (let i = 0; i < bodies.length; i++) {
        tmpMatrix.compose(bodies[i].position, bodies[i].quaternion, _ONE);
        instancedMesh.setMatrixAt(i, tmpMatrix);
      }
      instancedMesh.instanceMatrix.needsUpdate = true;

      if (statElems.springLen) {
        statElems.springLen.textContent = `${bodies.length} bodies`;
        statElems.springForce.textContent = `${info.totalContacts} contacts`;
        statElems.springEnergy.textContent = '—';
        statElems.totalEnergy.textContent = '—';
      }
    },

    setDamping(value) { currentDamping = value; },
    setFriction(scale) { frictionScale = scale; },

    statElems,

    dispose() {
      try { scene.remove(instancedMesh); } catch(e){}
      try { geom.dispose(); } catch(e){}
      try { mat.dispose(); } catch(e){}
      try { scene.remove(containerGroup); } catch(e){}
      try { floorGeom.dispose(); } catch(e){}
      try { floorMat.dispose(); } catch(e){}
      try { wallEdges.dispose(); } catch(e){}
      try { wallLineMat.dispose(); } catch(e){}
    }
  };
}

const _ONE = new THREE.Vector3(1, 1, 1);
