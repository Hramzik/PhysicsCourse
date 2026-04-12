import * as THREE from '../../vendor/three/three.module.min.js';
import { buildDeformableCubeBody } from '../sim/builders.js';

export function buildScene1({ sim, scene, params, setStatus, clearVisuals, shellMeshes, particleMeshes }) {
  // Clear previous
  sim.reset();
  clearVisuals();

  // Deformable body
  const body = buildDeformableCubeBody({
    size: 1.2,
    center: new THREE.Vector3(0, 0.7, 0),
    particleRadius: params.particleRadius,
  });
  sim.addGroup(body);

  // Pin a few top particles
  const indices = body.particles
    .map((p, i) => ({ i, y: p.x.y }))
    .sort((a, b) => b.y - a.y)
    .slice(0, 2)
    .map(o => o.i);
  for (const i of indices) body.particles[i].invMass = 0;

  setStatus(
    `Ок: частицы тела = ${body.particles.length}\n` +
      `Pinned anchors: ${indices.length}\n` +
      `Запуск: ${params.paused ? 'PAUSED' : 'RUN'}`
  );

  // Visualize deformable: mesh shell
  const bodyMesh = new THREE.Mesh(
    body.renderGeometry,
    new THREE.MeshStandardMaterial({
      color: 0x6aa6ff,
      transparent: true,
      opacity: 0.25,
      roughness: 0.2,
      metalness: 0.0,
      side: THREE.DoubleSide,
    })
  );
  scene.add(bodyMesh);
  shellMeshes.push(bodyMesh);

  // Particles (spheres)
  const sphereGeo = new THREE.SphereGeometry(1, 16, 12);
  for (let i = 0; i < body.particles.length; i++) {
    const p = body.particles[i];
    const mat = new THREE.MeshStandardMaterial({
      color: p.invMass === 0 ? 0xff4b4b : 0xdbe6ff,
      roughness: 0.35,
    });
    const mesh = new THREE.Mesh(sphereGeo, mat);
    mesh.scale.setScalar(params.particleRadius);
    mesh.position.copy(p.x);
    mesh.userData = { group: body, index: i };
    scene.add(mesh);
    particleMeshes.push(mesh);
  }

  body.kind = 'cube';
}
