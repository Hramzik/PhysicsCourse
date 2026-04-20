import * as THREE from '../../vendor/three/three.module.min.js';
import { buildDeformableCubeBody } from '../sim/builders.js';

export function buildScene3({ sim, scene, params, setStatus, clearVisuals, shellMeshes, particleMeshes }) {
  sim.reset();
  clearVisuals();

  const body = buildDeformableCubeBody({
    size: 1.2,
    center: new THREE.Vector3(0, 0.7, 0),
    particleRadius: params.particleRadius,
  });
  sim.addGroup(body);

  const center = new THREE.Vector3();
  for (const p of body.particles) center.add(p.x);
  center.multiplyScalar(1 / body.particles.length);

  const top = body.particles
    .map((p, i) => ({ i, y: p.x.y }))
    .sort((a, b) => b.y - a.y)
    .slice(0, 4)
    .map(o => o.i);

  for (const i of top) {
    const p = body.particles[i];
    p.invMass = 0;

    const sx = Math.sign(p.x.x - center.x) || 1;
    const sz = Math.sign(p.x.z - center.z) || 1;

    const target = p.x
      .clone()
      .add(new THREE.Vector3(sx * 0.32, 0.0, sz * 0.12))
      .add(new THREE.Vector3(sz * 0.12, 0.0, -sx * 0.32));

    p.x.copy(target);
    p.xPrev.copy(target);
  }

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
