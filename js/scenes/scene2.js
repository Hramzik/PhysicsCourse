import * as THREE from '../../vendor/three/three.module.min.js';
import { buildRigidBall, buildSoftBall } from '../sim/builders.js';

export function buildScene2({ sim, scene, params, setStatus, clearVisuals, shellMeshes, particleMeshes }) {
  sim.reset();
  clearVisuals();

  const dt = params.dt;

  const rigid = buildRigidBall({
    radius: 0.32,
    detail: 0,
    center: new THREE.Vector3(-0.6, params.floorY + 2.2, 0),
    particleRadius: params.particleRadius,
  });
  rigid.kind = 'rigid';
  sim.addGroup(rigid);

  const soft = buildSoftBall({
    radius: 0.32,
    detail: 0,
    center: new THREE.Vector3(0.6, params.floorY + 2.2, 0),
    particleRadius: params.particleRadius,
  });
  soft.kind = 'soft';
  sim.addGroup(soft);

  const sphereGeo = new THREE.SphereGeometry(1, 16, 12);

  function addGroupVisual(group, shellColor, pointColor) {
    const shell = new THREE.Mesh(
      group.renderGeometry,
      new THREE.MeshStandardMaterial({
        color: shellColor,
        transparent: true,
        opacity: 0.22,
        roughness: 0.25,
        metalness: 0.0,
        side: THREE.DoubleSide,
      })
    );
    scene.add(shell);
    shellMeshes.push(shell);

    for (let i = 0; i < group.particles.length; i++) {
      const p = group.particles[i];
      const mat = new THREE.MeshStandardMaterial({
        color: pointColor,
        roughness: 0.35,
      });
      const mesh = new THREE.Mesh(sphereGeo, mat);
      mesh.scale.setScalar(params.particleRadius);
      mesh.position.copy(p.x);
      mesh.userData = { group, index: i };
      scene.add(mesh);
      particleMeshes.push(mesh);
    }
  }

  addGroupVisual(rigid, 0x9cff8f, 0xa5ff9a);
  addGroupVisual(soft, 0xffb15c, 0xffd4a3);
}
