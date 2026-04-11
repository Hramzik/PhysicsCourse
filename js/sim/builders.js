import { Group } from './xpbd.js';
import { DistanceConstraint, VolumeConstraint } from './constraints.js';
import { makeIcoShell, makeCubeShell, uniqueEdgesFromIndexedGeometry, indexedToParticles } from './geometry.js';

export function buildDeformableBody({ radius, detail, center, particleRadius }) {
  const { geometry } = makeIcoShell(radius, detail);

  const index = geometry.index.array;
  const topo = uniqueEdgesFromIndexedGeometry(index);

  const { particles, indexToParticle } = indexedToParticles(geometry, center, 1.0, particleRadius);

  const group = new Group();
  group.particles = particles;

  for (const [i0, i1] of topo.edges) {
    const rest = particles[i0].x.distanceTo(particles[i1].x);
    group.addConstraint(new DistanceConstraint(i0, i1, rest, (s) => s.complianceEdges));
  }

  const restVolume = VolumeConstraint.computeVolume(particles, topo.triangles);
  group.addConstraint(new VolumeConstraint(topo.triangles, restVolume, (s) => s.complianceVolume));

  const renderGeometry = geometry.clone();
  group.setRenderGeometry(renderGeometry, indexToParticle);

  group.renderGeometry = renderGeometry;
  group.particleRadius = particleRadius;

  return group;
}

export function buildDeformableCubeBody({ size, center, particleRadius }) {
  const { geometry } = makeCubeShell(size);

  const index = geometry.index.array;
  const topo = uniqueEdgesFromIndexedGeometry(index);

  const { particles, indexToParticle } = indexedToParticles(geometry, center, 1.0, particleRadius);

  const group = new Group();
  group.particles = particles;

  for (const [i0, i1] of topo.edges) {
    const rest = particles[i0].x.distanceTo(particles[i1].x);
    group.addConstraint(new DistanceConstraint(i0, i1, rest, (s) => s.complianceEdges));
  }

  const restVolume = VolumeConstraint.computeVolume(particles, topo.triangles);
  group.addConstraint(new VolumeConstraint(topo.triangles, restVolume, (s) => s.complianceVolume));

  const renderGeometry = geometry.clone();
  group.setRenderGeometry(renderGeometry, indexToParticle);
  group.renderGeometry = renderGeometry;
  group.particleRadius = particleRadius;

  return group;
}

export function buildRigidBall({ radius, detail, center, particleRadius }) {
  const { geometry } = makeIcoShell(radius, detail);

  const index = geometry.index.array;
  const topo = uniqueEdgesFromIndexedGeometry(index);

  const { particles, indexToParticle } = indexedToParticles(geometry, center, 1.0, particleRadius);

  const group = new Group();
  group.particles = particles;

  for (const [i0, i1] of topo.edges) {
    const rest = particles[i0].x.distanceTo(particles[i1].x);
    group.addConstraint(new DistanceConstraint(i0, i1, rest, () => 0));
  }

  const renderGeometry = geometry.clone();
  group.setRenderGeometry(renderGeometry, indexToParticle);

  group.renderGeometry = renderGeometry;
  group.particleRadius = particleRadius;

  return group;
}
