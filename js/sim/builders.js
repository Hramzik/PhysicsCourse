import { Group } from './xpbd.js';
import { DistanceConstraint, VolumeConstraint } from './constraints.js';
import { makeIcoShell, makeCubeShell, uniqueEdgesFromIndexedGeometry, indexedToParticles } from './geometry.js';

function buildNeighborEdgeSet(edges) {
  const set = new Set();
  for (const [i0, i1] of edges) {
    const key = i0 < i1 ? i0 + ',' + i1 : i1 + ',' + i0;
    set.add(key);
  }
  return set;
}

export function buildDeformableBody({ radius, detail, center, particleRadius }) {
  const { geometry } = makeIcoShell(radius, detail);

  const index = geometry.index.array;
  const topo = uniqueEdgesFromIndexedGeometry(index);

  const { particles, indexToParticle } = indexedToParticles(geometry, center, 1.0, particleRadius);

  const group = new Group();
  group.particles = particles;
  group.neighborEdges = buildNeighborEdgeSet(topo.edges);

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
  group.neighborEdges = buildNeighborEdgeSet(topo.edges);

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

export function buildSoftBall({ radius, detail, center, particleRadius }) {
  const { geometry } = makeIcoShell(radius, detail);

  const index = geometry.index.array;
  const topo = uniqueEdgesFromIndexedGeometry(index);

  const { particles, indexToParticle } = indexedToParticles(geometry, center, 1.0, particleRadius);

  const group = new Group();
  group.particles = particles;
  group.neighborEdges = buildNeighborEdgeSet(topo.edges);

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

  const { particles, indexToParticle } = indexedToParticles(geometry, center, 1.0, particleRadius);

  const group = new Group();
  group.particles = particles;
  group.neighborEdges = new Set();

  for (let i0 = 0; i0 < particles.length; i0++) {
    for (let i1 = i0 + 1; i1 < particles.length; i1++) {
      const rest = particles[i0].x.distanceTo(particles[i1].x);
      group.addConstraint(new DistanceConstraint(i0, i1, rest, () => 0));
      group.neighborEdges.add(i0 + ',' + i1);
    }
  }

  const renderGeometry = geometry.clone();
  group.setRenderGeometry(renderGeometry, indexToParticle);

  group.renderGeometry = renderGeometry;
  group.particleRadius = particleRadius;

  return group;
}
