import * as THREE from '../../vendor/three/three.module.min.js';
import { mergeVertices } from '../../vendor/three/BufferGeometryUtils.js';
import { Particle } from './simulation.js';

export function uniqueEdgesFromIndexedGeometry(indexArray) {
  const edges = new Map();
  const triangles = [];

  for (let i = 0; i < indexArray.length; i += 3) {
    const a = indexArray[i + 0];
    const b = indexArray[i + 1];
    const c = indexArray[i + 2];
    triangles.push([a, b, c]);

    addEdge(a, b);
    addEdge(b, c);
    addEdge(c, a);
  }

  function addEdge(i0, i1) {
    const lo = Math.min(i0, i1);
    const hi = Math.max(i0, i1);
    const key = lo + ',' + hi;
    if (!edges.has(key)) edges.set(key, [lo, hi]);
  }

  return { edges: Array.from(edges.values()), triangles };
}

export function makeIcoShell(radius, detail) {
  // For XPBD constraints we need shared vertices => indexed geometry.
  const base = new THREE.IcosahedronGeometry(radius, detail);

  // For polyhedra, non-indexed geometry often has per-face normals, so identical
  // positions won't merge unless we remove normals first.
  base.deleteAttribute('normal');

  const geometry = mergeVertices(base, 1e-6);
  geometry.computeVertexNormals();
  base.dispose();

  if (!geometry.index) {
    throw new Error('makeIcoShell: failed to create indexed geometry');
  }

  return { geometry, indexed: true };
}

export function makeDodecaShell(radius, detail = 0) {
  const base = new THREE.DodecahedronGeometry(radius, detail);

  base.deleteAttribute('normal');
  const geometry = mergeVertices(base, 1e-6);
  geometry.computeVertexNormals();
  base.dispose();

  if (!geometry.index) {
    throw new Error('makeDodecaShell: failed to create indexed geometry');
  }

  return { geometry, indexed: true };
}

export function makeCubeShell(size) {
  const h = size * 0.5;
  const positions = new Float32Array([
    -h, -h, -h, // 0
     h, -h, -h, // 1
     h,  h, -h, // 2
    -h,  h, -h, // 3
    -h, -h,  h, // 4
     h, -h,  h, // 5
     h,  h,  h, // 6
    -h,  h,  h, // 7
  ]);

  // 12 triangles
  const indices = new Uint16Array([
    0, 1, 2, 0, 2, 3, // -Z
    4, 6, 5, 4, 7, 6, // +Z
    0, 3, 7, 0, 7, 4, // -X
    1, 5, 6, 1, 6, 2, // +X
    0, 4, 5, 0, 5, 1, // -Y
    3, 2, 6, 3, 6, 7, // +Y
  ]);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  geometry.computeVertexNormals();

  return { geometry, indexed: true };
}

export function indexedToParticles(geometry, center, invMass, particleRadius) {
  const posAttr = geometry.getAttribute('position');
  const indexToParticle = [];

  for (let i = 0; i < posAttr.count; i++) indexToParticle.push(i);

  const particles = [];
  for (let i = 0; i < posAttr.count; i++) {
    const x = new THREE.Vector3(
      posAttr.getX(i) + center.x,
      posAttr.getY(i) + center.y,
      posAttr.getZ(i) + center.z
    );
    particles.push(new Particle(x, invMass, particleRadius));
  }

  return { particles, indexToParticle };
}
