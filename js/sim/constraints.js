import * as THREE from '../../vendor/three/three.module.min.js';

export class DistanceConstraint {
  constructor(i0, i1, restLength, complianceSelector) {
    this.i0 = i0;
    this.i1 = i1;
    this.restLength = restLength;
    this._getCompliance = complianceSelector;
    this.lambda = 0;
  }

  resetLambda() {
    this.lambda = 0;
  }

  project(group, dt, settings) {
    const p0 = group.particles[this.i0];
    const p1 = group.particles[this.i1];

    const w0 = p0.invMass;
    const w1 = p1.invMass;
    if (w0 + w1 === 0) return;

    const d = p0.x.clone().sub(p1.x);
    const len = d.length();
    if (len < 1e-9) return;

    const C = len - this.restLength;
    const n = d.multiplyScalar(1 / len);

    const alpha = (this._getCompliance(settings) || 0) / (dt * dt);
    const denom = w0 + w1 + alpha;
    const deltaLambda = (-C - alpha * this.lambda) / denom;
    this.lambda += deltaLambda;

    p0.x.addScaledVector(n, w0 * deltaLambda);
    p1.x.addScaledVector(n, -w1 * deltaLambda);
  }
}

export class VolumeConstraint {
  constructor(triangles, restVolume, complianceSelector) {
    this.triangles = triangles;
    this.restVolume = restVolume;
    this._getCompliance = complianceSelector;
    this.lambda = 0;
  }

  resetLambda() {
    this.lambda = 0;
  }

  static computeVolume(particles, triangles) {
    let V = 0;
    const cross = new THREE.Vector3();
    for (let t = 0; t < triangles.length; t++) {
      const [i0, i1, i2] = triangles[t];
      const x0 = particles[i0].x;
      const x1 = particles[i1].x;
      const x2 = particles[i2].x;
      cross.crossVectors(x1, x2);
      V += x0.dot(cross);
    }
    return V / 6.0;
  }

  project(group, dt, settings) {
    const particles = group.particles;
    const triangles = this.triangles;

    const V = VolumeConstraint.computeVolume(particles, triangles);
    const C = V - this.restVolume;
    if (!Number.isFinite(C)) return;

    const grads = new Array(particles.length);
    for (let i = 0; i < grads.length; i++) grads[i] = new THREE.Vector3(0, 0, 0);

    const tmp = new THREE.Vector3();
    for (let t = 0; t < triangles.length; t++) {
      const [i0, i1, i2] = triangles[t];
      const x0 = particles[i0].x;
      const x1 = particles[i1].x;
      const x2 = particles[i2].x;

      tmp.crossVectors(x1, x2).multiplyScalar(1 / 6);
      grads[i0].add(tmp);

      tmp.crossVectors(x2, x0).multiplyScalar(1 / 6);
      grads[i1].add(tmp);

      tmp.crossVectors(x0, x1).multiplyScalar(1 / 6);
      grads[i2].add(tmp);
    }

    let sum = 0;
    for (let i = 0; i < particles.length; i++) {
      const w = particles[i].invMass;
      if (w === 0) continue;
      sum += w * grads[i].lengthSq();
    }

    const alpha = (this._getCompliance(settings) || 0) / (dt * dt);
    const denom = sum + alpha;
    if (denom < 1e-12) return;

    const deltaLambda = (-C - alpha * this.lambda) / denom;
    this.lambda += deltaLambda;

    for (let i = 0; i < particles.length; i++) {
      const p = particles[i];
      const w = p.invMass;
      if (w === 0) continue;
      p.x.addScaledVector(grads[i], w * deltaLambda);
    }
  }
}
