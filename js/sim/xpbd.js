import * as THREE from '../../vendor/three/three.module.min.js';
import { projectFloorPositions, applyFloorVelocity, projectSelfCollisions } from './collisions.js';

export class Particle {
  constructor(position, invMass, radius) {
    this.x = position.clone();
    this.xPrev = position.clone();
    this.invMass = invMass;
    this.radius = radius;
  }
}

export class Group {
  constructor() {
    this.particles = [];
    this.constraints = [];

    this.renderGeometry = null;
    this._renderPositionAttr = null;
    this._renderPositions = null;
    this._renderIndexToParticle = null;
  }

  addParticle(p) {
    this.particles.push(p);
    return this.particles.length - 1;
  }

  addConstraint(c) {
    this.constraints.push(c);
    return c;
  }

  setRenderGeometry(geometry, indexToParticle) {
    this.renderGeometry = geometry;
    this._renderPositionAttr = geometry.getAttribute('position');
    this._renderPositions = this._renderPositionAttr.array;
    this._renderIndexToParticle = indexToParticle;

    this.syncRenderGeometry();
  }

  syncRenderGeometry() {
    if (!this.renderGeometry) return;
    const arr = this._renderPositions;
    const map = this._renderIndexToParticle;
    for (let vi = 0; vi < map.length; vi++) {
      const pi = map[vi];
      const x = this.particles[pi].x;
      const base = vi * 3;
      arr[base + 0] = x.x;
      arr[base + 1] = x.y;
      arr[base + 2] = x.z;
    }
    this._renderPositionAttr.needsUpdate = true;
    this.renderGeometry.computeVertexNormals();
  }

  resetLambdas() {
    for (const c of this.constraints) c.resetLambda?.();
  }

  solve(dt, settings) {
    this.resetLambdas();

    for (let it = 0; it < settings.iterations; it++) {
      for (const c of this.constraints) c.project(this, dt, settings);
    }
  }
}

export class Simulation {
  constructor() {
    this.groups = [];
  }

  reset() {
    this.groups = [];
  }

  addGroup(group) {
    this.groups.push(group);
    return group;
  }

  step(dt, settings) {
    const g = new THREE.Vector3(0, -settings.gravity, 0);

    // integrate (verlet style)
    for (const group of this.groups) {
      for (const p of group.particles) {
        if (p.invMass === 0) {
          p.xPrev.copy(p.x);
          continue;
        }

        const xTemp = p.x.clone();
        const v = p.x.clone().sub(p.xPrev).multiplyScalar(1.0 - settings.damping);
        p.x.add(v).addScaledVector(g, dt * dt);
        p.xPrev.copy(xTemp);
      }
    }

    // Reset XPBD lambdas once per time-step
    for (const group of this.groups) group.resetLambdas();

    for (let it = 0; it < settings.iterations; it++) {
      // structural constraints
      for (const group of this.groups) {
        for (const c of group.constraints) c.project(group, dt, settings);
      }

      // collision projection (position level)
      if (settings.enableFloorCollision) {
        projectFloorPositions(this.groups, settings.floorY);
      }
      if (settings.enableSelfCollision) {
        projectSelfCollisions(this.groups, settings.selfCollisionRadiusScale);
      }
    }

    // velocity-level effects for floor contacts (restitution + friction)
    if (settings.enableFloorCollision) {
      applyFloorVelocity(this.groups, dt, settings.floorY, settings.restitution, settings.friction);
    }
  }
}
