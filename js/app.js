import * as THREE from '../vendor/three/three.module.min.js';
import { OrbitControls } from '../vendor/three/OrbitControls.js';
import { Simulation } from './sim/simulation.js';
import { DistanceConstraint } from './sim/constraints.js';
import { buildScene1 } from './scenes/scene1.js';
import { buildScene2 } from './scenes/scene2.js';
import { buildScene3 } from './scenes/scene3.js';

(() => {
  const statusEl = document.getElementById('status');
  let baseStatusText = '';
  let volumesStatusLine = '';

  const renderStatus = () => {
    if (!statusEl) return;
    const parts = [];
    if (baseStatusText) parts.push(baseStatusText);
    if (volumesStatusLine) parts.push(volumesStatusLine);
    statusEl.textContent = parts.join('\n');
  };
  const setStatus = (msg) => {
    baseStatusText = String(msg ?? '');
    renderStatus();
  };

  try {
    const canvas = document.createElement('canvas');
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
    if (!gl) {
      setStatus('Ошибка: WebGL недоступен в этом браузере.');
      return;
    }
  } catch (e) {
    setStatus('Ошибка инициализации окружения: ' + String(e));
    return;
  }

  const viewport = document.getElementById('viewport');
  if (!viewport) {
    setStatus('Ошибка: не найден #viewport');
    return;
  }

  const ui = {
    iterations: document.getElementById('iterations'),
    iterationsValue: document.getElementById('iterationsValue'),

    solverMethod: document.getElementById('solverMethod'),

    complianceEdges: document.getElementById('complianceEdges'),
    complianceEdgesValue: document.getElementById('complianceEdgesValue'),

    complianceVolume: document.getElementById('complianceVolume'),
    complianceVolumeValue: document.getElementById('complianceVolumeValue'),

    gravity: document.getElementById('gravity'),
    gravityValue: document.getElementById('gravityValue'),

    damping: document.getElementById('damping'),
    dampingValue: document.getElementById('dampingValue'),

    particleRadius: document.getElementById('particleRadius'),
    particleRadiusValue: document.getElementById('particleRadiusValue'),

    timeScale: document.getElementById('timeScale'),
    timeScaleValue: document.getElementById('timeScaleValue'),

    friction: document.getElementById('friction'),
    frictionValue: document.getElementById('frictionValue'),

    restitution: document.getElementById('restitution'),
    restitutionValue: document.getElementById('restitutionValue'),

    selfCollision: document.getElementById('selfCollision'),

    scene1: document.getElementById('scene1'),
    scene2: document.getElementById('scene2'),
    scene3: document.getElementById('scene3'),

    reset: document.getElementById('reset'),
    pause: document.getElementById('pause'),
    zeroVel: document.getElementById('zeroVel'),
  };

  const params = {
    dt: 1 / 60,
    solverMethod: 'xpbd',
    iterations: 16,
    complianceEdges: 0.00002,
    complianceVolume: 0.00005,
    gravity: 12.0,
    damping: 0.005,
    particleRadius: 0.05,
    timeScale: 1.0,
    friction: 0.5,
    restitution: 1,
    enableSelfCollision: true,

    activeScene: 1,
    floorY: -1.2,

    paused: false,
  };

  function bindRange(input, valueEl, format, onChange) {
    const update = () => {
      const v = Number(input.value);
      valueEl.textContent = format(v);
      onChange(v);
    };
    input.addEventListener('input', update);
    update();
  }

  // Defaults
  if (ui.solverMethod) ui.solverMethod.value = params.solverMethod;
  ui.iterations.value = String(params.iterations);
  ui.complianceEdges.value = String(params.complianceEdges);
  ui.complianceVolume.value = String(params.complianceVolume);
  ui.gravity.value = String(params.gravity);
  ui.damping.value = String(params.damping);
  ui.particleRadius.value = String(params.particleRadius);
  ui.timeScale.value = String(params.timeScale);
  ui.friction.value = String(params.friction);
  ui.restitution.value = String(params.restitution);
  ui.selfCollision.checked = params.enableSelfCollision;

  bindRange(ui.iterations, ui.iterationsValue, v => String(v), v => (params.iterations = v | 0));

  ui.solverMethod?.addEventListener('change', () => {
    params.solverMethod = ui.solverMethod.value;
    buildActiveScene();
  });
  bindRange(ui.complianceEdges, ui.complianceEdgesValue, v => v.toExponential(2), v => (params.complianceEdges = v));
  bindRange(ui.complianceVolume, ui.complianceVolumeValue, v => v.toExponential(2), v => (params.complianceVolume = v));
  bindRange(ui.gravity, ui.gravityValue, v => v.toFixed(1), v => (params.gravity = v));
  bindRange(ui.damping, ui.dampingValue, v => v.toFixed(4), v => (params.damping = v));
  bindRange(ui.particleRadius, ui.particleRadiusValue, v => v.toFixed(3), v => (params.particleRadius = v));
  bindRange(ui.timeScale, ui.timeScaleValue, v => v.toFixed(2), v => (params.timeScale = v));
  bindRange(ui.friction, ui.frictionValue, v => v.toFixed(2), v => (params.friction = v));
  bindRange(ui.restitution, ui.restitutionValue, v => v.toFixed(2), v => (params.restitution = v));
  ui.selfCollision.addEventListener('change', () => (params.enableSelfCollision = ui.selfCollision.checked));

  // --- Three.js setup ---
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1020);

  const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 200);
  camera.position.set(2.8, 1.6, 2.8);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  viewport.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  scene.add(new THREE.AmbientLight(0xffffff, 0.35));
  const dir = new THREE.DirectionalLight(0xffffff, 0.8);
  dir.position.set(3, 5, 2);
  scene.add(dir);

  const grid = new THREE.GridHelper(10, 20, 0x3a4b7a, 0x243154);
  grid.position.y = -1.2;
  scene.add(grid);

  const sim = new Simulation();

  let shellMeshes = [];
  let particleMeshes = [];
  let constraintLineMeshes = [];
  let constraintLineInfos = []; // { line, group, pairs }

  const constraintLineMaterial = new THREE.LineBasicMaterial({
    color: 0xff3b3b,
    transparent: true,
    opacity: 0.85,
  });

  // picking / dragging
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const dragPlane = new THREE.Plane();
  const dragTarget = new THREE.Vector3();
  let dragged = null; // { group, index }

  // keyboard movement
  const keysDown = new Set();
  const moveSpeed = 2.2; // units/sec

  function isTypingInInput() {
    const el = document.activeElement;
    if (!el) return false;
    const tag = String(el.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || tag === 'select';
  }

  function onKeyDown(ev) {
    if (isTypingInInput()) return;
    keysDown.add(ev.code);
    if (ev.code === 'Space') ev.preventDefault();
  }

  function onKeyUp(ev) {
    keysDown.delete(ev.code);
  }

  window.addEventListener('keydown', onKeyDown, { passive: false });
  window.addEventListener('keyup', onKeyUp);

  function setSize() {
    const w = viewport.clientWidth;
    const h = viewport.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  window.addEventListener('resize', setSize);
  setSize();

  function clearVisuals() {
    for (const m of particleMeshes) scene.remove(m);
    particleMeshes.length = 0;
    for (const m of shellMeshes) scene.remove(m);
    shellMeshes.length = 0;

    for (const l of constraintLineMeshes) scene.remove(l);
    constraintLineMeshes.length = 0;
    constraintLineInfos.length = 0;
  }

  function rebuildConstraintLines() {
    for (const l of constraintLineMeshes) scene.remove(l);
    constraintLineMeshes.length = 0;
    constraintLineInfos.length = 0;

    for (const group of sim.groups) {
      const pairs = [];
      for (const c of group.constraints) {
        if (c instanceof DistanceConstraint) pairs.push([c.i0, c.i1]);
      }
      if (pairs.length === 0) continue;

      const positions = new Float32Array(pairs.length * 2 * 3);
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

      const line = new THREE.LineSegments(geom, constraintLineMaterial);
      line.frustumCulled = false;
      scene.add(line);

      constraintLineMeshes.push(line);
      constraintLineInfos.push({ line, group, pairs });
    }
  }

  function updateConstraintLines() {
    for (const info of constraintLineInfos) {
      const attr = info.line.geometry.getAttribute('position');
      const arr = attr.array;
      let k = 0;
      const particles = info.group.particles;
      for (const [i0, i1] of info.pairs) {
        const a = particles[i0].x;
        const b = particles[i1].x;
        arr[k++] = a.x;
        arr[k++] = a.y;
        arr[k++] = a.z;
        arr[k++] = b.x;
        arr[k++] = b.y;
        arr[k++] = b.z;
      }
      attr.needsUpdate = true;
    }
  }

  function buildActiveScene() {
    if (params.activeScene === 2) {
      buildScene2({ sim, scene, params, setStatus, clearVisuals, shellMeshes, particleMeshes });
    } else if (params.activeScene === 3) {
      buildScene3({ sim, scene, params, setStatus, clearVisuals, shellMeshes, particleMeshes });
    } else {
      buildScene1({ sim, scene, params, setStatus, clearVisuals, shellMeshes, particleMeshes });
    }

    rebuildConstraintLines();
  }

  function updateMaterials() {
    for (const m of particleMeshes) {
      const { group, index } = m.userData;
      const p = group.particles[index];
      p.radius = params.particleRadius;
      m.scale.setScalar(params.particleRadius);
      if (dragged && dragged.mesh === m) {
        m.material.color.setHex(0xffe066);
      } else {
        if (p.invMass === 0) {
          m.material.color.setHex(0xff4b4b);
        } else if (group.kind === 'rigid') {
          m.material.color.setHex(0xa5ff9a);
        } else if (group.kind === 'soft') {
          m.material.color.setHex(0xffd4a3);
        } else {
          m.material.color.setHex(0xdbe6ff);
        }
      }
    }
  }

  ui.scene1.addEventListener('click', () => {
    params.activeScene = 1;
    buildActiveScene();
  });
  ui.scene2.addEventListener('click', () => {
    params.activeScene = 2;
    buildActiveScene();
  });
  ui.scene3?.addEventListener('click', () => {
    params.activeScene = 3;
    buildActiveScene();
  });

  ui.reset.addEventListener('click', () => buildActiveScene());
  ui.pause.addEventListener('click', () => {
    params.paused = !params.paused;
    ui.pause.textContent = params.paused ? 'Run' : 'Pause';
    baseStatusText = (baseStatusText || '').replace(/Запуск: .*/g, `Запуск: ${params.paused ? 'PAUSED' : 'RUN'}`);
    renderStatus();
  });

  ui.zeroVel?.addEventListener('click', () => {
    for (const group of sim.groups) {
      for (const p of group.particles) {
        p.xPrev.copy(p.x);
      }
    }
  });

  // --- Mouse picking ---
  function setPointerFromEvent(ev) {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -(((ev.clientY - rect.top) / rect.height) * 2 - 1);
  }

  function pick(ev) {
    setPointerFromEvent(ev);
    raycaster.setFromCamera(pointer, camera);
    const intersects = raycaster.intersectObjects(particleMeshes, false);
    if (intersects.length === 0) return null;
    return intersects[0].object;
  }

  function beginDrag(ev) {
    if (ev.button !== 0) return; // left only
    const obj = pick(ev);
    if (!obj) return;

    const { group, index } = obj.userData;
    const p = group.particles[index];

    dragged = { group, index, mesh: obj };
    dragged.mesh.material.color.setHex(0xffe066);

    const camDir = new THREE.Vector3();
    camera.getWorldDirection(camDir);
    dragPlane.setFromNormalAndCoplanarPoint(camDir, p.x);

    raycaster.setFromCamera(pointer, camera);
    raycaster.ray.intersectPlane(dragPlane, dragTarget);

    // Mark particle as kinematic while dragging
    p._savedInvMass = p.invMass;
    p.invMass = 0;
    p.xPrev.copy(p.x);
    p.x.copy(dragTarget);

    controls.enabled = false;
    ev.preventDefault();
  }

  function updateDrag(ev) {
    if (!dragged) return;
    setPointerFromEvent(ev);
    raycaster.setFromCamera(pointer, camera);
    if (raycaster.ray.intersectPlane(dragPlane, dragTarget)) {
      const p = dragged.group.particles[dragged.index];
      p.xPrev.copy(p.x);
      p.x.copy(dragTarget);
    }
  }

  function endDrag() {
    if (!dragged) return;
    const p = dragged.group.particles[dragged.index];
    p.invMass = p._savedInvMass;
    delete p._savedInvMass;

    dragged.mesh.material.color.setHex(0xdbe6ff);
    dragged = null;
    controls.enabled = true;
  }

  renderer.domElement.addEventListener('pointerdown', (ev) => {
    setPointerFromEvent(ev);
    beginDrag(ev);
  });
  window.addEventListener('pointermove', updateDrag);
  window.addEventListener('pointerup', endDrag);
  window.addEventListener('pointercancel', endDrag);

  let acc = 0;
  let last = performance.now();
  let volAcc = 0;

  try {
    buildActiveScene();
    const axes = new THREE.AxesHelper(1.0);
    scene.add(axes);
    animate();
  } catch (err) {
    console.error(err);
    setStatus('Runtime error:\n' + (err?.stack || String(err)));
  }

  function animate() {
    requestAnimationFrame(animate);

    const now = performance.now();
    let frameDt = (now - last) / 1000;
    last = now;

    frameDt = Math.min(frameDt, 0.05);

    // Camera movement
    {
      const forward = new THREE.Vector3();
      camera.getWorldDirection(forward);
      forward.y = 0;
      const fLen = forward.length();
      if (fLen > 1e-6) forward.multiplyScalar(1 / fLen);

      const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0)).normalize();
      const up = new THREE.Vector3(0, 1, 0);

      const move = new THREE.Vector3(0, 0, 0);
      if (keysDown.has('KeyW')) move.add(forward);
      if (keysDown.has('KeyS')) move.sub(forward);
      if (keysDown.has('KeyD')) move.add(right);
      if (keysDown.has('KeyA')) move.sub(right);
      if (keysDown.has('Space')) move.add(up);
      if (keysDown.has('ShiftLeft') || keysDown.has('ShiftRight')) move.sub(up);

      const mLen = move.length();
      if (mLen > 1e-6) {
        move.multiplyScalar((moveSpeed * frameDt) / mLen);
        camera.position.add(move);
        controls.target.add(move);
      }
    }

    controls.update();

    // Volume reporting (throttled)
    {
      volAcc += frameDt;
      if (volAcc >= 0.1) {
        volAcc = 0;

        const parts = [];
        for (let gi = 0; gi < sim.groups.length; gi++) {
          const group = sim.groups[gi];
          const v = computeGroupVolume(group);
          const name = group.kind || `group${gi}`;
          if (Number.isFinite(v)) parts.push(`${name}: ${v.toFixed(4)}`);
        }
        volumesStatusLine = parts.length ? `V: ${parts.join(' | ')}` : '';
        renderStatus();
      }
    }

    if (!params.paused) {
      acc += frameDt * params.timeScale;
      const fixedDt = params.dt;
      const maxSteps = 3;
      let steps = 0;
      while (acc >= fixedDt && steps < maxSteps) {
        sim.step(fixedDt, {
          solverMethod: params.solverMethod,
          iterations: params.iterations,
          complianceEdges: params.complianceEdges,
          complianceVolume: params.complianceVolume,
          gravity: params.gravity,
          damping: params.damping,

          enableFloorCollision: params.activeScene === 2,
          floorY: params.floorY,
          restitution: params.restitution,
          friction: params.friction,
          enableSelfCollision: params.activeScene === 2 && params.enableSelfCollision,
          selfCollisionRadiusScale: 1.0,
        });
        acc -= fixedDt;
        steps++;
      }
    }

    // update visuals
    if (sim.groups.length > 0) {
      for (const group of sim.groups) {
        group.syncRenderGeometry();
      }

      // deformable particle meshes
      for (const m of particleMeshes) {
        const { group, index } = m.userData;
        m.position.copy(group.particles[index].x);
      }

      for (const s of shellMeshes) {
        s.geometry.attributes.position.needsUpdate = true;
      }

      updateConstraintLines();

      updateMaterials();
    }

    renderer.render(scene, camera);
  }

  function computeGroupVolume(group) {
    const geom = group.renderGeometry;
    const map = group.renderIndexToParticle;
    if (!geom || !geom.index || !map) return NaN;

    const idx = geom.index.array;
    let V = 0;

    const crossX = new THREE.Vector3();
    for (let t = 0; t < idx.length; t += 3) {
      const vi0 = idx[t + 0];
      const vi1 = idx[t + 1];
      const vi2 = idx[t + 2];

      const p0 = group.particles[map[vi0]];
      const p1 = group.particles[map[vi1]];
      const p2 = group.particles[map[vi2]];
      if (!p0 || !p1 || !p2) continue;

      crossX.crossVectors(p1.x, p2.x);
      V += p0.x.dot(crossX);
    }

    return V / 6.0;
  }
})();
