// GARMI in the browser: MuJoCo (official WASM bindings) + three.js.
//
// Pipeline: fetch the slimmed model files -> place them in a MuJoCo VFS ->
// compile scene.xml -> mirror the model into three.js -> fixed-timestep
// physics with a twist-teleoperated mecanum base.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import loadMujoco from '@mujoco/mujoco';

import { MjThreeScene } from './mjRender.js';
import { BaseController, VMAX, WMAX } from './teleop.js';
import { StickPad } from './joystick.js';
import './garmi.css';

const ASSET_ROOT = `${import.meta.env.BASE_URL}garmi-sim/`;
const SLOW_TIMESTEP = 0.004; // fallback if the machine can't hold 500 Hz

const $ = (id) => document.getElementById(id);
const overlay = $('overlay');
const overlayMsg = $('overlay-msg');
const overlayBar = $('overlay-bar');

const stderrTail = [];

async function fetchAssets() {
  const manifest = await (await fetch(`${ASSET_ROOT}manifest.json`)).json();
  const files = manifest.files;
  // Meshes are stored pre-gzipped (GitHub Pages doesn't compress .obj/.stl);
  // inflate them here and hand MuJoCo the logical file name.
  const gzipped = new Set(manifest.gzipped ?? []);
  let done = 0;
  let bytes = 0;
  const buffers = await Promise.all(files.map(async (path) => {
    const gz = gzipped.has(path);
    const res = await fetch(ASSET_ROOT + path + (gz ? '.gz' : ''));
    if (!res.ok) throw new Error(`${path}: HTTP ${res.status}`);
    let buf = new Uint8Array(await res.arrayBuffer());
    // Decompress only if the payload is actually gzip — some servers (e.g.
    // Vite dev) serve .gz files with Content-Encoding and the browser has
    // already inflated them; GitHub Pages hands over the raw bytes.
    if (gz && buf[0] === 0x1f && buf[1] === 0x8b) {
      const stream = new Blob([buf]).stream().pipeThrough(new DecompressionStream('gzip'));
      buf = new Uint8Array(await new Response(stream).arrayBuffer());
    }
    done += 1;
    bytes += buf.length;
    setProgress(`Downloading model — ${done}/${files.length} files, ${(bytes / 1e6).toFixed(1)} MB`,
      done / files.length);
    return [path, buf];
  }));
  return { scene: manifest.scene, buffers };
}

function setProgress(msg, frac = null) {
  overlayMsg.textContent = msg;
  if (frac !== null) overlayBar.style.width = `${Math.round(frac * 100)}%`;
}

function fail(err) {
  console.error(err, stderrTail);
  overlay.classList.add('error');
  overlayMsg.innerHTML = `Simulation failed to start.<br><code>${err}</code>` +
    (stderrTail.length ? `<br><code>${stderrTail.slice(-3).join('<br>')}</code>` : '');
}

// Let the overlay text paint before the long synchronous compile. rAF alone
// deadlocks in hidden/background tabs, so race it with a timeout.
const nextPaint = () => new Promise((r) => {
  requestAnimationFrame(() => setTimeout(r));
  setTimeout(r, 80);
});

async function main() {
  // --- Load engine + model ------------------------------------------------
  setProgress('Starting physics engine…');
  const [mujoco, { scene: sceneFile, buffers }] = await Promise.all([
    loadMujoco({
      print: (t) => console.log('[mujoco]', t),
      printErr: (t) => { stderrTail.push(t); console.warn('[mujoco]', t); },
    }),
    fetchAssets(),
  ]);

  setProgress('Compiling model…', 1);
  await nextPaint();

  const vfs = new mujoco.MjVFS();
  for (const [path, buf] of buffers) vfs.addBuffer(path, buf);
  const model = mujoco.MjModel.from_xml_path(sceneFile, vfs);
  if (!model) throw new Error('model compilation failed');
  const data = new mujoco.MjData(model);

  mujoco.mj_resetDataKeyframe(model, data, 0); // 'home' keyframe
  data.ctrl.set(model.key_ctrl.subarray(0, model.nu)); // hold arms/lift/head
  mujoco.mj_forward(model, data);

  // --- three.js scene -----------------------------------------------------
  const stage = $('stage');
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  stage.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.55;

  const hemi = new THREE.HemisphereLight(0xffffff, 0x667788, 0.5);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = sun.shadow.camera.bottom = -3;
  sun.shadow.camera.right = sun.shadow.camera.top = 3;
  sun.shadow.camera.near = 0.5;
  sun.shadow.camera.far = 15;
  sun.shadow.bias = -0.0001;
  sun.shadow.normalBias = 0.02; // suppresses acne on the flat base covers
  scene.add(sun, sun.target);

  const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 120);
  camera.up.set(0, 0, 1); // MuJoCo is z-up
  camera.position.set(2.8, -2.4, 1.9);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.8);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = 1.5;
  controls.minDistance = 1.2;
  controls.maxDistance = 10;

  const mjScene = new MjThreeScene(mujoco, model, data, scene);

  function resize() {
    const w = stage.clientWidth;
    const h = stage.clientHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  resize();

  // --- Teleop + UI state ----------------------------------------------------
  const controller = new BaseController(mujoco, model);
  const ctrl = data.ctrl;
  const act = (name) => model.actuator(name).id;
  const actuators = {
    headPan: act('head_pan'),
    headTilt: act('head_tilt'),
    lift: act('lift'),
    gripL: act('arm_0_gripper'),
    gripR: act('arm_1_gripper'),
  };

  const keys = new Set();
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    if (e.code === 'Space') { togglePause(); e.preventDefault(); return; }
    if (e.code === 'KeyR') { reset(); return; }
    keys.add(e.code);
    if (e.code.startsWith('Arrow')) e.preventDefault();
  });
  window.addEventListener('keyup', (e) => keys.delete(e.code));
  window.addEventListener('blur', () => keys.clear());

  const movePad = new StickPad($('pad-move'));
  const rotPad = new StickPad($('pad-rot'), { horizontalOnly: true });

  function desiredTwist() {
    const key = (a, b) => (keys.has(a) ? 1 : 0) - (keys.has(b) ? 1 : 0);
    const vx = key('KeyW', 'KeyS') + key('ArrowUp', 'ArrowDown') + movePad.y;
    const vy = key('KeyA', 'KeyD') + key('ArrowLeft', 'ArrowRight') - movePad.x;
    const wz = key('KeyQ', 'KeyE') - rotPad.x;
    const c = (v) => Math.max(-1, Math.min(1, v));
    return [c(vx) * VMAX, c(vy) * VMAX, c(wz) * WMAX];
  }

  // Panel bindings.
  const bindSlider = (id, actuator) => {
    $(id).addEventListener('input', (e) => { ctrl[actuators[actuator]] = +e.target.value; });
  };
  bindSlider('s-pan', 'headPan');
  bindSlider('s-tilt', 'headTilt');
  bindSlider('s-lift', 'lift');
  $('s-grip').addEventListener('input', (e) => {
    ctrl[actuators.gripL] = +e.target.value;
    ctrl[actuators.gripR] = +e.target.value;
  });
  $('t-loop').addEventListener('change', (e) => { controller.closedLoop = e.target.checked; });
  let follow = true;
  $('t-follow').addEventListener('change', (e) => { follow = e.target.checked; });

  let paused = false;
  function togglePause() {
    paused = !paused;
    $('b-pause').textContent = paused ? 'Resume' : 'Pause';
  }
  $('b-pause').addEventListener('click', togglePause);

  function reset() {
    mujoco.mj_resetDataKeyframe(model, data, 0);
    data.ctrl.set(model.key_ctrl.subarray(0, model.nu));
    mujoco.mj_forward(model, data);
    for (const [id, val] of [['s-pan', 0], ['s-tilt', 0], ['s-lift', 0], ['s-grip', 255]]) {
      $(id).value = val;
    }
  }
  $('b-reset').addEventListener('click', reset);

  // --- Main loop ------------------------------------------------------------
  const opt = model.opt;
  let timestep = opt.timestep;
  const stats = $('stats');
  let tPrev = performance.now();
  let accum = 0;
  let fpsEma = 60;
  let rtEma = 1;
  let slowFrames = 0;

  function frame(tNow) {
    requestAnimationFrame(frame);
    const dtWall = Math.min((tNow - tPrev) / 1000, 0.1);
    tPrev = tNow;
    if (document.hidden) { accum = 0; return; }

    if (!paused) {
      controller.apply(data, desiredTwist(), dtWall);
      accum += dtWall;
      const maxSteps = Math.ceil(0.034 / timestep); // ≤ two frames of catch-up
      let steps = Math.floor(accum / timestep);
      let stepped = 0;
      if (steps > maxSteps) { steps = maxSteps; accum = 0; }
      const tBudget = performance.now() + 26; // keep the UI responsive
      for (; stepped < steps && performance.now() < tBudget; stepped++) {
        mujoco.mj_step(model, data);
      }
      accum = Math.max(0, accum - stepped * timestep);

      // Sustained overload -> halve the physics rate once (implicitfast is
      // comfortably stable at 4 ms for this model).
      rtEma += 0.05 * ((stepped * timestep) / Math.max(dtWall, 1e-3) - rtEma);
      slowFrames = rtEma < 0.85 ? slowFrames + 1 : 0;
      if (slowFrames > 90 && timestep < SLOW_TIMESTEP) {
        timestep = SLOW_TIMESTEP;
        opt.timestep = SLOW_TIMESTEP;
        console.warn(`[garmi] physics timestep relaxed to ${SLOW_TIMESTEP}s`);
        slowFrames = 0;
      }
    }

    // Camera follows the base (keeps your orbit offset).
    if (follow) {
      const [bx, by] = controller.basePosition(data);
      const target = new THREE.Vector3(bx, by, 0.8);
      const delta = target.sub(controls.target)
        .multiplyScalar(1 - Math.exp(-5 * dtWall));
      controls.target.add(delta);
      camera.position.add(delta);
    }
    sun.position.set(controls.target.x + 2, controls.target.y + 1.5, 4.5);
    sun.target.position.set(controls.target.x, controls.target.y, 0);

    mjScene.update();
    controls.update();
    renderer.render(scene, camera);

    fpsEma += 0.05 * (1 / Math.max(dtWall, 1e-3) - fpsEma);
    stats.textContent = `${fpsEma.toFixed(0)} fps · ${rtEma.toFixed(2)}× realtime`;
  }

  overlay.classList.add('hidden');
  // Console access for tinkering / debugging (renderOnce works while paused).
  const renderOnce = () => {
    mujoco.mj_forward(model, data);
    mjScene.update();
    renderer.render(scene, camera);
  };
  window.__garmi = {
    mujoco, model, data, controller, keys, desiredTwist,
    renderer, scene, camera, controls, renderOnce,
  };
  requestAnimationFrame((t) => { tPrev = t; requestAnimationFrame(frame); });
}

main().catch(fail);
