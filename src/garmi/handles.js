// Draggable end-effector handles: a translucent cube floats at each gripper
// target, highlights on hover, and drags in a camera-parallel plane through
// the handle (orbit the camera to move in a different plane). While a handle
// is grabbed, OrbitControls is disabled; pointer events make this work for
// both mouse and touch. A thin leash line appears when the arm can't reach
// the target.
import * as THREE from 'three';

const SIZE = 0.085;
const OPACITY = { idle: 0.25, hover: 0.45, drag: 0.6 };

export class EEHandles {
  constructor({ scene, camera, dom, controls, arms, data }) {
    this.camera = camera;
    this.dom = dom;
    this.controls = controls;
    this.data = data;
    this.raycaster = new THREE.Raycaster();
    this.plane = new THREE.Plane();
    this.hit = new THREE.Vector3();
    this.ndc = new THREE.Vector2();
    this.dragging = null;
    this.hovered = null;

    this.entries = arms.map((arm) => {
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(SIZE, SIZE, SIZE),
        new THREE.MeshStandardMaterial({
          color: 0x4a9eea, transparent: true, opacity: OPACITY.idle,
          roughness: 0.35, metalness: 0, depthTest: false, depthWrite: false,
        }),
      );
      mesh.renderOrder = 10;
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(mesh.geometry),
        new THREE.LineBasicMaterial({
          color: 0x9fc4e8, transparent: true, opacity: 0.7,
          depthTest: false, depthWrite: false,
        }),
      );
      edges.renderOrder = 11;
      mesh.add(edges);
      scene.add(mesh);

      const leash = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
        new THREE.LineDashedMaterial({
          color: 0x4a9eea, transparent: true, opacity: 0.8,
          dashSize: 0.02, gapSize: 0.015, depthTest: false,
        }),
      );
      leash.renderOrder = 9;
      leash.visible = false;
      scene.add(leash);
      return { arm, mesh, leash };
    });

    dom.addEventListener('pointerdown', (e) => this.#down(e));
    dom.addEventListener('pointermove', (e) => this.#move(e));
    dom.addEventListener('pointerup', (e) => this.#up(e));
    dom.addEventListener('pointercancel', (e) => this.#up(e));
  }

  #castRay(e) {
    const r = this.dom.getBoundingClientRect();
    this.ndc.set(
      ((e.clientX - r.left) / r.width) * 2 - 1,
      -((e.clientY - r.top) / r.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.ndc, this.camera);
  }

  #pick(e) {
    this.#castRay(e);
    for (const entry of this.entries) {
      if (this.raycaster.intersectObject(entry.mesh, false).length) return entry;
    }
    return null;
  }

  #down(e) {
    const entry = this.#pick(e);
    if (!entry) return;
    this.dragging = entry;
    try { this.dom.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    // OrbitControls saw this pointerdown first, but it checks `enabled` on
    // every subsequent move, so flipping it here is sufficient.
    this.controls.enabled = false;
    const dir = new THREE.Vector3();
    this.camera.getWorldDirection(dir);
    this.plane.setFromNormalAndCoplanarPoint(dir, entry.mesh.position);
    entry.mesh.material.opacity = OPACITY.drag;
  }

  #move(e) {
    if (this.dragging) {
      this.#castRay(e);
      if (this.raycaster.ray.intersectPlane(this.plane, this.hit)) {
        this.dragging.arm.setWorldTarget(this.data, [this.hit.x, this.hit.y, Math.max(0.05, this.hit.z)]);
      }
      return;
    }
    const entry = this.#pick(e);
    if (entry !== this.hovered) {
      if (this.hovered) this.hovered.mesh.material.opacity = OPACITY.idle;
      this.hovered = entry;
      if (entry) entry.mesh.material.opacity = OPACITY.hover;
      this.dom.style.cursor = entry ? 'grab' : '';
    }
  }

  #up(e) {
    if (!this.dragging) return;
    this.dragging.mesh.material.opacity = OPACITY.idle;
    this.dragging = null;
    this.controls.enabled = true;
    if (this.dom.hasPointerCapture?.(e.pointerId)) this.dom.releasePointerCapture(e.pointerId);
  }

  // Called once per rendered frame.
  update() {
    for (const { arm, mesh, leash } of this.entries) {
      const t = arm.worldTarget(this.data);
      mesh.position.set(t[0], t[1], t[2]);
      const p = arm.eePos(this.data);
      const far = Math.hypot(t[0] - p[0], t[1] - p[1], t[2] - p[2]) > 0.02;
      leash.visible = arm.active && far;
      if (leash.visible) {
        const pos = leash.geometry.attributes.position;
        pos.setXYZ(0, t[0], t[1], t[2]);
        pos.setXYZ(1, p[0], p[1], p[2]);
        pos.needsUpdate = true;
        leash.computeLineDistances();
      }
    }
  }
}
