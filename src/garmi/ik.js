// Differential inverse kinematics for one GARMI arm.
//
// Each control tick: position Jacobian at the hand (mj_jacBody), damped
// least-squares step toward the drag target (3x3 solve), nullspace bias
// toward the home pose, clamp to joint ranges, and write the result into the
// arm's existing joint position actuators -- the fr3 servos remain the only
// thing applying torques, so arm behavior stays as tuned upstream.
//
// Targets are stored in the mobile base's frame, so a dragged hand rides
// along when the robot drives.

// Runs at the physics rate (call step() every mj_step): the fr3 position
// servos are stiff (kp up to 4500 with 87 Nm force saturation), so the servo
// targets must move as smooth ramps -- per-frame target jumps make the arms
// ring. Joint-velocity limiting plus a short low-pass on the drag point keep
// the commanded trajectory inside what the actuators can track.
const LAMBDA2 = 0.01;    // DLS damping (squared)
const STEP = 0.5;        // fraction of the DLS step applied per update
const NS_GAIN = 0.05;    // nullspace pull toward home [1/s]
const LEASH = 0.05;      // max Cartesian error consumed per update [m]
const WINDUP = 0.15;     // max |qTarget - q| per joint [rad]
const QDOT_MAX = 2.0;    // max servo-target speed per joint [rad/s]
const TARGET_TAU = 0.08; // low-pass time constant for the drag point [s]

const scalar = (v) => (typeof v === 'number' ? v : Number(v[0]));

// Rotate v by quaternion q = (w,x,y,z); inv=true applies the inverse rotation.
function rotate(v, q, inv = false) {
  const s = inv ? -1 : 1;
  const w = q[0], x = s * q[1], y = s * q[2], z = s * q[3];
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}

export class ArmIK {
  constructor(mujoco, model, data, prefix, baseQposAdr) {
    this.mujoco = mujoco;
    this.model = model;
    this.nv = model.nv;
    this.handId = model.body(`${prefix}_hand`).id;
    const joints = [1, 2, 3, 4, 5, 6, 7].map((k) => model.jnt(`${prefix}_joint${k}`));
    this.dofs = joints.map((j) => scalar(j.dofadr));
    this.qadrs = joints.map((j) => scalar(j.qposadr));
    this.ranges = joints.map((j) => Array.from(j.range));
    this.actIds = [1, 2, 3, 4, 5, 6, 7].map((k) => model.actuator(`${prefix}_joint${k}`).id);
    this.baseQposAdr = baseQposAdr;
    this.jacp = new mujoco.DoubleBuffer(3 * this.nv);
    this.jacr = new mujoco.DoubleBuffer(3 * this.nv);
    this.qHome = this.qadrs.map((a) => data.qpos[a]); // home keyframe is applied
    this.qTarget = [...this.qHome];
    // Always hold the Cartesian target (initially the home EE pose, a no-op),
    // so the arm folds to track its handle as the lift raises the shoulders --
    // not only after the handle has been dragged once.
    this.active = true;
    this.targetLocal = this.#toLocal(data, this.eePos(data));
    this.rawTargetLocal = [...this.targetLocal];
  }

  eePos(data) {
    const i = 3 * this.handId;
    return [data.xpos[i], data.xpos[i + 1], data.xpos[i + 2]];
  }

  #basePose(data) {
    const a = this.baseQposAdr, q = data.qpos;
    return [[q[a], q[a + 1], q[a + 2]], [q[a + 3], q[a + 4], q[a + 5], q[a + 6]]];
  }

  #toLocal(data, world) {
    const [p, q] = this.#basePose(data);
    return rotate([world[0] - p[0], world[1] - p[1], world[2] - p[2]], q, true);
  }

  worldTarget(data) {
    const [p, q] = this.#basePose(data);
    const v = rotate(this.targetLocal, q);
    return [v[0] + p[0], v[1] + p[1], v[2] + p[2]];
  }

  setWorldTarget(data, world) {
    this.rawTargetLocal = this.#toLocal(data, world);
    this.active = true;
  }

  reset(data) {
    this.active = true;
    this.qTarget = [...this.qHome];
    this.targetLocal = this.#toLocal(data, this.eePos(data));
    this.rawTargetLocal = [...this.targetLocal];
  }

  // Advance the servo targets by one physics step of size h [s].
  step(data, h) {
    if (!this.active) return;
    // low-pass the drag point so pointer jumps arrive as a smooth approach
    const a = 1 - Math.exp(-h / TARGET_TAU);
    for (let i = 0; i < 3; i++) {
      this.targetLocal[i] += a * (this.rawTargetLocal[i] - this.targetLocal[i]);
    }
    const { model, nv } = this;
    this.mujoco.mj_jacBody(model, data, this.jacp, this.jacr, this.handId);
    const J = this.jacp.GetView();
    const J7 = [0, 1, 2].map((r) => this.dofs.map((c) => J[r * nv + c]));

    const p = this.eePos(data);
    const t = this.worldTarget(data);
    let e = [t[0] - p[0], t[1] - p[1], t[2] - p[2]];
    // Subtract the effect of commands the servo hasn't executed yet
    // (J * (qTarget - q)). Without this the integrator chases the servo's
    // ~100 ms lag and limit-cycles around the target.
    for (let r = 0; r < 3; r++) {
      for (let k = 0; k < 7; k++) {
        e[r] -= J7[r][k] * (this.qTarget[k] - data.qpos[this.qadrs[k]]);
      }
    }
    const en = Math.hypot(e[0], e[1], e[2]);
    if (en < 1e-4) return;
    if (en > LEASH) e = e.map((v) => (v * LEASH) / en);

    // A = J J^T + lambda^2 I, solve A y = e (Cramer, 3x3)
    const A = [[LAMBDA2, 0, 0], [0, LAMBDA2, 0], [0, 0, LAMBDA2]];
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        for (let k = 0; k < 7; k++) A[r][c] += J7[r][k] * J7[c][k];
      }
    }
    const det = (M) =>
      M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
      M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
      M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    const D = det(A);
    const col = (i, v) => A.map((row, r) => row.map((x, c) => (c === i ? v[r] : x)));
    const y = [det(col(0, e)) / D, det(col(1, e)) / D, det(col(2, e)) / D];

    const dq = new Array(7);
    for (let k = 0; k < 7; k++) {
      const q = data.qpos[this.qadrs[k]];
      dq[k] = STEP * (J7[0][k] * y[0] + J7[1][k] * y[1] + J7[2][k] * y[2])
        + NS_GAIN * (this.qHome[k] - q) * h;
    }
    // Rate limit as a UNIFORM scale so the step's direction is preserved --
    // clamping joints independently bends the motion off the commanded
    // Cartesian direction whenever the limit engages.
    const dqMax = QDOT_MAX * h;
    let scale = 1;
    for (let k = 0; k < 7; k++) {
      const m = Math.abs(dq[k]);
      if (m > dqMax) scale = Math.min(scale, dqMax / m);
    }
    for (let k = 0; k < 7; k++) {
      const q = data.qpos[this.qadrs[k]];
      let next = this.qTarget[k] + dq[k] * scale;
      // anti-windup: don't let the servo target run away from the actual
      // joint (e.g. when the arm is blocked by the torso or a joint limit)
      next = Math.min(q + WINDUP, Math.max(q - WINDUP, next));
      this.qTarget[k] = Math.min(this.ranges[k][1], Math.max(this.ranges[k][0], next));
      data.ctrl[this.actIds[k]] = this.qTarget[k];
    }
  }
}
