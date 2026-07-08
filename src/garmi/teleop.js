// Twist teleoperation for the GARMI mecanum base.
//
// Direct port of garmi_description/mujoco/teleop.py (Apache-2.0, TUM):
// mecanum inverse kinematics plus a feedforward + proportional controller on
// the measured base twist ("perfect odometry" from the free joint velocity),
// which cancels the open-loop drift of the free-roller mecanum model.

// Base geometry (matches garmi.xml / the URDF).
export const WHEEL_R = 0.0759;          // wheel radius [m]
export const LXY = 0.319 + 0.2755;      // half wheelbase + half track [m]
export const WHEEL_CLAMP = 10.0;        // rad/s, matches the actuator ctrlrange

// Teleop maxima (a single axis stays within the wheel speed limit).
export const VMAX = 0.7;                // m/s
export const WMAX = 1.2;                // rad/s

// Feedforward compensates the open-loop DC gain (rotation is ~4x because the
// free rollers let the base spin faster than no-slip kinematics predicts).
const FF = [1.0, 1.0, 0.25];
const KP = [1.6, 1.6, 1.6];

const clamp = (v, lim) => Math.max(-lim, Math.min(lim, v));

// Mecanum inverse kinematics: body twist -> (fl, fr, rl, rr) wheel speeds.
export function twistToWheels(vx, vy, wz) {
  return [
    (vx + vy + LXY * wz) / WHEEL_R,
    (vx - vy - LXY * wz) / WHEEL_R,
    (vx - vy + LXY * wz) / WHEEL_R,
    (vx + vy - LXY * wz) / WHEEL_R,
  ].map((v) => clamp(v, WHEEL_CLAMP));
}

export class BaseController {
  constructor(mujoco, model) {
    this.model = model;
    this.wheelIds = ['fl', 'fr', 'rl', 'rr'].map(
      (w) => model.actuator(`wheel_${w}`).id,
    );
    // Accessor address fields are scalars in the wasm bindings (arrays in
    // the Python ones) -- accept either.
    const scalar = (v) => (typeof v === 'number' ? v : Number(v[0]));
    const free = model.jnt('base_free');
    this.qposAdr = scalar(free.qposadr);
    this.dofAdr = scalar(free.dofadr);
    this.closedLoop = true;
  }

  // Base twist (vx, vy, wz) in the base frame. For a free joint MuJoCo stores
  // qvel as linear velocity in the global frame followed by angular velocity
  // in the local frame: rotate the linear part into the base frame and take
  // the local yaw rate directly.
  measureTwist(data) {
    const q = data.qpos, v = data.qvel, qa = this.qposAdr, da = this.dofAdr;
    // Free joint quaternion is (w, x, y, z); rotate world velocity by q^-1.
    const w = q[qa + 3], x = q[qa + 4], y = q[qa + 5], z = q[qa + 6];
    const vx = v[da], vy = v[da + 1], vz = v[da + 2];
    // q^-1 * (vx,vy,vz) * q
    const tx = 2 * (y * vz - z * vy);
    const ty = 2 * (z * vx - x * vz);
    const tz = 2 * (x * vy - y * vx);
    return [
      vx - w * tx + (y * tz - z * ty),
      vy - w * ty + (z * tx - x * tz),
      v[da + 5],
    ];
  }

  // Write wheel controls for the desired twist. Open loop ignores odometry.
  apply(data, desired, dt) {
    let cmd = desired;
    if (this.closedLoop) {
      const meas = this.measureTwist(data);
      cmd = desired.map((d, k) => FF[k] * d + KP[k] * (d - meas[k]));
    }
    const wheels = twistToWheels(cmd[0], cmd[1], cmd[2]);
    const ctrl = data.ctrl;
    this.wheelIds.forEach((id, i) => { ctrl[id] = wheels[i]; });
  }

  basePosition(data) {
    const q = data.qpos, qa = this.qposAdr;
    return [q[qa], q[qa + 1], q[qa + 2]];
  }
}
