// Twist teleoperation for the GARMI mecanum base.
//
// Port of garmi_description/mujoco/teleop.py @ 0.1.1 (Apache-2.0, TUM):
// mecanum inverse kinematics plus a feedforward + proportional controller on
// the measured base twist ("perfect odometry" from the free joint velocity),
// with a tightly-clamped yaw integral that cancels the residual rotation the
// discrete-roller model induces while strafing.
//
// Web deviations from upstream: apply() runs once per physics substep with
// h = timestep (an integral needs sim-time accounting -- per-frame wall-clock
// integration over-integrates whenever the sim runs below realtime), and a
// zero-command deadband brakes the wheels instead of closing the loop on
// contact noise (at the relaxed 4 ms mobile timestep the P/I terms would
// chase roller-contact jitter and creep the base).

// Base geometry (matches garmi.xml / the URDF).
export const WHEEL_R = 0.0759;          // wheel radius [m]
export const LXY = 0.319 + 0.2755;      // half wheelbase + half track [m]
export const WHEEL_CLAMP = 10.0;        // rad/s, matches the actuator ctrlrange

// Teleop maxima (a single axis stays within the wheel speed limit).
export const VMAX = 0.7;                // m/s (forward/back)
export const WMAX = 1.2;                // rad/s
// Like the real robot, strafing is noticeably slower than driving forward.
export const LATERAL_SCALE = 0.6;

// Feedforward compensates the open-loop DC gain (rotation is ~4x because the
// free rollers let the base spin faster than no-slip kinematics predicts).
// The small yaw integral cancels strafe-induced rotation; its tight clamp
// bounds any windup.
const FF = [1.0, 1.0, 0.25];
const KP = [1.6, 1.6, 1.6];
const KI = [0.0, 0.0, 2.5];
const I_CLAMP = [0.5, 0.5, 0.1];

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
    this.integ = [0, 0, 0];
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

  // Write wheel controls for the desired twist; call once per physics step
  // of size h [s]. Open loop ignores odometry.
  apply(data, desired, h) {
    const ctrl = data.ctrl;
    const meas = this.measureTwist(data);
    // One control law, driving and idle alike (upstream behaviour): brake the
    // measured twist toward the command with a tightly-clamped yaw integral.
    // Velocity feedback commanding velocity is naturally damped, so the base
    // settles without ringing when the command returns to zero (a position
    // hold, by contrast, commands wheel velocity from a pose error and rings
    // on the wheel servos' lag).
    let cmd = desired;
    if (this.closedLoop) {
      cmd = desired.map((d, k) => {
        const e = d - meas[k];
        this.integ[k] = Math.max(-I_CLAMP[k],
          Math.min(I_CLAMP[k], this.integ[k] + KI[k] * e * h));
        return FF[k] * d + KP[k] * e + this.integ[k];
      });
    }
    const wheels = twistToWheels(cmd[0], cmd[1], cmd[2]);
    this.wheelIds.forEach((id, i) => { ctrl[id] = wheels[i]; });
  }

  basePosition(data) {
    const q = data.qpos, qa = this.qposAdr;
    return [q[qa], q[qa + 1], q[qa + 2]];
  }
}
