// Pointer-driven joystick pad (the web sibling of the Tk TwistJoystick).
// Values are normalized to [-1, 1] and spring back to zero on release.
export class StickPad {
  constructor(el, { horizontalOnly = false } = {}) {
    this.el = el;
    this.knob = el.querySelector('.knob');
    this.horizontalOnly = horizontalOnly;
    this.x = 0; // +right
    this.y = 0; // +up
    this.active = false;

    el.addEventListener('pointerdown', (e) => {
      this.active = true;
      el.setPointerCapture(e.pointerId);
      this.#track(e);
    });
    el.addEventListener('pointermove', (e) => { if (this.active) this.#track(e); });
    const release = () => {
      this.active = false;
      this.x = 0;
      this.y = 0;
      this.#render();
    };
    el.addEventListener('pointerup', release);
    el.addEventListener('pointercancel', release);
  }

  #track(e) {
    const r = this.el.getBoundingClientRect();
    const range = r.width / 2;
    let x = (e.clientX - (r.left + r.width / 2)) / range;
    let y = -(e.clientY - (r.top + r.height / 2)) / range;
    if (this.horizontalOnly) y = 0;
    const mag = Math.hypot(x, y);
    if (mag > 1) { x /= mag; y /= mag; }
    this.x = x;
    this.y = y;
    this.#render();
  }

  #render() {
    const range = this.el.clientWidth / 2 - this.knob.clientWidth / 2;
    this.knob.style.transform =
      `translate(${this.x * range}px, ${-this.y * range}px)`;
  }
}
