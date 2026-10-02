/**
 * The demo, stripped to its 3D stage, for embedding in other sites as an iframe.
 *
 * The host page draws its own chrome (labels, readouts, buttons) and talks to this
 * page over postMessage, so the embed carries no UI beyond a loading line.
 *
 *   embed → host   { type: 'rip:ready' }                         3D model is shown
 *                  { type: 'rip:status', text }                  loading progress ('' when live)
 *                  { type: 'rip:grab' }                          visitor grabbed the pendulum
 *                  { type: 'rip:state', t, theta, phi, action,   ~8 times a second
 *                    balanced, controlOn }
 *   host → embed   { type: 'rip:push' }                          flick the pendulum
 *                  { type: 'rip:control', on }                   switch the network on or off
 *
 * Angles are in radians: `theta` from upright, `phi` the motor arm.
 */

import constants from '../generated/constants.json';
import { PendulumRenderer } from './renderer.ts';
import { Policy, type PolicyWeights } from './policy.ts';
import { PendulumController, type Constants } from './control.ts';

const C = constants as unknown as Constants;
const STATE_HZ = 8;

export async function mountEmbed(canvas: HTMLCanvasElement, statusEl: HTMLElement, baseUrl: string) {
  const host = window.parent !== window ? window.parent : null;
  const post = (msg: Record<string, unknown>) => host?.postMessage(msg, '*');
  const setStatus = (text: string) => {
    statusEl.textContent = text;
    post({ type: 'rip:status', text });
  };

  const renderer = new PendulumRenderer(canvas, baseUrl);
  renderer.showAxesGizmo({ corner: 'bottom-left', sizePx: 92 });
  new ResizeObserver(() => renderer.resize()).observe(canvas);
  setStatus('Loading the rig…');
  await renderer.load();
  renderer.render();
  post({ type: 'rip:ready' });

  const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
  let controlOn = !reduced; // With reduced motion, wait for the visitor to start it.
  let controller: PendulumController | null = null;

  window.addEventListener('message', (e) => {
    if (!host || e.source !== host || typeof e.data !== 'object' || !e.data) return;
    if (e.data.type === 'rip:push') controller?.nudge((Math.random() < 0.5 ? -1 : 1) * (6 + Math.random() * 3));
    if (e.data.type === 'rip:control') controlOn = Boolean(e.data.on);
  });

  let running = false;
  let acc = 0;
  let last = performance.now();
  let lastPost = 0;

  function frame(now: number) {
    if (!running) return;
    const dt = Math.min(0.25, (now - last) / 1000);
    last = now;
    if (controller) {
      acc += dt;
      const period = controller.controlPeriodS;
      let state = null;
      let ticks = 0;
      while (acc >= period && ticks < 8) {
        state = controlOn ? controller.step() : controller.coast();
        acc -= period;
        ticks++;
      }
      if (ticks === 8) acc = 0;
      if (state) {
        renderer.setJointAngles(state.motorPosRad, state.pendulumPosRad);
        renderer.setDragArrow(controller.grabArrow());
        if (now - lastPost > 1000 / STATE_HZ) {
          lastPost = now;
          post({
            type: 'rip:state',
            t: state.elapsedS,
            theta: state.thetaRad,
            phi: state.motorPosRad,
            action: state.action,
            balanced: state.balanced,
            controlOn,
          });
        }
      }
    }
    renderer.render();
    requestAnimationFrame(frame);
  }

  // Pause while hidden; the host's own scrolling hides the iframe too.
  const start = () => {
    if (running || document.hidden) return;
    running = true;
    last = performance.now();
    requestAnimationFrame(frame);
  };
  new IntersectionObserver(([e]) => (e.isIntersecting ? start() : (running = false))).observe(canvas);
  document.addEventListener('visibilitychange', () => (document.hidden ? (running = false) : start()));
  start();

  setStatus('Loading the physics engine…');
  const [{ default: loadMujoco }, xml, weights] = await Promise.all([
    import('@mujoco/mujoco'),
    fetch(`${baseUrl}sim/model.xml`).then((r) => r.text()),
    fetch(`${baseUrl}sim/policy.json`).then((r) => r.json() as Promise<PolicyWeights>),
  ]);
  const mujoco = await loadMujoco();
  const model = mujoco.MjModel.from_xml_string(xml);
  const data = new mujoco.MjData(model);
  const c = new PendulumController({
    mujoco: mujoco as never,
    model,
    data: data as never,
    policy: new Policy(weights),
    constants: C,
  });
  c.reset();
  const name2id = (mujoco as never as { mj_name2id(m: unknown, t: number, n: string): number }).mj_name2id;
  const body = name2id(model, 1, 'pendulum'); // 1 = mjOBJ_BODY
  if (body > 0) {
    renderer.setGrabDelegate({
      tryGrab: (p) => {
        c.grab(body, p);
        post({ type: 'rip:grab' });
        return true;
      },
      drag: (p) => c.dragTo(p),
      release: () => {
        c.release();
        renderer.setDragArrow(null);
      },
    });
  }
  controller = c;
  setStatus('');
}
