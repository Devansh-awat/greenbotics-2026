"""
Low-latency phone teleop for the car.

Serves a touch control page and carries the control stream over a WebSocket so
there is no per-command HTTP overhead. Only the DC motor (2 pins + PWM) and the
steering servo (1 PWM pin) are driven -- no sensors or encoder are required.

Run from the repo root:

    python3 -m src.teleop.server

Then open  http://<pi-ip>:8000  on your phone (same Wi-Fi). The Pi prints its
URL on startup.

Safety: there are no sensors, so a watchdog brakes the motor and recenters the
steering whenever the phone stops sending updates (lag, lock screen, or a closed
tab). The on-screen MAX SPEED slider caps the throttle.
"""

import json
import threading
import time

from flask import Flask
from flask_sock import Sock

from src.motors import motor, servo


# --- Tuning -----------------------------------------------------------------
STEER_MAX_DEG = 45.0          # maps |steer|=1 -> this servo angle
STEER_DEADZONE = 0.04         # ignore tiny jitter around centre
THROTTLE_DEADZONE = 0.06      # below this magnitude the motor brakes
WATCHDOG_TIMEOUT_S = 0.35     # no message for this long -> stop everything
HTTP_PORT = 8000


app = Flask(__name__)
sock = Sock(app)

# Serialises GPIO access across the WebSocket thread and the watchdog thread.
_hw_lock = threading.Lock()
# Wall-clock time of the last command actually applied to the hardware.
_last_command_ts = 0.0
_stopped = True


def _apply(throttle, steer):
    """Apply a normalised throttle (-1..1) and steer (-1..1) to the hardware."""
    global _last_command_ts, _stopped
    with _hw_lock:
        # Steering
        if abs(steer) < STEER_DEADZONE:
            steer = 0.0
        servo.set_angle(steer * STEER_MAX_DEG)

        # Throttle
        if abs(throttle) < THROTTLE_DEADZONE:
            motor.brake()
        elif throttle > 0:
            motor.forward(throttle * 100.0)
        else:
            motor.reverse(-throttle * 100.0)

        _last_command_ts = time.monotonic()
        _stopped = False


def _stop():
    """Brake the motor and centre the steering. Safe to call repeatedly."""
    global _stopped
    with _hw_lock:
        motor.brake()
        servo.set_angle(0.0)
        _stopped = True


def _watchdog():
    """Brake if the phone goes quiet, so the car never runs away unattended."""
    while True:
        time.sleep(WATCHDOG_TIMEOUT_S / 2.0)
        if _stopped:
            continue
        if time.monotonic() - _last_command_ts > WATCHDOG_TIMEOUT_S:
            print("WATCHDOG: no commands -> stopping")
            _stop()


@sock.route("/ws")
def ws(sock_conn):
    """Receives {"t": throttle, "s": steer} messages and drives the hardware."""
    print("CLIENT: connected")
    try:
        while True:
            raw = sock_conn.receive()  # blocks; returns None when closed
            if raw is None:
                break
            try:
                msg = json.loads(raw)
            except (ValueError, TypeError):
                continue
            if msg.get("type") == "stop":
                _stop()
                continue
            throttle = float(msg.get("t", 0.0))
            steer = float(msg.get("s", 0.0))
            # Clamp to the valid range in case the client misbehaves.
            throttle = max(-1.0, min(1.0, throttle))
            steer = max(-1.0, min(1.0, steer))
            _apply(throttle, steer)
    except Exception as err:  # noqa: BLE001 - log and fall through to stop
        print(f"CLIENT: error {err}")
    finally:
        print("CLIENT: disconnected")
        _stop()


@app.route("/")
def index():
    return PAGE


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>Car Teleop</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; -webkit-touch-callout: none; }
  html, body { margin: 0; height: 100%; overflow: hidden; background: #0c0f14; color: #e6edf3;
               font-family: system-ui, -apple-system, sans-serif; touch-action: none; }
  #top { position: fixed; top: 0; left: 0; right: 0; padding: 10px 14px;
         display: flex; align-items: center; gap: 14px; font-size: 14px;
         background: linear-gradient(#0c0f14, rgba(12,15,20,0)); z-index: 5; }
  #dot { width: 12px; height: 12px; border-radius: 50%; background: #e5534b; flex: none; }
  #dot.on { background: #3fb950; }
  #cap { flex: 1; display: flex; align-items: center; gap: 8px; }
  #cap input { flex: 1; }
  #readout { font-variant-numeric: tabular-nums; opacity: .8; }
  #pad { position: fixed; inset: 0; touch-action: none; }
  #ring { position: absolute; border: 2px solid #2b3340; }
  #knob { position: absolute; width: 88px; height: 88px; margin: -44px 0 0 -44px;
          border-radius: 50%; background: radial-gradient(circle at 35% 30%, #3b82f6, #1d4ed8);
          box-shadow: 0 6px 24px rgba(0,0,0,.5); }
  #hint { position: fixed; bottom: 14px; left: 0; right: 0; text-align: center;
          font-size: 13px; opacity: .45; }
</style>
</head>
<body>
  <div id="top">
    <div id="dot"></div>
    <div id="cap">MAX&nbsp;<input id="cap_slider" type="range" min="20" max="100" value="55"><span id="cap_val">55%</span></div>
    <div id="readout">T&nbsp;0&nbsp;&nbsp;S&nbsp;0</div>
  </div>
  <div id="pad">
    <div id="ring"></div>
    <div id="knob"></div>
  </div>
  <div id="hint">drag anywhere &middot; up/down = throttle &middot; left/right = steer &middot; release = stop</div>

<script>
const dot = document.getElementById('dot');
const readout = document.getElementById('readout');
const ring = document.getElementById('ring');
const knob = document.getElementById('knob');
const pad = document.getElementById('pad');
const capSlider = document.getElementById('cap_slider');
const capVal = document.getElementById('cap_val');

let cap = 0.55;
capSlider.oninput = () => { cap = capSlider.value / 100; capVal.textContent = capSlider.value + '%'; };

// --- joystick geometry ------------------------------------------------------
let RADIUS = 130;            // px travel for full deflection
let cx = 0, cy = 0;          // current ring centre
let active = false, pid = null;
let steer = 0, throttle = 0; // -1..1 (throttle already capped)

function layoutRing() {
  const d = RADIUS * 2;
  ring.style.width = d + 'px';
  ring.style.height = d + 'px';
}
function placeRing(x, y) {
  cx = x; cy = y;
  ring.style.left = (x - RADIUS) + 'px';
  ring.style.top = (y - RADIUS) + 'px';
  placeKnob(x, y);
}
function placeKnob(x, y) {
  knob.style.left = x + 'px';
  knob.style.top = y + 'px';
}
function sizeRadius() {
  RADIUS = Math.min(window.innerWidth, window.innerHeight) * 0.32;
  layoutRing();
}
sizeRadius();
window.addEventListener('resize', sizeRadius);
// rest position: centre of screen
placeRing(window.innerWidth / 2, window.innerHeight / 2);

function update(x, y) {
  let dx = x - cx, dy = y - cy;
  dx = Math.max(-RADIUS, Math.min(RADIUS, dx));
  dy = Math.max(-RADIUS, Math.min(RADIUS, dy));
  placeKnob(cx + dx, cy + dy);
  steer = dx / RADIUS;            // right = +
  throttle = (-dy / RADIUS) * cap; // up = +, capped
  readout.textContent = 'T ' + throttle.toFixed(2) + '  S ' + steer.toFixed(2);
}

pad.addEventListener('pointerdown', e => {
  if (active) return;
  active = true; pid = e.pointerId;
  pad.setPointerCapture(pid);
  placeRing(e.clientX, e.clientY);   // ring springs to where you grab
  update(e.clientX, e.clientY);
});
pad.addEventListener('pointermove', e => {
  if (!active || e.pointerId !== pid) return;
  update(e.clientX, e.clientY);
});
function release(e) {
  if (!active || (e && e.pointerId !== pid)) return;
  active = false; pid = null;
  steer = 0; throttle = 0;
  placeRing(window.innerWidth / 2, window.innerHeight / 2);
  readout.textContent = 'T 0  S 0';
  sendStop();
}
pad.addEventListener('pointerup', release);
pad.addEventListener('pointercancel', release);

// --- websocket --------------------------------------------------------------
let ws = null;
function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => dot.classList.add('on');
  ws.onclose = () => { dot.classList.remove('on'); setTimeout(connect, 600); };
  ws.onerror = () => ws.close();
}
connect();

function send() {
  if (ws && ws.readyState === 1) {
    ws.send(JSON.stringify({ t: throttle, s: steer }));
  }
}
function sendStop() {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: 'stop' }));
}
// Steady 20 Hz stream keeps latency low and feeds the server watchdog.
setInterval(send, 50);
// Stop the car if the page is hidden (lock screen / app switch).
document.addEventListener('visibilitychange', () => { if (document.hidden) { release(); } });
</script>
</body>
</html>
"""


def main():
    print("--- Car Teleop Server ---")
    servo_ok = servo.initialize()
    motor_ok = motor.initialize()
    if not (servo_ok and motor_ok):
        print("Initialization failed. Aborting.")
        if servo_ok:
            servo.cleanup()
        if motor_ok:
            motor.cleanup()
        return

    _stop()
    threading.Thread(target=_watchdog, daemon=True).start()

    print(f"\n  Open  http://<pi-ip>:{HTTP_PORT}  on your phone (same Wi-Fi).\n")
    try:
        # threaded=True so the WebSocket and HTTP requests run concurrently.
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True,
                debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        _stop()
        motor.cleanup()
        servo.cleanup()


if __name__ == "__main__":
    main()
