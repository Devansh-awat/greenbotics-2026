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
from src.obstacle_challenge import config


# --- Tuning -----------------------------------------------------------------
STEER_MAX_DEG = 60.0          # maps |steer|=1 -> this servo angle
STEER_DEADZONE = 0.04         # ignore tiny jitter around centre
THROTTLE_DEADZONE = 0.06      # below this magnitude the motor brakes
WATCHDOG_TIMEOUT_S = 0.35     # no message for this long -> stop everything
STEER_TARGET_EPS_DEG = 0.5    # ignore steer changes smaller than this (finger tremor)
CLIENT_SEND_HZ = 50           # phone update rate; must stay below the servo PWM rate
RPM_SAMPLE_PERIOD_S = 0.04    # 25 Hz steady sampling on Pi local clock (immune to network)
MAX_CHART_MOTOR_RPM = 2100.0  # scale chart up to 2100 RPM to capture no-load overspeed
MAX_CHART_WHEEL_RPM = MAX_CHART_MOTOR_RPM * motor.EXTERNAL_GEAR_RATIO  # ~718.4 wheel RPM
HTTP_PORT = 8000


app = Flask(__name__)
sock = Sock(app)

# Serialises GPIO access across the WebSocket thread and the watchdog thread.
_hw_lock = threading.Lock()
# Wall-clock time of the last command actually applied to the hardware.
_last_command_ts = 0.0
_stopped = True
# Angle last written to the servo.
_steer_deg = 0.0

# Pi-local encoder sampling state (independent of network/WebSocket timing)
_rpm_cond = threading.Condition()
_current_wheel_rpm = 0.0
_current_motor_rpm = 0.0


def _apply(throttle, steer):
    """Apply a normalised throttle (-1..1) and steer (-1..1) to the hardware."""
    global _last_command_ts, _stopped
    with _hw_lock:
        # Steering
        if abs(steer) < STEER_DEADZONE:
            steer = 0.0
        _set_steer(steer * STEER_MAX_DEG)

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
        _set_steer(0.0)
        _stopped = True


def _set_steer(angle):
    """Write a steering angle, skipping sub-threshold changes (caller holds lock)."""
    global _steer_deg
    if angle != 0.0 and abs(angle - _steer_deg) < STEER_TARGET_EPS_DEG:
        return
    _steer_deg = angle
    servo.set_angle_unlimited(angle)


def _rpm_monitor():
    """Sample the encoder at a fixed Pi-local cadence without smoothing.

    Runs on the Pi's monotonic clock so network arrival jitter has zero impact
    on dt or RPM calculations. No EMA filter is applied so true motor jerkiness,
    cogging, or friction hesitations are clearly visible.
    """
    global _current_wheel_rpm, _current_motor_rpm
    last_pos = motor.encoder_position() if motor.encoder else None
    last_t = time.monotonic()

    while True:
        time.sleep(RPM_SAMPLE_PERIOD_S)
        now = time.monotonic()
        dt = now - last_t
        if dt <= 0:
            continue

        pos = motor.encoder_position() if motor.encoder else None
        if pos is not None and last_pos is not None and motor.encoder is not None:
            cpr = motor.encoder.counts_per_rev
            delta_pos = pos - last_pos
            # Raw instantaneous wheel RPM over the steady local interval (no smoothing)
            wheel_rpm = (abs(delta_pos) / cpr) * (60.0 / dt)
            motor_rpm = wheel_rpm * motor.MAX_MOTOR_RPM / motor.MAX_WHEEL_RPM
        else:
            wheel_rpm = 0.0
            motor_rpm = 0.0

        with _rpm_cond:
            _current_wheel_rpm = wheel_rpm
            _current_motor_rpm = motor_rpm
            _rpm_cond.notify_all()

        last_pos = pos
        last_t = now


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
    """Receives control messages from the client and streams back live raw RPM data."""
    print("CLIENT: connected")
    stop_event = threading.Event()

    def send_rpm_loop():
        while not stop_event.is_set():
            with _rpm_cond:
                _rpm_cond.wait(timeout=0.1)
                w_rpm = _current_wheel_rpm
                m_rpm = _current_motor_rpm
            if stop_event.is_set():
                break
            try:
                sock_conn.send(json.dumps({
                    "wheel_rpm": round(w_rpm, 1),
                    "wheel_max": round(MAX_CHART_WHEEL_RPM, 1),
                    "motor_rpm": round(m_rpm, 1),
                    "motor_max": round(MAX_CHART_MOTOR_RPM, 1),
                }))
            except Exception:
                break

    threading.Thread(target=send_rpm_loop, daemon=True).start()

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
            else:
                throttle = float(msg.get("t", 0.0))
                steer = float(msg.get("s", 0.0))
                # Clamp to the valid range in case the client misbehaves.
                throttle = max(-1.0, min(1.0, throttle))
                steer = max(-1.0, min(1.0, steer))
                _apply(throttle, steer)
    except Exception as err:  # noqa: BLE001 - log and fall through to stop
        print(f"CLIENT: error {err}")
    finally:
        stop_event.set()
        with _rpm_cond:
            _rpm_cond.notify_all()
        print("CLIENT: disconnected")
        _stop()


@app.route("/")
def index():
    return PAGE.replace("CLIENT_SEND_HZ", str(CLIENT_SEND_HZ))


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
  #rpm { position: fixed; top: 38px; left: 0; right: 0; padding: 0 14px; font-size: 12px;
         font-variant-numeric: tabular-nums; opacity: .8; z-index: 5; }
  #rpm .wheel { color: #3b82f6; font-weight: 500; }
  #rpm .motor { color: #f59e0b; font-weight: 500; }
  #chart { position: fixed; top: 58px; left: 14px; right: 14px; height: 86px;
           z-index: 5; pointer-events: none; opacity: .95; border-radius: 6px;
           background: rgba(14, 18, 26, 0.75); border: 1px solid rgba(255, 255, 255, 0.1); }
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
  <div id="rpm"><span class="wheel">wheel 0 / 0 rpm</span>&nbsp;&nbsp;<span class="motor">motor 0 / 0 rpm</span></div>
  <canvas id="chart"></canvas>
  <div id="pad">
    <div id="ring"></div>
    <div id="knob"></div>
  </div>
  <div id="hint">drag anywhere &middot; up/down = throttle &middot; left/right = steer &middot; release = stop</div>

<script>
const dot = document.getElementById('dot');
const readout = document.getElementById('readout');
const rpmReadout = document.getElementById('rpm');
const chartCanvas = document.getElementById('chart');
const chartCtx = chartCanvas.getContext('2d');
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

// --- rpm history chart -------------------------------------------------------
const RPM_HISTORY_S = 12;        // rolling window shown on the chart (12s)
let rpmHistory = [];            // {t (s), wheel (0..1), motor (0..1)}

function sizeChart() {
  const dpr = window.devicePixelRatio || 1;
  const rect = chartCanvas.getBoundingClientRect();
  chartCanvas.width = rect.width * dpr;
  chartCanvas.height = rect.height * dpr;
  chartCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
sizeChart();
window.addEventListener('resize', sizeChart);

function drawChart() {
  const w = chartCanvas.clientWidth, h = chartCanvas.clientHeight;
  const now = performance.now() / 1000;
  rpmHistory = rpmHistory.filter(p => now - p.t <= RPM_HISTORY_S);
  chartCtx.clearRect(0, 0, w, h);

  const padTop = 10, padBot = 12, padRight = 38;
  const plotW = w - padRight;
  const plotH = h - padTop - padBot;

  // --- Grid lines & RPM level ticks (up to 2100 RPM) ---
  const levels = [500, 1000, 1500, 2000, 2100];
  chartCtx.save();
  chartCtx.font = '9px system-ui, -apple-system, sans-serif';
  chartCtx.textAlign = 'right';
  chartCtx.textBaseline = 'middle';

  levels.forEach(lvl => {
    const frac = lvl / 2100;
    const y = padTop + plotH * (1 - frac);

    chartCtx.beginPath();
    chartCtx.strokeStyle = (lvl === 2100) ? 'rgba(245, 158, 11, 0.4)' : 'rgba(255, 255, 255, 0.1)';
    chartCtx.lineWidth = 1;
    chartCtx.setLineDash(lvl === 2100 ? [] : [3, 4]);
    chartCtx.moveTo(0, y);
    chartCtx.lineTo(plotW, y);
    chartCtx.stroke();

    chartCtx.fillStyle = (lvl === 2100) ? '#f59e0b' : 'rgba(255, 255, 255, 0.45)';
    chartCtx.fillText(lvl, w - 4, y);
  });

  // Time markers along bottom
  chartCtx.fillStyle = 'rgba(255, 255, 255, 0.3)';
  chartCtx.textAlign = 'center';
  chartCtx.fillText('-10s', plotW * (1 - 10 / RPM_HISTORY_S), h - 4);
  chartCtx.fillText('-5s', plotW * (1 - 5 / RPM_HISTORY_S), h - 4);
  chartCtx.fillText('now', plotW - 6, h - 4);
  chartCtx.restore();

  if (rpmHistory.length < 2) return;

  const plot = (key, color) => {
    chartCtx.beginPath();
    rpmHistory.forEach((p, i) => {
      const x = plotW * (1 - (now - p.t) / RPM_HISTORY_S);
      const y = padTop + plotH * (1 - Math.max(0, Math.min(1, p[key])));
      if (i === 0) chartCtx.moveTo(x, y); else chartCtx.lineTo(x, y);
    });
    chartCtx.strokeStyle = color;
    chartCtx.lineWidth = 2;
    chartCtx.stroke();
  };
  plot('wheel', '#3b82f6');
  plot('motor', '#f59e0b');
}

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
  ws.onmessage = (ev) => {
    const d = JSON.parse(ev.data);
    rpmReadout.innerHTML =
      '<span class="wheel">wheel ' + d.wheel_rpm.toFixed(0) + ' / ' + d.wheel_max.toFixed(0) + ' rpm</span>' +
      '&nbsp;&nbsp;<span class="motor">motor ' + d.motor_rpm.toFixed(0) + ' / ' + d.motor_max.toFixed(0) + ' rpm</span>';
    rpmHistory.push({
      t: performance.now() / 1000,
      wheel: d.wheel_rpm / d.wheel_max,
      motor: d.motor_rpm / d.motor_max,
    });
    drawChart();
  };
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
// Steady stream keeps latency low and feeds the server watchdog.
setInterval(send, 1000 / CLIENT_SEND_HZ);
// Stop the car if the page is hidden (lock screen / app switch).
document.addEventListener('visibilitychange', () => { if (document.hidden) { release(); } });
</script>
</body>
</html>
"""


def main():
    print("--- Car Teleop Server ---")
    assert CLIENT_SEND_HZ < config.SERVO_PWM_FREQ, (
        "CLIENT_SEND_HZ must be below SERVO_PWM_FREQ or updates are dropped")
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
    threading.Thread(target=_rpm_monitor, daemon=True).start()

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
