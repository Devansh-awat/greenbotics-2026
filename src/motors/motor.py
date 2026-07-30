import lgpio
from rpi_hardware_pwm import HardwarePWM
import time
import math
from src.obstacle_challenge import config
from src.sensors.encoder import IncrementalEncoder
import board


gpio_handle = None
motor_pwm = None
encoder = None
_MAX_PWM_RETRIES = 5
_PWM_RETRY_DELAY = 0.1

# --- Closed-loop WHEEL-RPM control ---
# Max wheel speed: 1330 rpm output shaft * 13/38 external gear = 455 wheel rpm.
# set_speed_rpm() / start_rpm_control() take a WHEEL rpm in 0..MAX_WHEEL_RPM.
MAX_WHEEL_RPM = 1330.0 * 13.0 / 38.0  # 455.0
# Minimum loop interval (s) before a fresh RPM measurement is trusted. Below this
# the encoder delta is too small/noisy, so we skip the update.
_RPM_MIN_DT = 0.02
# The motor will not turn steadily below ~40 wheel rpm (~33% duty once moving,
# and it needs ~45% just to break free from rest). For any target below that we
# can't hold a steady speed, so the controller pulses: it tracks where the wheel
# *should* be and drives a fixed pulse whenever it falls behind, which holds the
# requested AVERAGE rpm (the motion is stop-start at low targets).
_RPM_PULSE_DUTY = 46.0   # duty used to pulse the wheel forward (above stall)
# Hardware polarity of the encoder: driving FORWARD makes encoder.position
# INCREASE, so forward travel has sign +1 (re-measured 2026-07-04 via
# test_encoder_raw.py after fixing a broken GND wire on the encoder - the
# previous -1.0 dated from before that fix and was inverted, which turned the
# rpm controller's error term into positive feedback: see the runaway during
# parking() where measured_rpm ran away from the +80 target instead of
# holding it). The controller uses this to track SIGNED progress in the
# commanded direction, so a wheel still coasting the wrong way (e.g. residual
# forward momentum when a reverse move begins) reads as negative progress and
# is actively driven the right way instead of being mistaken for progress
# (the old abs() behaviour drove it into a wall).
_ENC_FORWARD_SIGN = 1.0
# Closed-loop duty for targets at/above the stall floor (continuous mode): rather
# than commanding a FIXED duty (which stalls if the load is high, e.g. the wheels
# are cranked hard over), the controller ramps the duty up until the wheel turns
# and eases it back to hold the target rpm. PI gains are duty-% per wheel-rpm.
_RPM_CONTINUOUS_MIN = 40.0   # wheel rpm at/above which we hold rpm with closed-loop duty
_RPM_STALL_RPM = 5.0         # below this MEASURED rpm the wheel counts as "not moving yet"
_RPM_KP_DUTY = 0.20          # proportional duty-% per wheel-rpm of error (bumped from 0.15
                              # 2026-07-26 for faster tracking of a fluctuating target; needs
                              # on-hardware bench validation, see test_rpm_10.py)
_RPM_KI_DUTY = 0.50          # integral duty-% per (wheel-rpm * s) of error (bumped from 0.40,
                              # same caveat - watch for the oscillation failure mode described
                              # above (stall/re-stall chatter) if pushed further)
_RPM_KICK_DUTY = 60.0        # break-free duty the wheel starts at while stalled
_RPM_STALL_RAMP = 115.0      # while stalled, ramp duty up this fast (%/s): kick->cap in ~0.35s
_RPM_CAP = 100.0             # max duty (%) the loop may command
# Feed-forward estimate of the duty (%) needed to just HOLD a moving wheel at a
# given target: duty ~= static + gain * wheel_rpm. Calibrated off the known
# ~40 wheel-rpm @ ~33% duty operating point (30 + 0.12*40 = 34.8). Used to reseed
# the PI integrator the instant the wheel breaks free, so the (much higher)
# break-free duty doesn't get carried into the hold and overshoot the target -
# see the stall->move hand-off in set_speed_rpm().
_RPM_FF_STATIC = 30.0        # duty (%) a moving wheel needs near 0 rpm (static/coulomb friction)
_RPM_FF_GAIN = 0.12          # extra duty (%) per wheel-rpm to hold speed
_RPM_EMA_ALPHA = 0.3          # smoothing for the reported (not control-loop) measured rpm
# Persistent controller state, shared across successive set_speed_rpm() calls.
_rpm_state = {
    "start_pos": None,    # encoder position when control began
    "start_time": None,   # time control began (for the running-average rpm)
    "target_pos": 0.0,    # accumulated counts the wheel *should* have traveled
    "last_time": None,
    "last_pos": None,     # encoder position at the previous control step
    "pulsing": False,
    "duty": 0.0,          # current commanded/integrated duty (%)
    "broke_free": False,  # True once the wheel has broken free in the current continuous move
    "ema_rpm": 0.0,        # smoothed instantaneous rpm, for reporting/logging only
    "applied": None,      # what was LAST ACTUALLY WRITTEN to the driver (see _apply_*)
}

# Background control thread (so the loop can run once and hold a target).
import threading
_rpm_thread = None
_rpm_stop_event = None
_rpm_lock = threading.Lock()
_rpm_target = 0.0
_rpm_dir = "forward"
_rpm_measured = 0.0

def initialize():
    """Initializes the DC motor driver and PWM."""
    global gpio_handle, motor_pwm
    try:
        gpio_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(gpio_handle, config.AIN1_PIN)
        lgpio.gpio_claim_output(gpio_handle, config.AIN2_PIN)
        lgpio.gpio_claim_output(gpio_handle, config.STBY_PIN)

        standby()
    except Exception as err:
        print(f"FATAL: Motor failed to initialize GPIO: {err}")
        return False

    last_error = None
    for attempt in range(1, _MAX_PWM_RETRIES + 1):
        try:
            motor_pwm = HardwarePWM(
                pwm_channel=config.MOTOR_PWM_CHANNEL,
                hz=config.MOTOR_PWM_FREQ,
                chip=config.MOTOR_PWM_CHIP,
            )
            motor_pwm.start(0)
            
            global encoder
            try:
                encoder = IncrementalEncoder(board.D20)
                print("INFO: Encoder Initialized.")
            except Exception as e:
                print(f"WARNING: Encoder failed to initialize: {e}")

            print("INFO: Motor Initialized.")
            return True
        except PermissionError as err:
            last_error = err
            print(
                f"WARNING: Motor PWM permission denied (attempt {attempt}/{_MAX_PWM_RETRIES}): {err}"
            )
            time.sleep(_PWM_RETRY_DELAY * attempt)
        except Exception as err:
            last_error = err
            print(f"FATAL: Motor failed to initialize: {err}")
            time.sleep(_PWM_RETRY_DELAY * attempt)

    print(f"FATAL: Motor failed to initialize after {_MAX_PWM_RETRIES} attempts: {last_error}")
    return False


def _set_speed(speed):
    """Internal function to set motor PWM duty cycle."""
    if motor_pwm:
        motor_pwm.change_duty_cycle(max(0, min(100, speed)))


def _drive(ain1, speed):
    """Engage the driver in the direction given by AIN1 (AIN2 is its complement),
    honouring config.MOTOR_DIR_INVERT for the reversed motor leads."""
    if gpio_handle:
        if getattr(config, "MOTOR_DIR_INVERT", False):
            ain1 = 1 - ain1
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, ain1)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 1 - ain1)
        _set_speed(speed)


def forward(speed):
    """Drives the motor forward at a given speed."""
    _drive(1, speed)


def reverse(speed):
    """Drives the motor in reverse at a given speed."""
    _drive(0, speed)


def standby():
    """Puts the motor driver in standby mode (low power, disengaged)."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 0)


def brake():
    """Brakes the motor by shorting its terminals."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 1)
        _set_speed(0)


def move(distance_cm, max_speed=70, min_speed=40, accel_dist_cm=2, kp=3.0, ki=0.0, kd=0.1):
    """
    Moves the motor a specific distance in cm using the encoder.
    Uses PID control for accurate deceleration, while keeping acceleration ramp.
    """
    if not encoder:
        print("ERROR: Encoder not initialized. Cannot use move().")
        return

    target_mm = abs(distance_cm * 10)
    accel_mm = accel_dist_cm * 10
    
    start_dist = encoder.distance
    direction = 1 if distance_cm > 0 else -1
    motor_func = forward if direction == 1 else reverse
    
    brake()
    
    integral = 0.0
    previous_error = target_mm
    last_time = time.time()
    
    while True:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0:
            dt = 0.01
            
        # Signed distance travelled in the COMMANDED direction (direction is
        # +1 forward / -1 reverse, _ENC_FORWARD_SIGN maps forward onto the
        # encoder). If the wheel is still moving the wrong way (e.g. called
        # mid-motion against momentum) this goes negative, so the error grows and
        # it drives harder the right way instead of counting it as progress.
        current_travel_mm = direction * _ENC_FORWARD_SIGN * (encoder.distance - start_dist)
        error = target_mm - current_travel_mm
        
        # Stop condition: reached target or within 0.5mm tolerance
        if error <= 0.5:
            break
            
        integral += error * dt
        derivative = (error - previous_error) / dt
        
        # PID evaluates the required speed based on distance error
        pid_output = (kp * error) + (ki * integral) + (kd * derivative)
        
        # Cap speed by max_speed
        speed = min(max_speed, abs(pid_output))
        
        # Acceleration phase (overrides PID output to slowly ramp up if needed)
        if current_travel_mm < accel_mm:
            speed_ratio = current_travel_mm / accel_mm if accel_mm > 0 else 1
            accel_speed = min_speed + (max_speed - min_speed) * speed_ratio
            speed = min(speed, accel_speed)
            
        # Ensure minimum speed so motor does not stall before reaching target
        speed = max(min_speed, speed)
        
        motor_func(speed)
        
        previous_error = error
        last_time = current_time
        
        time.sleep(0.01)

    brake()
    print(f"move({distance_cm} cm) complete.")


def reset_rpm_control():
    """Clears the persistent state used by set_speed_rpm().

    Call this whenever you start a new RPM-controlled move, or after the motor
    has been stopped/braked, so stale position/timing don't carry over."""
    _rpm_state["start_pos"] = None
    _rpm_state["start_time"] = None
    _rpm_state["target_pos"] = 0.0
    _rpm_state["last_time"] = None
    _rpm_state["last_pos"] = None
    _rpm_state["pulsing"] = False
    _rpm_state["duty"] = 0.0
    _rpm_state["broke_free"] = False
    _rpm_state["ema_rpm"] = 0.0
    _rpm_state["applied"] = None


def _apply_drive(motor_func, duty, direction):
    """Write a drive command to the driver AND record exactly what was written.

    Every hardware write in the rpm controller goes through here or
    _apply_brake(), so the log can report what the driver actually received
    rather than what the controller intended. Those two silently diverged once
    already: a refactor left the stall branch computing a duty it never wrote,
    while the log's "Duty:" field (which prints the intended _rpm_state["duty"])
    happily showed 100% for a minute at a time with the motor sitting shorted
    and stone cold. Logging intent and reality side by side makes that class of
    bug obvious on the first line instead of invisible for days.

    NOTE: "applied" deliberately persists until the next write. That mirrors the
    hardware, which latches the last command -- if a control path stops writing,
    the driver keeps doing whatever it was last told, and the log should show
    that stale value rather than implying a fresh command.
    """
    motor_func(duty)
    _rpm_state["applied"] = f"{'FWD' if direction == 'forward' else 'REV'} {duty:.1f}%"


def _apply_brake():
    """Brake the motor and record it as the applied hardware state."""
    brake()
    _rpm_state["applied"] = "BRAKE"


def set_speed_rpm(target_rpm, direction="forward", pulse_duty=_RPM_PULSE_DUTY,
                  deadband_rev=0.02, kp_duty=_RPM_KP_DUTY, ki_duty=_RPM_KI_DUTY,
                  kick_duty=_RPM_KICK_DUTY, stall_ramp=_RPM_STALL_RAMP, cap=_RPM_CAP,
                  ff_static=_RPM_FF_STATIC, ff_gain=_RPM_FF_GAIN):
    """Hold a target WHEEL rpm using encoder feedback.

    Performs ONE control update per call. You can either call it yourself in a
    loop, OR use start_rpm_control() which runs this in a background thread so
    you only have to kick it off once.

    Two regimes, split at the ~40 wheel-rpm stall floor:

    * target >= ~40 rpm (continuous): holds rpm with a CLOSED-LOOP duty instead
      of a fixed value. While the wheel is still stalled it starts at `kick_duty`
      and ramps the duty toward `cap` fast (`stall_ramp` %/s) to break free even
      under a heavy steering load; the moment it's moving it switches to a PI
      loop on the rpm error that eases the duty up/down to hold the target.

    * target < ~40 rpm (pulsing): the wheel can't turn steadily that slowly, so
      it integrates where the wheel *should* be and drives a fixed break-free
      `pulse_duty` whenever it falls behind, braking once caught up; the AVERAGE
      equals the requested rpm (motion is stop-start).

        target_rpm:   desired WHEEL speed, clamped to 0..MAX_WHEEL_RPM (455).
        direction:    "forward" or "reverse".
        pulse_duty:   fixed break-free duty used in the sub-stall pulsing regime.
        deadband_rev: pulsing hysteresis as a fraction of a wheel rev.
        kp_duty/ki_duty: continuous-regime PI gains (duty-% per wheel-rpm[*s]).
        kick_duty:    continuous-regime duty the wheel starts at while stalled.
        stall_ramp:   how fast (duty-% per s) the duty climbs while still stalled.
        cap:          max duty (%) the continuous loop may command.

    Returns the running AVERAGE wheel rpm since control began (None if the
    encoder is unavailable or the call came too soon).

    NOTE: commands speed only; it does not stop on its own. Call brake() (and
    reset_rpm_control()) when done.
    """
    if not encoder:
        print("ERROR: Encoder not initialized. Cannot use set_speed_rpm().")
        return None

    target_rpm = max(0.0, min(MAX_WHEEL_RPM, target_rpm))
    dir_cmd = 1.0 if direction == "forward" else -1.0
    motor_func = forward if direction == "forward" else reverse
    cpr = encoder.counts_per_rev

    now = time.time()
    pos = encoder.position

    # First call after a reset: latch the starting position/time.
    if _rpm_state["last_time"] is None:
        _rpm_state["start_pos"] = pos
        _rpm_state["start_time"] = now
        _rpm_state["target_pos"] = 0.0
        _rpm_state["last_time"] = now
        _rpm_state["last_pos"] = pos
        _rpm_state["pulsing"] = False
        _rpm_state["duty"] = 0.0
        _rpm_state["broke_free"] = False
        return 0.0

    dt = now - _rpm_state["last_time"]
    if dt < _RPM_MIN_DT:
        return None  # too soon, hold the current command
    _rpm_state["last_time"] = now

    # Advance the reference position the wheel should have reached by now.
    _rpm_state["target_pos"] += (target_rpm / 60.0) * cpr * dt

    # Progress = SIGNED distance travelled in the COMMANDED direction. dir_cmd is
    # +1 forward / -1 reverse and _ENC_FORWARD_SIGN maps forward travel onto the
    # encoder, so a wheel coasting the wrong way (e.g. residual forward momentum
    # at the start of a reverse move) yields negative progress -> a large deficit
    # -> it gets actively driven the right way, instead of the old abs() reading
    # the wrong-way motion as progress and braking into a wall.
    actual = dir_cmd * _ENC_FORWARD_SIGN * (pos - _rpm_state["start_pos"])
    # Anti-windup: never let the reference lead the wheel by more than one rev,
    # so a brief stall doesn't make it race to catch up afterwards.
    _rpm_state["target_pos"] = min(_rpm_state["target_pos"], actual + cpr)

    # Measured WHEEL rpm over this step, signed toward the commanded direction
    # (negative if the wheel is moving the wrong way).
    step = dir_cmd * _ENC_FORWARD_SIGN * (pos - _rpm_state["last_pos"])
    _rpm_state["last_pos"] = pos
    meas_rpm = (step / cpr) * 60.0 / dt
    # Smoothed instantaneous rpm, for get_measured_rpm()/logging only - the PI
    # loop above uses the raw per-step meas_rpm directly. The lifetime average
    # returned below lags badly behind the true current speed (it's dragged
    # down by the break-free/ramp-up at the start of the session), which made
    # tuning off the log misleading - it looked like targets were never
    # reached even once the wheel had settled near them.
    _rpm_state["ema_rpm"] += _RPM_EMA_ALPHA * (meas_rpm - _rpm_state["ema_rpm"])

    deficit = _rpm_state["target_pos"] - actual
    deadband = deadband_rev * cpr

    if target_rpm <= 0:
        _apply_brake()
        _rpm_state["pulsing"] = False
        _rpm_state["duty"] = 0.0
        _rpm_state["broke_free"] = False
    elif target_rpm >= _RPM_CONTINUOUS_MIN:
        # Continuous regime, two phases:
        err = target_rpm - meas_rpm
        if meas_rpm < _RPM_STALL_RPM:
            # Stalled (not moving yet): start at the break-free kick and ramp the
            # duty quickly toward the cap so it breaks free even under a heavy
            # steering load. Feedback-driven, so this stops the instant it moves.
            # NOTE: broke_free is NOT cleared here. Under sustained heavy load
            # (e.g. reversing at full steering lock) the wheel can dip back
            # under the stall threshold and re-enter this branch mid-move; if
            # broke_free were reset to False, the next crossing back into the
            # "moving" branch below would reseed duty down to the light-load
            # ff_hold estimate, which is too low to sustain the wheel against
            # that load, so it stalls again immediately -- a self-inflicted
            # oscillation between ~35% and ~60-90% duty that can chatter for
            # many seconds (observed 2026-07-25: 40 rpm target reverse turn
            # at 55 deg lock, measured rpm stuck oscillating around 4-6 for
            # ~7s). Leaving broke_free True after the first break-free means
            # a re-stall just keeps ramping from its current (already-working)
            # duty instead of getting reseeded back down.
            _rpm_state["duty"] = min(cap, max(_rpm_state["duty"], kick_duty) + stall_ramp * dt)
            out = max(0.0, min(cap, _rpm_state["duty"]))
            # MUST command the motor here too. This used to live (dedented) after
            # the if/else so it covered both branches; when it was moved inside
            # the `else` the stall ramp became dead code -- the duty was computed
            # and logged but never written to the driver, so a wheel that dipped
            # below _RPM_STALL_RPM just held whatever was last commanded. Coming
            # out of a brake() that means the motor stays SHORTED while the log
            # prints "Duty: 100.0%", and it can only recover if something
            # externally spins the wheel back above the threshold.
            _apply_drive(motor_func, out, direction)
        else:
            # First step after breaking free: the ramped break-free duty (60..cap)
            # is usually well above the duty needed to merely HOLD the target,
            # especially under light load (e.g. wheels near-straight after the
            # camera wall-follow). Carrying it into the integrator makes the wheel
            # overshoot the target for the ~1-2 s the PI takes to bleed it back
            # down. So seed the integrator with a feed-forward hold estimate the
            # instant it starts moving, and it settles at the target immediately.
            # One-shot (broke_free): if a heavy load re-stalls the wheel and it
            # breaks free again we keep its higher duty rather than re-clamping,
            # so it doesn't chatter between the ramp and this seed.
            if not _rpm_state["broke_free"]:
                ff_hold = ff_static + ff_gain * target_rpm
                _rpm_state["duty"] = min(_rpm_state["duty"], ff_hold)
                _rpm_state["broke_free"] = True
            # Moving: PI trim on the rpm error eases the duty up/down to hold target.
            # If decelerating (err < 0), bleed accumulated duty 3x faster to eliminate overshoot lag
            if err < 0:
                _rpm_state["duty"] = max(0.0, _rpm_state["duty"] + 3.0 * ki_duty * err * dt)
            else:
                _rpm_state["duty"] = max(0.0, min(cap, _rpm_state["duty"] + ki_duty * err * dt))

            out = _rpm_state["duty"] + kp_duty * err
            if err < -20.0 and out <= 0.0:
                _apply_brake()
            else:
                out = max(0.0, min(cap, out))
                _apply_drive(motor_func, out, direction)
        _rpm_state["pulsing"] = True
    elif deficit > deadband:
        # Sub-stall regime: drive a fixed break-free pulse while behind.
        if not _rpm_state["pulsing"]:
            _apply_drive(motor_func, pulse_duty, direction)
            _rpm_state["pulsing"] = True
            _rpm_state["duty"] = pulse_duty
    else:
        # Sub-stall regime: caught up, brake and let the wheel coast.
        if _rpm_state["pulsing"]:
            _apply_brake()
            _rpm_state["pulsing"] = False
            _rpm_state["duty"] = 0.0

    # Running average rpm since start (robust, independent of per-step noise).
    elapsed = now - _rpm_state["start_time"]
    avg_rpm = (actual / cpr) * 60.0 / elapsed if elapsed > 0 else 0.0
    return avg_rpm


def _rpm_loop(stop_event, period, kwargs):
    """Background worker: holds the current target until stopped."""
    global _rpm_measured
    reset_rpm_control()
    step_count = 0
    while not stop_event.is_set():
        with _rpm_lock:
            target = _rpm_target
            direction = _rpm_dir
        rpm = set_speed_rpm(target, direction, **kwargs)
        if rpm is not None:
            with _rpm_lock:
                _rpm_measured = _rpm_state["ema_rpm"]
        
        step_count += 1
        if step_count >= 10:
            step_count = 0
            duty = _rpm_state.get("duty", 0.0)
            broke_free = _rpm_state.get("broke_free", False)
            measured = _rpm_measured
            # Duty = what the controller WANTS. Applied = what the driver last
            # actually got. If these disagree, believe Applied -- that is the
            # hardware truth, and a mismatch means a control path computed a
            # command without writing it.
            applied = _rpm_state.get("applied") or "none yet"
            print(f"[RPM Loop] Target: {target:.1f} RPM ({direction}) | Measured: {measured:.1f} RPM | Duty: {duty:.1f}% | Applied: {applied} | BrokeFree: {broke_free}")
            
        time.sleep(period)
    brake()
    reset_rpm_control()


def start_rpm_control(target_rpm, direction="forward", hz=50, **kwargs):
    """Start holding a WHEEL rpm in a background thread (run-once, no manual loop).

    After this, just call set_rpm_target() to change speed and get_measured_rpm()
    to read it; call stop_rpm_control() when finished. Extra kwargs (kp, ki,
    ff_gain, min_duty, kick_duty, kick_time) are forwarded to set_speed_rpm().
    """
    global _rpm_thread, _rpm_stop_event, _rpm_target, _rpm_dir
    if not encoder:
        print("ERROR: Encoder not initialized. Cannot start RPM control.")
        return False
    if _rpm_thread and _rpm_thread.is_alive():
        set_rpm_target(target_rpm, direction)
        return True
    with _rpm_lock:
        _rpm_target = target_rpm
        _rpm_dir = direction
    _rpm_stop_event = threading.Event()
    _rpm_thread = threading.Thread(
        target=_rpm_loop, args=(_rpm_stop_event, 1.0 / hz, kwargs), daemon=True
    )
    _rpm_thread.start()
    return True


def set_rpm_target(target_rpm, direction="forward"):
    """Update the target WHEEL rpm of the running background controller."""
    global _rpm_target, _rpm_dir
    with _rpm_lock:
        _rpm_target = target_rpm
        _rpm_dir = direction


def get_measured_rpm():
    """Most recent measured WHEEL rpm from the background controller."""
    with _rpm_lock:
        return _rpm_measured


def stop_rpm_control():
    """Stop the background controller and brake the motor."""
    global _rpm_thread, _rpm_stop_event
    if _rpm_stop_event:
        _rpm_stop_event.set()
    if _rpm_thread:
        _rpm_thread.join(timeout=1.0)
    _rpm_thread = None
    _rpm_stop_event = None
    brake()


def drive_distance(distance_cm, rpm=50.0, direction="forward", cap=75.0,
                   kp=0.15, ki=0.4, kick_duty=45.0, kick_time=0.25,
                   brake_lead_mm=None):
    """Drive a fixed distance at a target WHEEL rpm using continuous PWM.

    Closed-loop on speed: kicks to break free, then a PI loop trims the PWM duty
    to hold `rpm` while the encoder integrates distance. Holds the duty
    continuously (no pulsing), so use this for rpm ABOVE the ~40 wheel-rpm stall
    floor; below that the motor can't run steadily and you'd want the pulsing
    set_speed_rpm() instead.

        distance_cm:   distance to travel (cm).
        rpm:           target wheel rpm.
        direction:     "forward" or "reverse".
        cap:           max PWM duty (%) the loop may command (default 75).
        kp, ki:        PI gains on the rpm error (duty-% per wheel-rpm).
        kick_duty:     startup duty to break static friction.
        kick_time:     how long (s) to hold the kick before the PI takes over.
        brake_lead_mm: brake this many mm early to absorb coast-down overshoot.
                       Defaults to ~0.16*rpm mm (tuned: ~8 mm at 50 rpm).

    Returns the distance actually travelled (cm)."""
    if not encoder:
        print("ERROR: Encoder not initialized. Cannot use drive_distance().")
        return None

    target_mm = abs(distance_cm) * 10.0
    if brake_lead_mm is None:
        brake_lead_mm = 0.16 * rpm
    dir_cmd = 1.0 if direction == "forward" else -1.0
    motor_func = forward if direction == "forward" else reverse
    circ = math.pi * encoder.wheel_diameter_mm
    cpr = encoder.counts_per_rev
    # Signed counts -> mm in the commanded direction (negative if the wheel is
    # moving the wrong way, e.g. called mid-motion against residual momentum).
    counts_to_mm = dir_cmd * _ENC_FORWARD_SIGN / cpr * circ

    start_pos = encoder.position
    duty = kick_duty
    motor_func(duty)
    last_pos = start_pos
    t0 = time.time()
    lt = t0
    try:
        while True:
            time.sleep(0.04)
            now = time.time()
            dt = now - lt
            lt = now
            pos = encoder.position
            travelled = (pos - start_pos) * counts_to_mm
            if travelled >= target_mm - brake_lead_mm:
                break
            if now - t0 > kick_time:
                # Signed wheel rpm in the commanded direction: wrong-way motion
                # reads negative, so the PI loop drives harder the right way.
                rpm_meas = (pos - last_pos) * dir_cmd * _ENC_FORWARD_SIGN / cpr * 60.0 / dt
                err = rpm - rpm_meas
                duty += kp * err + ki * err * dt
                duty = max(8.0, min(cap, duty))
                motor_func(duty)
            last_pos = pos
    finally:
        brake()

    final_mm = (encoder.position - start_pos) * counts_to_mm
    print(f"drive_distance({distance_cm} cm @ {rpm} rpm) -> stopped at {final_mm / 10:.1f} cm")
    return final_mm / 10.0


def cleanup():
    """Stops the motor and releases GPIO resources (safe to call twice)."""
    global gpio_handle, motor_pwm
    print("--- Cleaning up Motor ---")
    # Stop the background RPM thread first so it can't issue a gpio_write on the
    # handle we're about to close (which would raise lgpio 'unknown handle').
    stop_rpm_control()
    if motor_pwm:
        try:
            forward(0)
            standby()
            motor_pwm.stop()
        except Exception:
            pass
        motor_pwm = None
    if gpio_handle is not None:
        # Null the handle BEFORE closing so any stray forward()/brake() (e.g. from
        # a late thread) sees a falsy handle and no-ops instead of using a dead one.
        h, gpio_handle = gpio_handle, None
        try:
            lgpio.gpiochip_close(h)
        except Exception:
            pass
    if encoder:
        try:
            encoder.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    print("--- Motor RPM Test ---")
    if not initialize():
        print("Motor test failed during initialization.")
    else:
        try:
            print("Starting motor at full speed (100%)...")
            forward(100)
            
            if encoder:
                last_pos = encoder.position
                last_time = time.time()
                
                print("Press Ctrl+C to stop.")
                while True:
                    time.sleep(0.5)
                    current_pos = encoder.position
                    current_time = time.time()
                    
                    delta_pos = current_pos - last_pos
                    dt = current_time - last_time
                    
                    if dt > 0:
                        # RPM = (Revolutions / Minute)
                        # Revolutions = delta_pos / counts_per_rev
                        # Minutes = dt / 60
                        rpm = (delta_pos / encoder.counts_per_rev) * (60.0 / dt)
                        print(f"RPM: {rpm:7.2f} | Position: {current_pos}")
                    
                    last_pos = current_pos
                    last_time = current_time
            else:
                print("ERROR: Encoder not initialized. Cannot measure RPM.")
                time.sleep(2)

        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()