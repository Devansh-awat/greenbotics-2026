"""Quick bench test: hold the wheel at 10 rpm using closed-loop control.

Run with wheels lifted off the ground. Ctrl+C to stop early.
"""
import time
from src.motors import motor


def main():
    if not motor.initialize():
        print("Init failed.")
        return
    if not motor.encoder:
        print("ERROR: encoder not available, aborting.")
        motor.cleanup()
        return

    target = 10.0  # wheel rpm
    print(f"Target: {target} wheel rpm. Running 5 s...")
    motor.start_rpm_control(target)
    try:
        t_end = time.time() + 5.0
        while time.time() < t_end:
            time.sleep(0.25)
            print(f"  measured: {motor.get_measured_rpm():6.2f} wheel rpm")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        motor.stop_rpm_control()
        motor.cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
