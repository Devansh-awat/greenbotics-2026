import time
from src.motors import motor, servo

def stage(n, msg, action):
    print(f"\n######## STAGE {n}: {msg} ########", flush=True)
    print(f"   motor ON in 1s...", flush=True); time.sleep(1)
    action()
    print(f"   >>> STAGE {n} RUNNING NOW (3s) <<<", flush=True)
    time.sleep(3)
    motor.brake()
    print(f"   stage {n} stopped. (pause)", flush=True)
    time.sleep(2.5)

motor.initialize()
stage(1, "NO servo -> forward(100)", lambda: motor.forward(100))
stage(2, "NO servo -> forward(65)",  lambda: motor.forward(65))
print("\n*** initializing servo now ***", flush=True)
servo.initialize()
stage(3, "servo ON -> forward(100)", lambda: motor.forward(100))
stage(4, "servo ON -> forward(65)",  lambda: motor.forward(65))
motor.brake(); motor.cleanup(); servo.cleanup()
print("\nDONE", flush=True)
