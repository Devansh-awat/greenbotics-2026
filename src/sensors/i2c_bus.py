# -*- coding: utf-8 -*-
"""
Shared I2C bus and lock for all devices on physical bus 1.

The TCA9548A mux (0x70) + VL53L1X sensors (0x29) and the BNO055 IMU (0x28) all
live on the same physical I2C bus. They are read from different threads
(SensorThread vs ImuThread). Previously each module created its own
``busio.I2C(board.SCL, board.SDA)`` handle, giving each its own software lock;
those locks don't know about each other, so the threads could drive the wire
concurrently and corrupt transactions (raising OSError).

This module gives everyone ONE bus object (``board.I2C()`` is a cached
singleton) plus ONE process-wide lock to serialize transactions across threads.
Every high-level read that touches the bus must be wrapped in ``with LOCK:``.
"""

import threading
import board

# Re-entrant so a locked code path can call another helper that also locks
# (same thread) without deadlocking.
LOCK = threading.RLock()


def get_bus():
    """Return the shared, cached I2C bus singleton (physical bus 1)."""
    return board.I2C()
