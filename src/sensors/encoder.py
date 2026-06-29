import array
import sys
import math

import adafruit_pioasm

if sys.implementation.name == 'circuitpython':
    from rp2pio import StateMachine
    _n_read = 9
else:
    from adafruit_rp1pio import StateMachine
    _n_read = 17

_program = adafruit_pioasm.Program("""
;
; Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
;
; SPDX-License-Identifier: BSD-3-Clause
;
.pio_version 0 // only requires PIO version 0

.program quadrature_encoder

; the code must be loaded at address 0, because it uses computed jumps
.origin 0


; the code works by running a loop that continuously shifts the 2 phase pins into
; ISR and looks at the lower 4 bits to do a computed jump to an instruction that
; does the proper "do nothing" | "increment" | "decrement" action for that pin
; state change (or no change)

; ISR holds the last state of the 2 pins during most of the code. The Y register
; keeps the current encoder count and is incremented / decremented according to
; the steps sampled

; the program keeps trying to write the current count to the RX FIFO without
; blocking. To read the current count, the user code must drain the FIFO first
; and wait for a fresh sample (takes ~4 SM cycles on average). The worst case
; sampling loop takes 10 cycles, so this program is able to read step rates up
; to sysclk / 10  (e.g., sysclk 125MHz, max step rate = 12.5 Msteps/sec)

; 00 state
    jmp update    ; read 00
    jmp decrement ; read 01
    jmp increment ; read 10
    jmp update    ; read 11

; 01 state
    jmp increment ; read 00
    jmp update    ; read 01
    jmp update    ; read 10
    jmp decrement ; read 11

; 10 state
    jmp decrement ; read 00
    jmp update    ; read 01
    jmp update    ; read 10
    jmp increment ; read 11

; to reduce code size, the last 2 states are implemented in place and become the
; target for the other jumps

; 11 state
    jmp update    ; read 00
    jmp increment ; read 01
decrement:
    ; note: the target of this instruction must be the next address, so that
    ; the effect of the instruction does not depend on the value of Y. The
    ; same is true for the "jmp y--" below. Basically "jmp y--, <next addr>"
    ; is just a pure "decrement y" instruction, with no other side effects
    jmp y--, update ; read 10

    ; this is where the main loop starts
.wrap_target
update:
    mov isr, y      ; read 11
    push noblock

sample_pins:
    ; we shift into ISR the last state of the 2 input pins (now in OSR) and
    ; the new state of the 2 pins, thus producing the 4 bit target for the
    ; computed jump into the correct action for this state. Both the PUSH
    ; above and the OUT below zero out the other bits in ISR
    out isr, 2
    in pins, 2

    ; save the state in the OSR, so that we can use ISR for other purposes
    mov osr, isr
    ; jump to the correct state machine action
    mov pc, isr

    ; the PIO does not have a increment instruction, so to do that we do a
    ; negate, decrement, negate sequence
increment:
    mov y, ~y
    jmp y--, increment_cont
increment_cont:
    mov y, ~y
.wrap    ; the .wrap here avoids one jump instruction and saves a cycle too
""")

_zero_y = adafruit_pioasm.assemble("set y 0")

class IncrementalEncoder:
    def __init__(self, pin_a, pin_b=None, divisor=1, counts_per_rev=44 * 4.2 * 38 / 13, gear_ratio=1.0, wheel_diameter_mm=62.4):
        """Create an incremental encoder on pin_a and the next higher pin

        Always operates in "x4" mode (one count per quadrature edge).

        counts_per_rev is the full x4 resolution per WHEEL revolution:
            44 counts/motor-rev (x4)
            * 4.2  internal gearbox reduction
            * 38/13 external gear reduction (13T motor -> 38T wheel)
            = 540.18 counts per wheel revolution.
        gear_ratio is therefore folded in and defaults to 1.0, so position /
        counts_per_rev gives wheel revolutions directly.

        Assumes but does not check that pin_b is one above pin_a."""
        self._sm = StateMachine(
            _program.assembled,
            frequency=0,
            init=_zero_y,
            first_in_pin=pin_a,
            in_pin_count=2,
            pull_in_pin_up=0x3,
            auto_push=True,
            push_threshold=32,
            in_shift_right=False,
            **_program.pio_kwargs
        )
        self._buffer = array.array('i',[0] * _n_read)
        self.divisor = divisor
        self.counts_per_rev = counts_per_rev
        self.gear_ratio = gear_ratio
        self.wheel_diameter_mm = wheel_diameter_mm
        self._position = 0

    def deinit(self):
        self._sm.deinit()

    @property
    def position(self):
        self._sm.readinto(self._buffer) # read N stale values + 1 fresh value
        raw_position = self._buffer[-1]
        delta = int((raw_position - self._position * self.divisor) / self.divisor)
        self._position += delta
        return self._position

    @property
    def distance(self):
        """Returns the distance traveled in mm."""
        return self.position * (self.gear_ratio * math.pi * self.wheel_diameter_mm) / self.counts_per_rev

if __name__ == '__main__':
    import board
    # D20/D21 on header pins 38/40
    # GND on header pin 39
    # +5V on header pins 2/4
    q = IncrementalEncoder(board.D20)
    old_position = q.position
    while True:
        position = q.position
        distance = q.distance
        if position != old_position:
            delta = position - old_position
            print(f"{position:8d} {delta=:+4d}  {distance:8.2f} mm")
        old_position = position