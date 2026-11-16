import sys
import select
from machine import Pin
import time

# =====================================
# firmware_conveyor.py  (MicroPython, Motion 2350 Pro)
# Copy to the board as main.py.
#
# Pure reporter and driver:
#   - reports sensor status to the Pi over USB serial
#   - receives speed and direction from the Pi and drives the stepper
#   - NO start/stop decision logic here (it lives in conveyor.py on the Pi)
#   - watchdog: if the motor is running and no command line arrives for
#     WATCHDOG_MS, the speed is set to 0 until the Pi sends a new SPEED
# =====================================

# =====================================
# GPIO CONFIGURATION
# =====================================

# Digital sensors
sensor1 = Pin(26, Pin.IN)
sensor2 = Pin(27, Pin.IN)

# Stepper motor driver
step_pin = Pin(0, Pin.OUT)
dir_pin  = Pin(1, Pin.OUT)

# Encoder
enc_a = Pin(4, Pin.IN)
enc_b = Pin(5, Pin.IN)

# =====================================
# SETTINGS
# =====================================

FW_VERSION = 2

# Change to 0 if your sensors are active LOW
SENSOR_ACTIVE = 1

# Motor speed limits (steps per second)
MIN_SPEED = 50
MAX_SPEED = 1800

# Step pulse width in microseconds
PULSE_WIDTH_US = 5

# Stop the motor if no command line was received for this long
WATCHDOG_MS = 1500

# Direction polarity
FORWARD = 1
BACKWARD = 0

# =====================================
# ENCODER COUNTER
# =====================================

encoder_count = 0

def encoder_callback(pin):
    global encoder_count
    if enc_b.value():
        encoder_count += 1
    else:
        encoder_count -= 1

enc_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)

# =====================================
# COMMAND PARSER
#
#  Accepted commands (from Raspberry Pi via serial):
#    S <value>       Set speed  e.g. S 1000
#    SPEED:<value>   Set speed  e.g. SPEED:1000
#    F               Direction forward
#    B               Direction backward
#    DIR:FORWARD     Direction forward
#    DIR:BACKWARD    Direction backward
#    STOP            Stop motor
#    STATUS          Request status reply (one line)
#    PING            No reply, only refreshes the watchdog
#
#  Output (sent to Pi):
#    BOOT fw=2                                (once at start-up)
#    EVENT S1=x S2=x SENSOR=DETECTED|CLEAR    (on sensor change only)
#    EVENT WATCHDOG                           (once when the watchdog stops the motor)
#    STATUS S1=.. S2=.. SENSOR=.. ENC=.. SPD=.. DIR=.. WD=0|1
#                                             (only when STATUS command received)
# =====================================

def new_state():
    return {"speed": 0, "direction": FORWARD, "wd": 0, "want_status": False}


def parse_command(line, state):
    """Apply one command line to state (pure: no I/O). Returns state."""
    cmd   = line.strip()
    parts = cmd.split()
    if not parts:
        return state
    verb = parts[0].upper()

    if verb == "STOP":
        state["speed"] = 0

    elif verb in ("SPEED", "S") or verb.startswith("SPEED:") or verb.startswith("S:"):
        raw    = cmd.upper().replace("SPEED:", " ").replace("S:", " ").strip()
        tokens = raw.split()
        try:
            val            = int(tokens[-1])
            state["speed"] = max(0, min(MAX_SPEED, val))
            state["wd"]    = 0       # a new SPEED re-arms after a watchdog stop
        except:
            pass

    elif verb.startswith("DIR:"):
        d = verb.replace("DIR:", "")
        if d in ("FORWARD", "F"):
            state["direction"] = FORWARD
        elif d in ("BACKWARD", "B", "BACK"):
            state["direction"] = BACKWARD

    elif verb == "F":
        state["direction"] = FORWARD

    elif verb in ("B", "BACK"):
        state["direction"] = BACKWARD

    elif verb == "STATUS":
        state["want_status"] = True

    # PING and unknown verbs: nothing to apply (watchdog is refreshed by the caller)
    return state


def watchdog_expired(now, last_cmd, speed, limit):
    """True when the motor should run but no command arrived for `limit` ms.
    last_cmd is None until the first command (no countdown at boot)."""
    if last_cmd is None or speed <= 0:
        return False
    return time.ticks_diff(now, last_cmd) > limit


def sensor_text(s1, s2):
    return "DETECTED" if (s1 == SENSOR_ACTIVE or s2 == SENSOR_ACTIVE) else "CLEAR"


def status_line(state, s1, s2):
    d = "FORWARD" if state["direction"] == FORWARD else "BACKWARD"
    return "STATUS S1={} S2={} SENSOR={} ENC={} SPD={} DIR={} WD={}".format(
        s1, s2, sensor_text(s1, s2), encoder_count, state["speed"], d, state["wd"]
    )

# =====================================
# MAIN PROGRAM
# =====================================

def main():
    state = new_state()
    rx_buffer = ""
    last_cmd = None

    step_pin.value(0)
    last_step_time = time.ticks_us()
    step_high_time = 0
    step_active    = False

    last_s1 = sensor1.value()
    last_s2 = sensor2.value()

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)

    print("BOOT fw={}".format(FW_VERSION))

    while True:

        # ---- Read USB serial (non-blocking) ----
        if poll.poll(0):
            char = sys.stdin.read(1)
            if char in ('\n', '\r'):
                if rx_buffer:
                    parse_command(rx_buffer, state)
                    rx_buffer = ""
                    last_cmd = time.ticks_ms()      # any complete line feeds the watchdog
                    if state["want_status"]:
                        state["want_status"] = False
                        print(status_line(state, sensor1.value(), sensor2.value()))
            else:
                rx_buffer += char

        # ---- Watchdog ----
        if watchdog_expired(time.ticks_ms(), last_cmd, state["speed"], WATCHDOG_MS):
            state["speed"] = 0
            state["wd"] = 1
            print("EVENT WATCHDOG")

        # ---- Read sensors ----
        s1 = sensor1.value()
        s2 = sensor2.value()

        # Send event only when sensor state changes
        if s1 != last_s1 or s2 != last_s2:
            print("EVENT S1={} S2={} SENSOR={}".format(s1, s2, sensor_text(s1, s2)))
            last_s1 = s1
            last_s2 = s2

        # ---- Determine effective speed ----
        speed     = state["speed"]
        direction = state["direction"]

        # Clamp to valid range
        if 0 < speed < MIN_SPEED:
            speed = MIN_SPEED

        # ---- Generate step pulses ----
        if speed > 0:
            dir_pin.value(direction)
            interval_us = 1000000 // speed
            now         = time.ticks_us()

            if step_active:
                if time.ticks_diff(now, step_high_time) >= PULSE_WIDTH_US:
                    step_pin.value(0)
                    step_active = False
            else:
                if time.ticks_diff(now, last_step_time) >= interval_us:
                    step_pin.value(1)
                    step_active    = True
                    step_high_time = now
                    last_step_time = now
        else:
            step_pin.value(0)
            step_active    = False
            last_step_time = time.ticks_us()

        time.sleep_us(50)


if __name__ == "__main__":
    main()
