"""
conveyor.py (Pi side)
The only program that opens the serial port to the Motion 2350 Pro
(firmware_conveyor.py on the board).

Manual mode (no flag): interactive speed/direction/start/stop, same as the
retired conveyor_only.py.

Auto mode (--auto): starts the belt when the Start sensor is seen, stops it
when the End sensor is seen, and waits for the Dobot to report "done" (or a
timer, per conveyor_settings.json release_mode) before arming the next cycle.
Auto status (sensors, running/stopped, speed, direction, object-at-end,
object id) is written atomically to state/conveyor_status.json on every
heartbeat, per STATE_CONTRACT.md.

Do NOT run this at the same time as piSerial.py's children: only one process
may open the serial port.

Usage:
    python3 conveyor.py                     # manual, interactive menu
    python3 conveyor.py --port /dev/ttyACM0
    python3 conveyor.py --no-autostart       # connect without sending settings
    python3 conveyor.py --daemon             # manual, no keyboard input
    python3 conveyor.py --auto               # auto mode (reads/writes state/)
    python3 conveyor.py --clear-fault        # clear a latched fault and exit
"""

import argparse
import json
import os
import sys
import threading
import time

import serial
import serial.tools.list_ports

import state_io

# =====================================
# CONFIGURATION
# =====================================

DEFAULT_BAUD  = 115200
BOARD_VID     = 0x2E8A      # Raspberry Pi / Motion 2350 Pro
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "conveyor_settings.json")

DEFAULT_SETTINGS = {
    "speed":            2000,
    "direction":        "FORWARD",
    "auto_start":       True,
    "min_speed":        50,
    "max_speed":        2000,
    "start_sensor":      "S1",
    "end_sensor":        "S2",
    "sensor_active_level": 1,
    "start_debounce_s": 0.2,
    "end_debounce_s":   0.05,
    "stop_settle_s":    0.5,
    "run_timeout_s":    60,
    "release_mode":     "dobot",
    "release_timer_s":  5,
    "detect_wait_s":    3.0,
}

# =====================================
# SETTINGS (shared file with piSerial.py)
# Unknown keys are preserved on save.
# =====================================

def load_settings():
    """Return settings dict. Missing keys are filled from defaults."""
    data = {}
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, "r") as f:
                data = json.load(f)
        except Exception as e:
            print("[SETTINGS] Cannot read {}: {} (using defaults)".format(SETTINGS_FILE, e))
            data = {}
    for k, v in DEFAULT_SETTINGS.items():
        data.setdefault(k, v)
    return data


def save_speed_direction(speed, direction):
    """Write speed + direction back, keeping all other keys untouched."""
    data = {}
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, "r") as f:
                data = json.load(f)
        except Exception as e:
            print("[SETTINGS] Cannot read existing file, not saving: {}".format(e))
            return False
    data["speed"] = speed
    data["direction"] = direction
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump(data, f, indent=4)
        print("[SETTINGS] Saved speed={} direction={}".format(speed, direction))
        return True
    except Exception as e:
        print("[SETTINGS] Save failed: {}".format(e))
        return False


def print_settings(cfg):
    print("[SETTINGS] {}".format(SETTINGS_FILE))
    for k, v in cfg.items():
        print("           {} = {}".format(k, v))

# =====================================
# SERIAL LINK
# =====================================

def find_port():
    """Auto-detect the board by USB vendor id."""
    for p in serial.tools.list_ports.comports():
        if p.vid == BOARD_VID:
            print("[AUTO] Found board at {}".format(p.device))
            return p.device
    return None


def parse_firmware_line(line):
    """Parse one line from the board into a dict, or None if not useful.
    Pure: no I/O. Handles STATUS, EVENT (sensor and WATCHDOG) and BOOT."""
    line = (line or "").strip()
    if not line:
        return None
    if line.startswith("STATUS"):
        fields = {}
        for part in line[len("STATUS"):].split():
            if "=" in part:
                k, v = part.split("=", 1)
                fields[k] = v
        return {"type": "STATUS", "fields": fields}
    if line.startswith("EVENT"):
        rest = line[len("EVENT"):].strip()
        if rest.upper().startswith("WATCHDOG"):
            return {"type": "EVENT_WATCHDOG"}
        fields = {}
        for part in rest.split():
            if "=" in part:
                k, v = part.split("=", 1)
                fields[k] = v
        return {"type": "EVENT_SENSOR", "fields": fields}
    if line.startswith("BOOT"):
        return {"type": "BOOT", "raw": line}
    return {"type": "OTHER", "raw": line}


class FirmwareLink:
    """Wraps a serial-like object (pyserial Serial or a FakeSerial in tests).

    serial_like only needs: write(bytes), readline()->bytes, reset_input_buffer()
    (optional), close() (optional).
    """

    def __init__(self, serial_like, max_speed=1800, clock=time.monotonic):
        self.ser = serial_like
        self.max_speed = max_speed
        self.clock = clock
        self._lock = threading.Lock()
        self._last_rx_monotonic = None
        self._rx_buf = ""

    def send(self, line):
        with self._lock:
            self.ser.write((line + "\n").encode("utf-8"))

    def set_speed(self, speed):
        speed = max(0, min(int(self.max_speed), int(speed)))
        self.send("SPEED:{}".format(speed))
        return speed

    def set_direction(self, direction):
        d = direction.upper()
        if d not in ("FORWARD", "BACKWARD"):
            return None
        self.send("DIR:{}".format(d))
        return d

    def stop(self):
        self.send("STOP")

    def request_status(self):
        self.send("STATUS")

    def ping(self):
        self.send("PING")

    def poll(self):
        """Read and parse any whole lines currently available. Returns a list
        of parsed events (see parse_firmware_line). Updates fw_fresh clock on
        any line received."""
        events = []
        if hasattr(self.ser, "in_waiting"):
            # Real serial: read only the bytes already buffered so poll() never
            # waits for the port timeout. Keeps the auto tick short (B1).
            n = self.ser.in_waiting
            if n:
                data = self.ser.read(n)
                self._rx_buf += data.decode("utf-8", errors="ignore") \
                    if isinstance(data, bytes) else data
            lines = []
            while "\n" in self._rx_buf:
                line, self._rx_buf = self._rx_buf.split("\n", 1)
                lines.append(line)
        else:
            # Fake serial in tests: readline() returns immediately (queued
            # lines or b""), so draining it this way stays non-blocking too.
            lines = []
            while True:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="ignore") if isinstance(raw, bytes) else raw
                if not line.strip():
                    break
                lines.append(line)
        for line in lines:
            if not line.strip():
                continue
            self._last_rx_monotonic = self.clock()
            parsed = parse_firmware_line(line)
            if parsed is not None:
                events.append(parsed)
        return events

    def fw_fresh(self, now=None, max_age_s=1.0):
        if self._last_rx_monotonic is None:
            return False
        now = now if now is not None else self.clock()
        return (now - self._last_rx_monotonic) <= max_age_s

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def connect(port=None, baud=DEFAULT_BAUD, timeout=1):
    """timeout=1 suits manual mode (blocking reader thread). Auto mode passes
    a tiny timeout so no call can stall the control loop."""
    if port is None:
        port = find_port()
    if port is None:
        print("[ERROR] Board not found. Use --port /dev/ttyACM0")
        sys.exit(1)
    try:
        ser = serial.Serial(port, baud, timeout=timeout, write_timeout=1)
    except serial.SerialException as e:
        print("[ERROR] Cannot open {}: {}".format(port, e))
        print("        Is piSerial.py or another program already using it?")
        sys.exit(1)
    time.sleep(1.5)
    ser.reset_input_buffer()
    print("[CONNECTED] {} @ {} baud".format(port, baud))
    return ser

# =====================================
# SENSOR DEBOUNCE
# =====================================

class SensorDebounce:
    """Logical sensor value becomes active only after `debounce_s` of a
    continuously active raw reading, and inactive the instant the raw
    reading clears (no debounce needed to clear)."""

    def __init__(self, debounce_s, clock=time.monotonic):
        self.debounce_s = debounce_s
        self.clock = clock
        self._raw_since = None
        self.active = False

    def update(self, raw_active):
        now = self.clock()
        if raw_active:
            if self._raw_since is None:
                self._raw_since = now
            if not self.active and (now - self._raw_since) >= self.debounce_s:
                self.active = True
        else:
            self._raw_since = None
            self.active = False
        return self.active

# =====================================
# CYCLE MACHINE (pure phase logic, auto mode)
# =====================================

class MotorCommand:
    """What the Pi wants the board to do right now."""

    def __init__(self, speed, direction):
        self.speed = speed
        self.direction = direction

    def __eq__(self, other):
        return isinstance(other, MotorCommand) and \
            self.speed == other.speed and self.direction == other.direction

    def __repr__(self):
        return "MotorCommand(speed={}, direction={})".format(self.speed, self.direction)


class CycleMachine:
    """Pure phase logic for auto mode (STATE_CONTRACT.md section 5).

    step(sensors, link_ok, fw_fresh, fw_speed, dobot_doc, dobot_fresh)
      -> (MotorCommand, status_body)

    sensors: {"start": bool, "end": bool, "valid": bool, "raw_s1": 0|1, "raw_s2": 0|1}
    dobot_doc: dict or None (parsed dobot_status.json body, or None if stale/missing)
    """

    def __init__(self, settings, clock=time.monotonic, id_source=None):
        self.settings = settings
        self.clock = clock
        self.id_source = id_source or (lambda: int(time.time()))
        self.phase = "idle"
        self.object_id = None
        self.last_object_id = None
        self.object_at_end = False
        self.pick_permission = False
        self.hold_reason = None
        self.fault = None
        self.cycle = {"started_ts": None, "end_ts": None, "stopped_ts": None, "released_ts": None}
        self._run_started_at = None
        self._commanded_speed = 0
        self._commanded_dir = self.settings.get("direction", "FORWARD")
        self._fw_fresh = False
        self._fw_speed = None

    def clear_fault(self):
        self.fault = None
        if self.phase == "fault":
            self.phase = "idle"
            self.object_id = None
            self.object_at_end = False
            self.pick_permission = False
            self._run_started_at = None

    def set_fault(self, fault):
        """Latch a fault and stop the motor (used for state_write_failed)."""
        self.fault = fault
        self.phase = "fault"
        self._commanded_speed = 0
        self.pick_permission = False

    def step(self, sensors, link_ok, fw_fresh, fw_speed, dobot_doc, dobot_fresh):
        now = self.clock()
        settings = self.settings
        self._fw_fresh = bool(fw_fresh)
        self._fw_speed = fw_speed

        if not link_ok:
            self.set_fault("serial_lost")
            return MotorCommand(0, self._commanded_dir), self._status_body(sensors, link_ok, fw_fresh, fw_speed, now)

        if self.fault == "serial_lost":
            # link is back: the operator state is unknown, so return to idle
            self.clear_fault()

        if not sensors.get("valid", True):
            # link not fresh enough to trust sensors: hold, do not advance
            return MotorCommand(self._commanded_speed, self._commanded_dir), \
                self._status_body(sensors, link_ok, fw_fresh, fw_speed, now)

        start_active = bool(sensors.get("start"))
        end_active = bool(sensors.get("end"))

        if self.phase == "idle":
            if end_active:
                self.phase = "blocked"
            elif start_active:
                self._arm_new_cycle(now)
        elif self.phase == "blocked":
            if not end_active:
                self.phase = "idle"
        elif self.phase == "running":
            if self._run_started_at is not None and \
                    (now - self._run_started_at) > settings.get("run_timeout_s", 60):
                self.set_fault("end_timeout")
            elif end_active:
                self._stop_at_end(now)
        elif self.phase == "at_end":
            self._update_at_end(now, dobot_doc, dobot_fresh)
        elif self.phase == "fault":
            pass  # stays until clear_fault()

        return MotorCommand(self._commanded_speed, self._commanded_dir), \
            self._status_body(sensors, link_ok, fw_fresh, fw_speed, now)

    def _arm_new_cycle(self, now):
        self.object_id = max(self.id_source(), (self.last_object_id or 0) + 1)
        self.last_object_id = self.object_id
        self.object_at_end = False
        self.pick_permission = False
        self.hold_reason = None
        self.cycle = {"started_ts": now, "end_ts": None, "stopped_ts": None, "released_ts": None}
        self._run_started_at = now
        self._commanded_speed = max(0, min(1800, int(self.settings.get("speed", 500)),
                                           int(self.settings.get("max_speed", 1800))))
        self._commanded_dir = self.settings.get("direction", "FORWARD")
        self.phase = "running"

    def _stop_at_end(self, now):
        self._commanded_speed = 0
        self.cycle["end_ts"] = now
        self.cycle["stopped_ts"] = now
        self.object_at_end = True
        self.phase = "at_end"

    def _update_at_end(self, now, dobot_doc, dobot_fresh):
        settings = self.settings
        stopped_ts = self.cycle.get("stopped_ts")
        settled = stopped_ts is not None and (now - stopped_ts) >= settings.get("stop_settle_s", 0.5)
        self.pick_permission = bool(
            settled and self._commanded_speed == 0 and self._fw_fresh and self._fw_speed == 0)

        release_mode = settings.get("release_mode", "dobot")
        if release_mode == "timer":
            self.hold_reason = None
            if settled and stopped_ts is not None and \
                    (now - stopped_ts) >= settings.get("release_timer_s", 5):
                self._release(now)
            return

        # release_mode == "dobot"
        if not dobot_fresh or dobot_doc is None:
            self.hold_reason = "dobot_stale"
            return
        if dobot_doc.get("state") == "fault":
            self.hold_reason = "dobot_fault"
            return
        handled_id = dobot_doc.get("handled_id")
        if dobot_doc.get("state") in ("done", "idle") and handled_id == self.object_id:
            self.hold_reason = None
            self._release(now)
        else:
            self.hold_reason = "waiting_dobot"

    def _release(self, now):
        self.cycle["released_ts"] = now
        self.phase = "idle"
        self.object_id = None
        self.object_at_end = False
        self.pick_permission = False
        self.hold_reason = None
        self._run_started_at = None

    def _status_body(self, sensors, link_ok, fw_fresh, fw_speed, now):
        return {
            "mode": "auto",
            "link": {"connected": bool(link_ok), "port": self.settings.get("_port"), "fw_fresh": bool(fw_fresh)},
            "sensors": {
                "start": bool(sensors.get("start")),
                "end": bool(sensors.get("end")),
                "raw_s1": sensors.get("raw_s1"),
                "raw_s2": sensors.get("raw_s2"),
                "valid": bool(sensors.get("valid", True)),
            },
            "motor": {
                "running": self._commanded_speed > 0,
                "speed": self._commanded_speed,
                "direction": self._commanded_dir,
                "fw_speed": fw_speed,
                "encoder": sensors.get("encoder"),
                "watchdog": sensors.get("watchdog", False),
            },
            "phase": self.phase,
            "object_id": self.object_id,
            "last_object_id": self.last_object_id,
            "object_at_end": self.object_at_end,
            "pick_permission": self.pick_permission,
            "cycle": dict(self.cycle),
            "hold_reason": self.hold_reason,
            "fault": self.fault,
        }

# =====================================
# MANUAL MODE (interactive, same as the retired conveyor_only.py)
# =====================================

_stop_reader = threading.Event()

def reader_thread(ser):
    while not _stop_reader.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            time.sleep(0.2)
            continue
        if not line:
            continue
        if line.startswith("EVENT"):
            print("[EVENT ] {}".format(line[len("EVENT"):].strip()))
        elif line.startswith("STATUS"):
            print("[STATUS] {}".format(line[len("STATUS"):].strip()))
        else:
            print("[BOARD ] {}".format(line))


MENU = """
Commands:
  s <value>   Set speed (steps/sec)
  f           Direction FORWARD
  b           Direction BACKWARD
  stop        Stop motor
  status      Ask the board for one status line
  show        Show settings file
  save        Save current speed + direction to settings file
  q           Quit (motor is stopped)
"""

class ManualKeepAlive:
    """Manual mode keep-alive: the board watchdog (1.5 s) stops the motor when
    no line arrives, so while the commanded speed is above 0 send PING every
    `period_s`. `speed` is updated by the menu (s / stop / auto-start)."""

    def __init__(self, link, period_s=0.5, clock=time.monotonic):
        self.link = link
        self.period_s = period_s
        self.clock = clock
        self.speed = 0
        self._last = clock()
        self._stop = threading.Event()

    def tick(self):
        """Send one PING if due. Returns True when a PING was sent."""
        now = self.clock()
        if self.speed > 0 and (now - self._last) >= self.period_s:
            self._last = now
            try:
                self.link.ping()
            except Exception as e:
                print("[SERIAL] keep-alive failed: {}".format(e))
            return True
        return False

    def mark_sent(self):
        """A real command was just sent, so it already fed the watchdog."""
        self._last = self.clock()

    def run(self):
        while not self._stop.is_set():
            self.tick()
            self._stop.wait(0.1)

    def stop(self):
        self._stop.set()


def interactive(link, cfg, keepalive=None):
    print(MENU)
    current = {"speed": int(cfg["speed"]), "direction": cfg["direction"]}
    link.max_speed = cfg["max_speed"]

    def sync():
        if keepalive is not None:
            keepalive.speed = current["speed"]
            keepalive.mark_sent()

    def set_direction(direction):
        d = direction.upper()
        if d not in ("FORWARD", "BACKWARD"):
            print("[ERROR] direction must be FORWARD or BACKWARD")
            return None
        link.send("DIR:{}".format(d))
        time.sleep(0.05)
        return d

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd == "q":
            break
        elif cmd == "s":
            if len(parts) < 2:
                print("Usage: s <speed>")
                continue
            try:
                current["speed"] = link.set_speed(int(parts[1]))
                time.sleep(0.05)
                sync()
            except ValueError:
                print("Speed must be a whole number")
        elif cmd == "f":
            current["direction"] = set_direction("FORWARD") or current["direction"]
        elif cmd == "b":
            current["direction"] = set_direction("BACKWARD") or current["direction"]
        elif cmd == "stop":
            link.stop()
            time.sleep(0.05)
            current["speed"] = 0
            sync()
        elif cmd == "status":
            link.request_status()
            time.sleep(0.05)
        elif cmd == "show":
            print_settings(load_settings())
        elif cmd == "save":
            save_speed_direction(current["speed"], current["direction"])
        else:
            print("Unknown command.")


def daemon_mode():
    print("[DAEMON] Conveyor only. Waiting for sensor events. Ctrl+C / SIGTERM to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


def run_manual(args, cfg):
    ser = connect(args.port, args.baud)

    link = FirmwareLink(ser, max_speed=min(int(cfg["max_speed"]), 1800))
    keepalive = ManualKeepAlive(link)

    reader = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    reader.start()
    threading.Thread(target=keepalive.run, daemon=True).start()

    if not args.no_autostart and cfg.get("auto_start", True):
        speed = link.set_speed(int(cfg["speed"]))
        time.sleep(0.1)
        link.send("DIR:{}".format(cfg["direction"]))
        time.sleep(0.1)
        keepalive.speed = speed
        keepalive.mark_sent()
        print("[SETTINGS] Applied speed={} direction={}".format(speed, cfg["direction"]))

    try:
        if args.daemon or not sys.stdin.isatty():
            daemon_mode()
        else:
            interactive(link, cfg, keepalive)
    finally:
        keepalive.stop()
        try:
            link.stop()
        except Exception:
            pass
        _stop_reader.set()
        time.sleep(0.2)
        ser.close()
        print("[DISCONNECTED] motor stopped")

# =====================================
# AUTO MODE
# =====================================

class AutoRuntime:
    """Bundles the pieces of run_auto's loop state that must persist across
    polls: the CycleMachine, the sensor debouncers, the last fields seen from
    the board and the last command actually sent. Building one explicitly
    (instead of always creating fresh state inside run_auto) lets tests call
    run_auto repeatedly to script a sequence of board readings."""

    def __init__(self, cfg, clock=time.monotonic):
        self.cfg = cfg
        self.clock = clock
        self.start_level = int(cfg.get("sensor_active_level", 1))
        self.start_debounce = SensorDebounce(cfg.get("start_debounce_s", 0.2), clock=clock)
        self.end_debounce = SensorDebounce(cfg.get("end_debounce_s", 0.05), clock=clock)
        self.machine = CycleMachine(cfg, clock=clock)
        self.dobot_fresh = state_io.Freshness(3.0, clock=clock)
        self.last_fields = {}
        self.last_speed_sent = None
        self.last_dir_sent = None
        self.last_keepalive = clock()
        self.last_status_poll = clock()
        self.write_failures = 0


def run_auto(link, writer, cfg, state_dir, clock=time.monotonic, sleep=time.sleep,
             max_ticks=None, runtime=None):
    """The auto loop: poll the board, step the CycleMachine, write status.
    Runs until KeyboardInterrupt / SIGTERM (or max_ticks loops, used by tests).

    Pass `runtime` (an AutoRuntime) to keep phase/object_id/debounce state
    across repeated calls, as tests do to script a sequence of board readings.
    """
    ticks = 0
    if runtime is None:
        runtime = AutoRuntime(cfg, clock=clock)
    start_level = runtime.start_level
    start_debounce = runtime.start_debounce
    end_debounce = runtime.end_debounce
    machine = runtime.machine

    dobot_fresh = runtime.dobot_fresh
    dobot_path = os.path.join(state_dir, "dobot_status.json")

    last_fields = runtime.last_fields
    last_speed_sent = runtime.last_speed_sent
    last_dir_sent = runtime.last_dir_sent
    last_keepalive = runtime.last_keepalive
    last_status_poll = runtime.last_status_poll
    write_failures = runtime.write_failures

    link.request_status()
    clear_flag_path = os.path.join(state_dir, "clear_conveyor.flag")

    try:
        while True:
            if os.path.exists(clear_flag_path):
                machine.clear_fault()
                try:
                    os.remove(clear_flag_path)
                except OSError:
                    pass

            link_ok = True
            try:
                events = link.poll()
            except (OSError, serial.SerialException) as e:
                print("[SERIAL] read failed: {}".format(e))
                events = []
                link_ok = False

            for ev in events:
                if ev["type"] == "STATUS":
                    last_fields.update(ev["fields"])
                elif ev["type"] == "EVENT_SENSOR":
                    last_fields.update(ev["fields"])
                elif ev["type"] == "EVENT_WATCHDOG":
                    last_fields["WD"] = "1"
                    last_fields["SPD"] = "0"
                    last_speed_sent = None      # force the command to be resent

            raw_s1 = last_fields.get("S1")
            raw_s2 = last_fields.get("S2")
            fw_ok = link.fw_fresh(now=clock())
            valid = fw_ok and raw_s1 is not None and raw_s2 is not None

            if valid:
                s1_active = int(raw_s1) == start_level
                s2_active = int(raw_s2) == start_level
            else:
                s1_active = False
                s2_active = False

            active = {"S1": s1_active, "S2": s2_active}
            sensors = {
                "start": start_debounce.update(active.get(cfg.get("start_sensor", "S1"), False)),
                "end": end_debounce.update(active.get(cfg.get("end_sensor", "S2"), False)),
                "raw_s1": int(raw_s1) if raw_s1 is not None else None,
                "raw_s2": int(raw_s2) if raw_s2 is not None else None,
                "valid": valid,
                "encoder": int(last_fields["ENC"]) if "ENC" in last_fields else None,
                "watchdog": last_fields.get("WD") == "1",
            }
            fw_speed = int(last_fields["SPD"]) if "SPD" in last_fields else None

            dobot_doc = state_io.read_json(dobot_path, "dobot_status/1")
            dobot_fresh.update(dobot_doc)

            cmd, body = machine.step(sensors, link_ok=link_ok, fw_fresh=fw_ok, fw_speed=fw_speed,
                                      dobot_doc=dobot_doc, dobot_fresh=dobot_fresh.is_fresh())

            now = clock()
            try:
                if cmd.speed != last_speed_sent or cmd.direction != last_dir_sent:
                    if cmd.speed <= 0:
                        link.stop()
                    else:
                        link.set_direction(cmd.direction)
                        link.set_speed(cmd.speed)
                    last_speed_sent = cmd.speed
                    last_dir_sent = cmd.direction
                    last_keepalive = now
                elif cmd.speed > 0 and (now - last_keepalive) >= 0.5:
                    link.set_speed(cmd.speed)
                    last_keepalive = now

                if (now - last_status_poll) >= 0.2:
                    link.request_status()
                    last_status_poll = now
            except (OSError, serial.SerialException) as e:
                print("[SERIAL] write failed: {}".format(e))
                machine.set_fault("serial_lost")

            try:
                writer.write(body)
                write_failures = 0
            except Exception as e:
                write_failures += 1
                print("[STATE] write failed ({}): {}".format(write_failures, e))
                if write_failures >= 10 and machine.fault != "state_write_failed":
                    machine.set_fault("state_write_failed")
                    link.stop()

            ticks += 1
            if max_ticks is not None and ticks >= max_ticks:
                break
            sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        runtime.last_speed_sent = last_speed_sent
        runtime.last_dir_sent = last_dir_sent
        runtime.last_keepalive = last_keepalive
        runtime.last_status_poll = last_status_poll
        runtime.write_failures = write_failures
        if max_ticks is None:       # tests (max_ticks set) keep the motor state between calls
            try:
                link.stop()
            except Exception:
                pass

# =====================================
# MAIN
# =====================================

def main():
    parser = argparse.ArgumentParser(description="Conveyor (Pi) serial controller")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int)
    parser.add_argument("--no-autostart", action="store_true",
                        help="Do not send speed/direction from the settings file on connect")
    parser.add_argument("--daemon", action="store_true",
                        help="No keyboard input (also automatic when stdin is not a terminal). Manual mode only.")
    parser.add_argument("--auto", action="store_true", help="Auto mode: run from sensors and shared state")
    parser.add_argument("--state-dir", default=None, help="Shared state folder (see STATE_CONTRACT.md)")
    parser.add_argument("--clear-fault", action="store_true",
                        help="Create state/clear_conveyor.flag and exit")
    args = parser.parse_args()

    if args.clear_fault:
        state_dir = state_io.resolve_state_dir(args.state_dir)
        flag = os.path.join(state_dir, "clear_conveyor.flag")
        open(flag, "w").close()
        print("[CLEAR] wrote {}".format(flag))
        return

    cfg = load_settings()
    print_settings(cfg)

    if not args.auto:
        run_manual(args, cfg)
        return

    state_dir = state_io.resolve_state_dir(args.state_dir)
    lock = state_io.acquire_single_instance("conveyor", state_dir)
    if lock is None:
        print("[ERROR] another conveyor.py --auto is already running (lock held)")
        sys.exit(1)

    ser = connect(args.port, args.baud, timeout=0)
    cfg["_port"] = ser.port
    link = FirmwareLink(ser, max_speed=min(cfg.get("max_speed", 1800), 1800))
    writer = state_io.StateWriter("conveyor_status", "conveyor.py", state_dir)

    try:
        run_auto(link, writer, cfg, state_dir)
    finally:
        link.close()
        print("[DISCONNECTED]")


if __name__ == "__main__":
    main()
