"""
dobot.py
Dobot MG400 pick-and-place cycle for the conveyor cell (SAFETY-CRITICAL).

The Dobot does NO detection thinking. It picks at a FIXED point (X,Y) =
(300, 0) at the conveyor End sensor and places downstream. The only inputs
are the conveyor status file (is an object waiting at End, is picking
allowed) and the detection result file written by camera.py (height and
along-belt length, used for Z and for validation only).

Manual (no flag):   python dobot.py            menu, dry run unless --live
                    python dobot.py --once --height 40 [--length 100]
Auto:               python dobot.py --auto     waits for the conveyor, writes
                                               state/dobot_status.json

SAFETY MODEL
    * DRY RUN is the default. In dry run NO socket is opened: every command that
      WOULD be sent is only logged (and written to dobot_last_cycle.json).
    * Live mode must be requested explicitly (PickPlaceController(live=True)),
      this file only does that with --live AND dobot_settings.json
      "dobot_live": true (piSerial.py passes --live when given --dobot-live).
    * The whole plan is validated BEFORE the first command is sent.
    * Any error in live mode -> robot stop, `fault` flag set, automatic and manual
      cycles are refused until dclear. The vacuum is NEVER released by software on
      a fault (a held object must not be dropped silently).

Protocol notes (verified against Dobot-Arm/TCP-IP-4Axis-Python dobot_api.py):
    Dashboard port 29999: EnableRobot(), DisableRobot(), ClearError(), ResetRobot(),
        SpeedFactor(n), DO(index,status), RobotMode(), GetErrorID(), GetPose(),
        EmergencyStop()
    Motion port 30003:    MovJ(x,y,z,r), MovL(x,y,z,r), Sync()   (Sync is a method
        of the MOVE class in dobot_api.py -> sent on 30003 and blocks until the
        motion queue is finished)
    Reply format: "ErrorID,{v1,v2,...},CmdName(args);"  ErrorID 0 = success.
    NOT verified (official PDF could not be read by the tooling): the exact
    semantics of EmergencyStop() on MG400 firmware, that the controller accepts a
    SECOND concurrent dashboard connection (stop therefore uses the existing
    socket first), that RobotMode 5 = enabled,
    and whether DO() issued on 29999 is queued with 30003 motion (we therefore
    Sync() after every motion before any DO()).
    The feedback port (30004) is intentionally NOT used.

Coordinates (user convention): X+ left, X- right; Y+ front, Y- back; Z+ up.
"""

import argparse
import collections
import json
import math
import os
import re
import socket
import sys
import threading
import time
from datetime import datetime

import state_io

# =====================================
# CONSTANTS (all in mm / seconds unless noted)
# =====================================

DOBOT_IP = "192.168.1.6"
DASHBOARD_PORT = 29999
MOTION_PORT = 30003
FEEDBACK_PORT = 30004          # documented only, never opened

CONNECT_TIMEOUT_S = 3.0        # every socket connect
CMD_TIMEOUT_S = 5.0            # every normal command reply
SYNC_TIMEOUT_S = 90.0          # Sync() blocks until the queue is done
ENABLE_WAIT_S = 10.0           # wait for the robot to report "enabled"
ENABLED_MODE = 5               # RobotMode() value for ENABLED. UNVERIFIED assumption.

# Conveyor travel direction. ASSUMPTION: belt moves toward Y+ (front). The place
# move is Y+ (forward) so the object ends up downstream of the end sensor S2.
CONVEYOR_FORWARD_AXIS = "+Y"

BELT_Z_MM = -100.0             # robot Z where the cup touches the belt surface (gating reference only)
PICK_X_MM = 300.0              # FIXED pick point X (belt centre line)
PICK_Y_MM = 0.0                # FIXED pick point Y (same position as the End sensor)
PICK_Z_MM = -100.0             # FIXED pick/place Z; the arm position never moves with height
LIFT_MM = 20.0                 # clearance above pick/place Z for travel moves
PLACE_CLEARANCE_MM = 20.0      # extra forward distance so trailing edge clears S2
SETTLE_S = 0.5                 # wait after vacuum ON before lifting
BLOW_MS = 300                  # blow-off pulse length
# SAFE / HOME pose (above belt, near the end sensor). R = tool rotation, kept 0.
HOME_X_MM, HOME_Y_MM, HOME_Z_MM, HOME_R = 300.0, 0.0, 120.0, 0.0

# Digital outputs. ASSUMPTION (flagged): user's "D1 vacuum / D2 blow" are the
# controller DO1 / DO2 used by DO(index,status). Check wiring before live use.
VACUUM_DO = 1
BLOW_DO = 2
# Manual "do <index> <0|1>" accepts 1..MANUAL_DO_MAX (an integer check, not a
# wiring check). UNVERIFIED: the MG400 controller has 16 DO terminals; lower
# manual_do_max in the config if fewer are wired. The automatic cycle is NOT
# affected: validate_plan still allows only VACUUM_DO / BLOW_DO.
MANUAL_DO_MAX = 16

# Speed. SpeedFactor is sent before any motion; never above the hard cap.
DOBOT_SPEED_PCT = 10
SPEED_HARD_CAP_PCT = 30

# Envelope. Z_MIN is the hard floor (belt surface). MG400 max reach is 440 mm
# (Dobot product page / spec sheet); we use a smaller default. REACH_MIN is an
# UNVERIFIED conservative guess for the inner dead zone around the base.
Z_MIN_MM = -100.0
Z_MAX_MM = 400.0
REACH_MAX_MM = 400.0
REACH_MIN_MM = 150.0
Y_LIMIT_MM = 200.0             # |Y| must not exceed this

# Scan gating
MAX_OBJECT_HEIGHT_MM = 400.0
MIN_L_MM = 20.0
MAX_L_MM = 200.0
MAX_WAIT_S = 10.0              # plan sanity limit for any single wait

# Auto mode
DETECT_WAIT_S = 3.0            # wait this long for the matching detection result
HEARTBEAT_S = 0.5              # dobot_status.json rewrite period
POLL_S = 0.1

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(BASE_DIR, "dobot_settings.json")
LAST_CYCLE_FILE = os.path.join(state_io.DEFAULT_STATE_DIR, "dobot_last_cycle.json")


def default_config():
    """Mutable copy of the constants above (tests / piSerial override entries)."""
    return {
        "ip": DOBOT_IP, "dashboard_port": DASHBOARD_PORT, "motion_port": MOTION_PORT,
        "connect_timeout_s": CONNECT_TIMEOUT_S, "cmd_timeout_s": CMD_TIMEOUT_S,
        "sync_timeout_s": SYNC_TIMEOUT_S, "enable_wait_s": ENABLE_WAIT_S,
        "enabled_mode": ENABLED_MODE,
        "belt_z": BELT_Z_MM, "pick_x": PICK_X_MM, "pick_y": PICK_Y_MM, "pick_z": PICK_Z_MM,
        "lift": LIFT_MM,
        "place_clearance": PLACE_CLEARANCE_MM, "settle_s": SETTLE_S, "blow_ms": BLOW_MS,
        "home": (HOME_X_MM, HOME_Y_MM, HOME_Z_MM, HOME_R),
        "vacuum_do": VACUUM_DO, "blow_do": BLOW_DO, "manual_do_max": MANUAL_DO_MAX,
        "speed_pct": DOBOT_SPEED_PCT, "speed_cap_pct": SPEED_HARD_CAP_PCT,
        "z_min": Z_MIN_MM, "z_max": Z_MAX_MM,
        "reach_max": REACH_MAX_MM, "reach_min": REACH_MIN_MM, "y_limit": Y_LIMIT_MM,
        "max_height": MAX_OBJECT_HEIGHT_MM, "min_l": MIN_L_MM, "max_l": MAX_L_MM,
        "max_wait_s": MAX_WAIT_S, "detect_wait_s": DETECT_WAIT_S,
        "last_cycle_file": LAST_CYCLE_FILE,
    }


def load_settings(path=None):
    """dobot_settings.json: {"dobot_live": bool, "dobot_ip": str}. Missing/bad -> safe defaults."""
    path = path or SETTINGS_FILE
    data = {"dobot_live": False, "dobot_ip": DOBOT_IP}
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                data.update(loaded)
        except Exception as e:
            _log("cannot read {}: {} (using safe defaults)".format(path, e))
    return data


def _log(msg):
    print("[DOBOT] {}".format(msg))


# =====================================
# PURE FUNCTIONS: scan gating, plan, validation, command list
# =====================================

def _num(v):
    """float(v) or NaN (never raises)."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def extract_scan(result, cfg=None):
    """
    Pull (height_mm, along_mm, along_status, height_status) out of a detection
    result (detect_result.json, flat keys). Anything missing reads as NaN /
    "unknown", which validate_plan refuses.
    """
    result = result or {}
    return (_num(result.get("object_height_mm")), _num(result.get("along_belt_mm")),
            result.get("along_status", "unknown"), result.get("height_status"))


def plan_pick_place(height_mm, along_mm, cfg, along_status="ok", height_status="ok"):
    """
    Pure: ordered plan of waypoint/action dicts. Never raises (bad inputs are
    reported by validate_plan, which checks the leading "meta" entry).

      pick  : (pick_x, pick_y, pick_z)                        FIXED, never moves
      place : (pick_x, pick_y + L + clearance, pick_z)        downstream of End
    height_mm/along_mm (h/L) are kept only in the "meta" entry for validate_plan
    gating; L also moves the place point. The arm Z is fixed, not height-driven.
    """
    h, L = _num(height_mm), _num(along_mm)
    px = cfg["pick_x"]
    pick_y = cfg["pick_y"] - _num((L / 2) + 10)
    place_y = pick_y + L + cfg["place_clearance"]
    z = min(cfg["pick_z"] + _num(height_mm), 150)
    zt = min(z + cfg["lift"] + _num(height_mm), 150)                     # travel height
    hx, hy, hz, hr = cfg["home"]

    def mv(op, name, x, y, zz):
        return {"op": op, "name": name, "x": x, "y": y, "z": zz, "r": hr}

    return [
        {"op": "meta", "height_mm": h, "along_mm": L, "along_status": along_status,
         "height_status": height_status},
        mv("movl", "home_start", hx, hy, hz),
        mv("movj", "above_pick", px, pick_y, zt),
        mv("movl", "pick_down", px, pick_y, z),
        {"op": "do", "name": "vacuum_on", "index": cfg["vacuum_do"], "value": 1},
        {"op": "wait", "name": "settle", "seconds": cfg["settle_s"]},
        mv("movl", "pick_lift", px, pick_y, zt),
        mv("movl", "carry_forward", px, place_y, zt),
        mv("movl", "place_down", px, place_y, z),
        {"op": "do", "name": "vacuum_off", "index": cfg["vacuum_do"], "value": 0},
        {"op": "do", "name": "blow_on", "index": cfg["blow_do"], "value": 1},
        {"op": "wait", "name": "blow", "seconds": cfg["blow_ms"] / 1000.0},
        {"op": "do", "name": "blow_off", "index": cfg["blow_do"], "value": 0},
        mv("movl", "place_lift", px, place_y, zt),
        mv("movl", "home_end", hx, hy, hz),
    ]


def _finite(*vals):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def validate_plan(plan, cfg):
    """Return list of violation strings; empty list = plan may run."""
    bad = []
    sp = cfg["speed_pct"]
    if not (isinstance(sp, int) and 1 <= sp <= cfg["speed_cap_pct"]):
        bad.append("speed_pct {} outside 1..{} (hard cap)".format(sp, cfg["speed_cap_pct"]))
    if cfg["speed_cap_pct"] > 100:
        bad.append("speed cap above 100")
    if not plan or plan[0].get("op") != "meta":
        return bad + ["plan has no meta entry"]
    meta = plan[0]
    h, L = meta["height_mm"], meta["along_mm"]
    if meta.get("height_status") != "ok":
        bad.append("height_status is {!r}, need 'ok'".format(meta.get("height_status")))
    if meta.get("along_status") != "ok":
        bad.append("along_status is {!r}, need 'ok' (truncated/unknown scans refused)"
                   .format(meta.get("along_status")))
    if not _finite(h) or not (0 < h < cfg["max_height"]):
        bad.append("object height {} not in (0, {})".format(h, cfg["max_height"]))
    if not _finite(L) or not (cfg["min_l"] <= L <= cfg["max_l"]):
        bad.append("along-belt length {} not in [{}, {}]".format(L, cfg["min_l"], cfg["max_l"]))
    for st in plan[1:]:
        op = st.get("op")
        if op in ("movj", "movl"):
            x, y, z = st.get("x"), st.get("y"), st.get("z")
            n = st.get("name")
            if not _finite(x, y, z, st.get("r")):
                bad.append("{}: non-finite coordinate".format(n))
                continue
            if z < cfg["z_min"]:
                bad.append("{}: Z {:.1f} below floor {}".format(n, z, cfg["z_min"]))
            if z > cfg["z_max"]:
                bad.append("{}: Z {:.1f} above ceiling {}".format(n, z, cfg["z_max"]))
            rad = math.hypot(x, y)
            if rad > cfg["reach_max"]:
                bad.append("{}: radius {:.1f} beyond reach {}".format(n, rad, cfg["reach_max"]))
            if rad < cfg["reach_min"]:
                bad.append("{}: radius {:.1f} inside min reach {}".format(n, rad, cfg["reach_min"]))
            if abs(y) > cfg["y_limit"]:
                bad.append("{}: |Y| {:.1f} beyond {}".format(n, abs(y), cfg["y_limit"]))
        elif op == "do":
            if st.get("index") not in (cfg["vacuum_do"], cfg["blow_do"]) \
                    or st.get("value") not in (0, 1):
                bad.append("{}: bad DO command".format(st.get("name")))
        elif op == "wait":
            s = st.get("seconds")
            if not _finite(s) or not (0 <= s <= cfg["max_wait_s"]):
                bad.append("{}: bad wait {}".format(st.get("name"), s))
        else:
            bad.append("unknown op {!r}".format(op))
    return bad


def build_command_list(plan, cfg):
    """
    Pure: the exact command sequence for a validated plan.
    Entries: {"ch": "dashboard"|"motion"|"wait", "cmd": str, ...}.
    SpeedFactor is first. Every motion is followed by Sync().
    """
    cl = [
        {"ch": "dashboard", "cmd": "SpeedFactor({})".format(int(cfg["speed_pct"]))},
        {"ch": "dashboard", "cmd": "EnableRobot()"},
        {"ch": "dashboard", "cmd": "RobotMode()", "expect_mode": cfg["enabled_mode"]},
        {"ch": "dashboard", "cmd": "GetErrorID()", "expect_no_alarm": True},
    ]
    for st in plan:
        op = st["op"]
        if op in ("movj", "movl"):
            cl.append({"ch": "motion", "cmd": "{}({:.2f},{:.2f},{:.2f},{:.2f})".format(
                "MovJ" if op == "movj" else "MovL", st["x"], st["y"], st["z"], st["r"]),
                "name": st["name"]})
            cl.append({"ch": "motion", "cmd": "Sync()", "long": True})
        elif op == "do":
            cl.append({"ch": "dashboard", "cmd": "DO({},{})".format(st["index"], st["value"]),
                       "name": st["name"]})
        elif op == "wait":
            cl.append({"ch": "wait", "cmd": "wait {:.3f}s".format(st["seconds"]),
                       "seconds": st["seconds"]})
    return cl


# =====================================
# CLIENT
# =====================================

class DobotError(Exception):
    """Socket error, timeout, parse error or non-zero ErrorID."""


_REPLY_RE = re.compile(r"^\s*(-?\d+)\s*,\s*\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}\s*,\s*"
                       r"([A-Za-z_]\w*)\s*\((.*)\)\s*$", re.S)


def parse_reply(text):
    """'ErrorID,{a,b},Cmd(args);' -> (error_id:int, values:str, name:str)."""
    m = _REPLY_RE.match(text.strip().rstrip(";"))
    if not m:
        raise DobotError("unparseable reply: {!r}".format(text[:120]))
    return int(m.group(1)), m.group(2), m.group(3)


class DobotClient:
    """Plain-socket client for the dashboard (29999) and motion (30003) ports."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._socks = {}
        self._bufs = {}
        self.sent = []                      # every command text actually sent

    # ---- connection ----
    def connect(self, channels=("dashboard", "motion")):
        """Open the named channels (manual DO / enable need only the dashboard)."""
        for ch, port in (("dashboard", self.cfg["dashboard_port"]),
                         ("motion", self.cfg["motion_port"])):
            if ch not in channels:
                continue
            try:
                s = socket.create_connection((self.cfg["ip"], port),
                                             timeout=self.cfg["connect_timeout_s"])
            except OSError as e:
                self.close()
                raise DobotError("connect {}:{} failed: {}".format(self.cfg["ip"], port, e))
            self._socks[ch] = s
            self._bufs[ch] = b""

    def close(self):
        for s in self._socks.values():
            try:
                s.close()
            except OSError:
                pass
        self._socks, self._bufs = {}, {}

    # ---- raw command ----
    def command(self, ch, text, timeout=None):
        """Send one command, read one reply, check ErrorID. Returns values string."""
        sock = self._socks.get(ch)
        if sock is None:
            raise DobotError("not connected ({})".format(ch))
        timeout = self.cfg["cmd_timeout_s"] if timeout is None else timeout
        try:
            sock.settimeout(timeout)
            self.sent.append(text)
            sock.sendall(text.encode("ascii"))
            deadline = time.monotonic() + timeout
            buf = self._bufs[ch]
            while b";" not in buf:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise socket.timeout()
                sock.settimeout(left)
                chunk = sock.recv(4096)
                if not chunk:
                    raise DobotError("connection closed by robot during {}".format(text))
                buf += chunk
        except socket.timeout:
            raise DobotError("timeout after {}s waiting for reply to {}".format(timeout, text))
        except OSError as e:
            raise DobotError("socket error during {}: {}".format(text, e))
        raw, _, rest = buf.partition(b";")
        self._bufs[ch] = rest
        reply = raw.decode("ascii", errors="replace")
        err, values, name = parse_reply(reply)
        want = text.split("(", 1)[0]
        if name.lower() != want.lower():
            raise DobotError("reply {!r} does not match command {}".format(reply[:80], text))
        if err != 0:
            raise DobotError("ErrorID {} for {} -> {}".format(err, text, reply[:120]))
        return values

    # ---- dashboard (29999) ----
    def enable_robot(self):  return self.command("dashboard", "EnableRobot()")
    def disable_robot(self): return self.command("dashboard", "DisableRobot()")
    def clear_error(self):   return self.command("dashboard", "ClearError()")
    def reset_robot(self):   return self.command("dashboard", "ResetRobot()")
    def robot_mode(self):    return self.command("dashboard", "RobotMode()")
    def get_error_id(self):  return self.command("dashboard", "GetErrorID()")
    def get_pose(self):      return self.command("dashboard", "GetPose()")
    def emergency_stop(self): return self.command("dashboard", "EmergencyStop()")

    def speed_factor(self, pct):
        pct = int(pct)
        if not (1 <= pct <= self.cfg["speed_cap_pct"]):
            raise DobotError("SpeedFactor {} exceeds hard cap {}".format(pct, self.cfg["speed_cap_pct"]))
        return self.command("dashboard", "SpeedFactor({})".format(pct))

    def do(self, index, status):
        return self.command("dashboard", "DO({},{})".format(int(index), int(status)))

    # ---- motion (30003) ----
    def mov_l(self, x, y, z, r):
        return self.command("motion", "MovL({:.2f},{:.2f},{:.2f},{:.2f})".format(x, y, z, r))

    def mov_j(self, x, y, z, r):
        return self.command("motion", "MovJ({:.2f},{:.2f},{:.2f},{:.2f})".format(x, y, z, r))

    def sync(self):
        return self.command("motion", "Sync()", timeout=self.cfg["sync_timeout_s"])


def emergency_stop_now(cfg, client=None):
    """
    Best-effort stop. Outputs are NOT touched. Returns a short text.
    Order: (1) EmergencyStop() on the caller's EXISTING dashboard socket (idle
    while the motion port waits in Sync(); no new connection needed, because it
    is UNVERIFIED that the MG400 accepts a second concurrent dashboard client);
    (2) if that fails, close the old sockets, open a FRESH dashboard connection
    and try EmergencyStop(), then DisableRobot() as last fallback.
    """
    notes = []
    if client is not None and "dashboard" in client._socks:
        try:
            client.emergency_stop()
            return "EmergencyStop() accepted on existing dashboard socket"
        except DobotError as e:
            notes.append("existing socket: {}".format(e))
        client.close()                      # free the controller's connection slot
    c = DobotClient(cfg)
    try:
        s = socket.create_connection((cfg["ip"], cfg["dashboard_port"]),
                                     timeout=cfg["connect_timeout_s"])
        c._socks["dashboard"], c._bufs["dashboard"] = s, b""
    except OSError as e:
        return "stop FAILED ({}; cannot connect: {}) -> use the physical E-stop".format(
            "; ".join(notes), e)
    try:
        try:
            c.emergency_stop()
            return "EmergencyStop() accepted (fresh connection)"
        except DobotError as e1:
            try:
                c.disable_robot()
                return "EmergencyStop failed ({}); DisableRobot() accepted".format(e1)
            except DobotError as e2:
                return "stop FAILED ({}; {}) -> use the physical E-stop".format(e1, e2)
    finally:
        c.close()

# =====================================
# CONTROLLER
# =====================================

class PickPlaceController:
    """
    Runs one pick-and-place cycle at a time.
      live=False (default): dry run, no sockets, nothing sent.
      on_cycle_start / on_cycle_end: callbacks (stop conveyor / re-apply settings).
      on_cycle_end is NOT called when the cycle ended in a fault.
    """

    def __init__(self, cfg=None, live=False, on_cycle_start=None, on_cycle_end=None):
        self.cfg = cfg or default_config()
        self.live = bool(live)
        self.on_cycle_start = on_cycle_start
        self.on_cycle_end = on_cycle_end
        self.fault = False
        self.fault_reason = ""
        self.outputs = {"vacuum": "unknown", "blow": "unknown"}
        self._lock = threading.Lock()
        self._abort = threading.Event()
        self._client = None
        self.last_record = None
        self.mode = "LIVE" if self.live else "DRY RUN"
        # so the web page can show the mode before the first cycle
        self._write_record({"mode": self.mode, "status": "idle", "fault": False,
                            "started": None, "finished": None})

    # ---- state ----
    @property
    def busy(self):
        return self._lock.locked()

    def fault_text(self):
        return "FAULT: {}".format(self.fault_reason) if self.fault else "no fault"

    def clear_fault(self):
        """Interactive only. Live: also best-effort ClearError() on the robot."""
        was = self.fault
        self.fault, self.fault_reason = False, ""
        _log("fault flag cleared by user (was {})".format(was))
        if self.live:
            c = DobotClient(self.cfg)
            try:
                c.connect()
                c.clear_error()
                _log("ClearError() sent. Robot may need re-enable; the next cycle sends EnableRobot().")
            except DobotError as e:
                _log("ClearError failed: {}".format(e))
            finally:
                c.close()

    def _manual(self, fn, *args):
        """Run a manual dashboard action only while no cycle is running (same lock as run_cycle)."""
        if not self._lock.acquire(blocking=False):
            _log("manual command REFUSED: a cycle is running (use dstop to stop it)")
            return False
        try:
            return fn(*args)
        finally:
            self._lock.release()

    def enable(self):
        return self._manual(self._enable)

    def _enable(self):
        """
        Manual 'enable': explicit EnableRobot(), separate from the automatic
        enable a pick cycle does for itself. Dry run: logged only, no socket.
        Refused while a fault is latched (same gate as run_cycle) so an
        operator cannot re-energise the arm around an unresolved fault.
        """
        if self.fault:
            _log("enable REFUSED: {} (use dclear first)".format(self.fault_text()))
            return False
        if not self.live:
            _log("DRY RUN would send [dashboard] EnableRobot()")
            return True
        c = DobotClient(self.cfg)
        try:
            c.connect(("dashboard",))
            c.enable_robot()
            _log("EnableRobot() sent")
            return True
        except DobotError as e:
            _log("enable failed: {}".format(e))
            return False
        finally:
            c.close()

    def disable(self):
        return self._manual(self._disable)

    def _disable(self):
        """
        Manual 'disable': explicit DisableRobot(). Does not touch the fault
        flag or the outputs (the vacuum is never released by software here).
        """
        if not self.live:
            _log("DRY RUN would send [dashboard] DisableRobot()")
            return True
        c = DobotClient(self.cfg)
        try:
            c.connect(("dashboard",))
            c.disable_robot()
            _log("DisableRobot() sent")
            return True
        except DobotError as e:
            _log("disable failed: {}".format(e))
            return False
        finally:
            c.close()

    def reset_alarm(self):
        return self._manual(self._reset_alarm)

    def _reset_alarm(self):
        """
        Manual 'reset alarm'. DISTINCT from clear_fault(): this does not touch
        the software fault latch (clear_fault() is still required, separately,
        before a cycle is allowed to run again - see run_cycle's fault gate).
        It only talks to the robot's own alarm state: ClearError() then, per
        the dobot_api.py protocol notes at the top of this file, ResetRobot()
        (undocumented exact semantics on MG400; both are best-effort and safe
        to call when there is no alarm). Rationale: an operator who only wants
        to clear a Dobot-reported alarm (e.g. after a collision stop) should
        not have to also silently clear this file's own latched safety fault;
        if BOTH need clearing, run dclear as well.
        """
        if not self.live:
            _log("DRY RUN would send [dashboard] ClearError() then ResetRobot()")
            return True
        c = DobotClient(self.cfg)
        try:
            c.connect(("dashboard",))
            c.clear_error()
            c.reset_robot()
            _log("ClearError() and ResetRobot() sent")
            return True
        except DobotError as e:
            _log("reset_alarm failed: {}".format(e))
            return False
        finally:
            c.close()

    def set_speed(self, factor):
        return self._manual(self._set_speed, factor)

    def _set_speed(self, factor):
        """
        Manual 'speed <1..cap>': global speed ratio, SpeedFactor(n) on the
        dashboard port. n must be an integer in 1..speed_cap_pct (the hard cap,
        30 by default; the robot itself accepts 1..100). On success the value
        is also stored in cfg["speed_pct"], because every pick cycle sends
        SpeedFactor(cfg["speed_pct"]) first and would otherwise overwrite this
        change. The new value lasts until the program exits; to keep it, edit
        DOBOT_SPEED_PCT. Moves nothing, so it is allowed during a fault.
        Dry run: logged only, no socket opened.
        """
        cap = self.cfg["speed_cap_pct"]
        if not (_is_int(factor) and 1 <= factor <= cap):
            _log("set_speed REFUSED: {!r} not an integer in 1..{} (hard cap)".format(factor, cap))
            return False
        if not self.live:
            self.cfg["speed_pct"] = factor
            _log("DRY RUN would send [dashboard] SpeedFactor({}); cycle speed now {}%".format(
                factor, factor))
            return True
        c = DobotClient(self.cfg)
        try:
            c.connect(("dashboard",))
            c.speed_factor(factor)
            self.cfg["speed_pct"] = factor
            _log("SpeedFactor({}) sent; cycle speed now {}%".format(factor, factor))
            return True
        except DobotError as e:
            _log("set_speed failed: {}".format(e))
            return False
        finally:
            c.close()

    def go_home(self):
        return self._manual(self._go_home)

    def _go_home(self):
        """
        Manual 'home': move to the HOME pose (cfg["home"], above the belt near
        the End sensor). Same pose the pick cycle starts and ends at.
        Sends SpeedFactor(cfg speed_pct), checks RobotMode, then MovJ + Sync.
        MovJ (joint move) is used because the start pose is unknown; the path
        is an arc, not a straight line, so keep the area clear.
        Refused while a fault is latched (dclear first) and refused if the
        robot is not enabled (use 'enable' first; this never enables for you).
        The pose goes through the same envelope check as a pick plan.
        Dry run: logged only, no socket opened. Use 'dstop' to stop it.
        """
        cfg = self.cfg
        hx, hy, hz, hr = cfg["home"]
        if self.fault:
            _log("home REFUSED: {} (use dclear first)".format(self.fault_text()))
            return False
        # reuse validate_plan's envelope checks; the meta values only satisfy its scan gating
        plan = [{"op": "meta", "height_mm": 1.0, "along_mm": cfg["min_l"],
                 "along_status": "ok", "height_status": "ok"},
                {"op": "movj", "name": "home", "x": hx, "y": hy, "z": hz, "r": hr}]
        bad = validate_plan(plan, cfg)
        if bad:
            _log("home REFUSED: " + "; ".join(bad))
            return False
        move = "MovJ({:.2f},{:.2f},{:.2f},{:.2f})".format(hx, hy, hz, hr)
        if not self.live:
            _log("DRY RUN would send [dashboard] SpeedFactor({}) then [motion] {} and Sync()"
                 .format(int(cfg["speed_pct"]), move))
            return True
        client = DobotClient(cfg)
        self._client = client                 # so dstop can use the open dashboard socket
        try:
            client.connect()
            client.speed_factor(int(cfg["speed_pct"]))
            mode = client.robot_mode().strip().strip("[]").split(",")[0].strip()
            if mode != str(cfg["enabled_mode"]):
                _log("home REFUSED: robot is not enabled (RobotMode {}). Use 'enable' first, "
                     "and 'ralarm' if it reports an alarm".format(mode))
                return False
            _log("moving to HOME X={:.1f} Y={:.1f} Z={:.1f}".format(hx, hy, hz))
            client.mov_j(hx, hy, hz, hr)
            client.sync()
            _log("at HOME")
            return True
        except DobotError as e:
            _log("home failed: {}".format(e))
            return False
        finally:
            self._client = None
            client.close()

    def set_do(self, index, value):
        return self._manual(self._set_do, index, value)

    def _set_do(self, index, value):
        """
        Manual 'do <index> <0|1>' for bench testing, independent of the fixed
        VACUUM_DO/BLOW_DO cycle outputs. Validated the same way validate_plan
        checks a plan's "do" steps: integer index in 1..manual_do_max, value
        0 or 1. Refused while a fault is latched. Dry run: logged only, no
        socket opened (consistent with every other dry-run path in this file).
        """
        if not (_is_int(index) and 1 <= index <= self.cfg["manual_do_max"]):
            _log("set_do REFUSED: index {!r} not an integer in 1..{}".format(
                index, self.cfg["manual_do_max"]))
            return False
        if value not in (0, 1):
            _log("set_do REFUSED: value {!r} not 0 or 1".format(value))
            return False
        if self.fault:
            _log("set_do REFUSED: {} (use dclear first)".format(self.fault_text()))
            return False
        if not self.live:
            _log("DRY RUN would send [dashboard] DO({},{})".format(index, value))
            return True
        c = DobotClient(self.cfg)
        try:
            c.connect(("dashboard",))
            c.do(index, value)
            for key, idx in (("vacuum", self.cfg["vacuum_do"]), ("blow", self.cfg["blow_do"])):
                if index == idx:
                    self.outputs[key] = "on" if value else "off"
            _log("DO({},{}) sent".format(index, value))
            return True
        except DobotError as e:
            _log("set_do failed: {}".format(e))
            return False
        finally:
            c.close()

    def stop_now(self):
        """dstop: stop robot motion immediately and latch the fault flag."""
        self._abort.set()
        self._set_fault("operator dstop")
        if self.live:
            _log("dstop: " + emergency_stop_now(self.cfg, self._client))
        else:
            _log("dstop: dry run, nothing to stop (fault latched anyway)")

    def latch_fault(self, reason):
        """Block all cycles until clear_fault() (used by the auto loop on restart)."""
        self._set_fault(reason)

    def _set_fault(self, reason):
        self.fault, self.fault_reason = True, reason
        print("[DOBOT] !!!!!! FAULT: {} !!!!!! automatic cycles BLOCKED until 'dclear'".format(reason))

    # ---- record ----
    def _write_record(self, rec):
        self.last_record = rec
        path = self.cfg["last_cycle_file"]
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(rec, f, indent=2)
            os.replace(tmp, path)
        except OSError as e:
            _log("could not write {}: {}".format(path, e))

    # ---- main entry ----
    def run_cycle(self, result, source="auto"):
        """Returns the cycle record dict (status: busy/fault_blocked/refused/dry_run_ok/done/fault)."""
        if not self._lock.acquire(blocking=False):
            _log("cycle already running - trigger ignored")
            return {"status": "busy"}
        try:
            if self.fault:
                _log("cycle REFUSED: {} (use dclear after checking the arm)".format(self.fault_text()))
                return {"status": "fault_blocked"}
            self._abort.clear()
            rec = {"mode": self.mode, "source": source, "status": "running",
                   "started": datetime.now().isoformat(timespec="seconds"),
                   "finished": None, "plan": None, "commands": None, "violations": [],
                   "error": None, "fault": False, "outputs": dict(self.outputs)}
            self._write_record(rec)
            started_cb = False
            try:
                if self.on_cycle_start:
                    self.on_cycle_start()
                    started_cb = True
                self._execute(result, rec)
            except Exception as e:                       # last-resort guard
                self._fail(rec, "unexpected {}: {}".format(type(e).__name__, e))
            finally:
                rec["finished"] = datetime.now().isoformat(timespec="seconds")
                rec["fault"] = self.fault
                rec["outputs"] = dict(self.outputs)
                self._write_record(rec)
                if started_cb and self.on_cycle_end:
                    if self.fault:
                        _log("fault: conveyor settings NOT re-applied (belt stays stopped)")
                    else:
                        try:
                            self.on_cycle_end()
                        except Exception as e:
                            _log("on_cycle_end error: {}".format(e))
            return rec
        finally:
            self._lock.release()

    # ---- internals ----
    def _fail(self, rec, reason):
        rec["status"], rec["error"] = "fault", reason
        if self.live:
            _log("stopping robot: " + emergency_stop_now(self.cfg, self._client))
        if self.outputs.get("vacuum") == "on":
            _log("VACUUM LEFT ON (object may be held) - not released by software")
        self._set_fault(reason)

    def _execute(self, result, rec):
        cfg = self.cfg
        h, L, a_status, h_status = extract_scan(result, cfg)
        plan = plan_pick_place(h, L, cfg, a_status, h_status)
        rec["plan"] = plan
        violations = validate_plan(plan, cfg)
        if violations:
            rec["status"], rec["violations"] = "refused", violations
            _log("cycle REFUSED before any motion: " + "; ".join(violations))
            return
        cmds = build_command_list(plan, cfg)
        rec["commands"] = [c["cmd"] for c in cmds]
        _log("{} cycle: height={:.1f} L={:.1f}. Waypoints:".format(self.mode, h, L))
        for st in plan[1:]:
            if st["op"] in ("movj", "movl"):
                _log("  {:5s} {:14s} X={:.1f} Y={:.1f} Z={:.1f}".format(
                    st["op"].upper(), st["name"], st["x"], st["y"], st["z"]))
            else:
                _log("  {}".format(st))
        if not self.live:
            for c in cmds:
                _log("  DRY RUN would send [{}] {}".format(c["ch"], c["cmd"]))
            rec["status"] = "dry_run_ok"
            return

        client = DobotClient(cfg)
        self._client = client
        try:
            client.connect()
            for c in cmds:
                if self._abort.is_set():
                    raise DobotError("aborted by operator")
                self._run_one(client, c)
            rec["status"] = "done"
            self.outputs = {"vacuum": "off", "blow": "off"}
            _log("cycle done, outputs: vacuum OFF, blow OFF")
        except DobotError as e:
            self._fail(rec, str(e))
        finally:
            rec["commands_sent"] = list(client.sent)
            client.close()
            self._client = None

    def _run_one(self, client, c):
        cfg = self.cfg
        if c["ch"] == "wait":
            if self._abort.wait(c["seconds"]):
                raise DobotError("aborted by operator")
            return
        _log("send [{}] {}".format(c["ch"], c["cmd"]))
        name = c["cmd"]
        if "expect_mode" in c:
            deadline = time.monotonic() + cfg["enable_wait_s"]
            while True:
                vals = client.command(c["ch"], c["cmd"])
                if vals.strip().strip("[]").split(",")[0].strip() == str(c["expect_mode"]):
                    return
                if time.monotonic() > deadline:
                    raise DobotError("robot not enabled (RobotMode {}) after {}s"
                                     .format(vals, cfg["enable_wait_s"]))
                time.sleep(0.3)
        vals = client.command(c["ch"], c["cmd"], cfg["sync_timeout_s"] if c.get("long") else None)
        if c.get("expect_no_alarm") and re.search(r"\d", vals):
            raise DobotError("robot reports active alarm(s): {}".format(vals))
        if name.startswith("DO("):
            on = name.endswith(",1)")
            idx = name[3:-1].split(",")[0]
            # only the two cycle outputs are tracked; other manual DOs are not
            key = {str(cfg["vacuum_do"]): "vacuum", str(cfg["blow_do"]): "blow"}.get(idx)
            if key:
                self.outputs[key] = "on" if on else "off"


# =====================================
# AUTO MODE (STATE_CONTRACT.md section 5)
# =====================================

Decision = collections.namedtuple("Decision", "action outcome reason")
_WAIT = Decision("wait", None, None)


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def decide(conveyor, conveyor_fresh, detect, handled_id, state_busy, waited_s,
           detect_wait_s=DETECT_WAIT_S, last_detect_id=0):
    """
    Pure. Returns Decision("wait" | "pick" | "skip", outcome, reason).

    A detection result is used only if its object_id equals the object waiting
    at End, it came from auto mode, and its detect_id is newer than the last
    one used. A result is never judged by its time stamp.
    Validation of height / length / envelope (skipped_rejected) happens when
    the cycle is planned (PickPlaceController.run_cycle), not here.
    """
    if not conveyor_fresh or not isinstance(conveyor, dict):
        return _WAIT
    oid = conveyor.get("object_id")
    if (state_busy or conveyor.get("phase") != "at_end"
            or not conveyor.get("pick_permission")
            or not _is_int(oid) or oid <= handled_id):
        return _WAIT
    did = detect.get("detect_id") if isinstance(detect, dict) else None
    matches = (isinstance(detect, dict) and detect.get("object_id") == oid
               and detect.get("source") == "auto"
               and _is_int(did) and did > last_detect_id)
    if not matches:
        if waited_s >= detect_wait_s:
            return Decision("skip", "skipped_no_detection", "no_matching_detection")
        return _WAIT
    if detect.get("status") != "ok":
        return Decision("skip", "skipped_no_detection", str(detect.get("status")))
    return Decision("pick", None, None)


class StateReaders:
    """Read-only access to the other programs' state files."""

    def __init__(self, state_dir, clock=time.monotonic):
        self.dir = state_dir
        self._conv_fresh = state_io.Freshness(1.0, clock=clock)

    def _path(self, name):
        return os.path.join(self.dir, name)

    def conveyor(self):
        doc = state_io.read_json(self._path("conveyor_status.json"), "conveyor_status/1")
        self._conv_fresh.update(doc)
        return doc, self._conv_fresh.is_fresh()

    def detect(self):
        return state_io.read_json(self._path("detect_result.json"), "detect_result/1")

    def old_status(self):
        return state_io.read_json(self._path("dobot_status.json"), "dobot_status/1")

    def clear_flag(self):
        """True (and the flag is removed) when `dobot.py --clear-fault` was run."""
        path = self._path("clear_dobot.flag")
        if not os.path.exists(path):
            return False
        try:
            os.remove(path)
        except OSError:
            pass
        return True


class AutoLoop:
    """
    Waits for the conveyor to hold an object at End with pick permission, picks
    it once, and reports idle / busy / done / fault in dobot_status.json.
    handled_id is a high-water mark: an object id <= handled_id is never picked.
    Does no detection thinking: it only checks the result matches the object.
    """

    def __init__(self, controller, readers, writer, clock=time.monotonic,
                 detect_wait_s=DETECT_WAIT_S):
        self.controller = controller
        self.readers = readers
        self.writer = writer
        self.clock = clock
        self.detect_wait_s = detect_wait_s
        self.state = "idle"
        self.current_id = None
        self.handled_id = 0
        self.outcome = None
        self.reason = None
        self.detect_id_used = None
        self.last_detect_id = 0
        self.started_ts = None
        self.finished_ts = None
        self._worker = None
        self._result = None
        self._wait_oid = None
        self._wait_since = None
        self._dirty = True
        self._last_write = -1e9
        self._restart_checked = False

        old = readers.old_status()
        if old is not None:
            self._restart_checked = True
            hid = old.get("handled_id")
            self.handled_id = hid if _is_int(hid) else 0
            used = old.get("detect_id_used")
            self.last_detect_id = used if _is_int(used) else 0
            if old.get("state") == "busy":
                cur = old.get("current_id")
                if _is_int(cur):
                    self.handled_id = max(self.handled_id, cur)
                self._enter_fault("restart_after_busy")
            elif old.get("state") == "fault":
                self._enter_fault("restart_after_fault")

    # ---- state changes ----
    def _enter_fault(self, reason):
        self.state, self.outcome, self.reason = "fault", "failed", reason
        self.current_id = None
        self.controller.latch_fault(reason)
        self._dirty = True

    def _finalize(self, oid, outcome, reason, state="done"):
        self.handled_id = max(self.handled_id, oid)
        self.current_id = None
        self.state, self.outcome, self.reason = state, outcome, reason
        self.finished_ts = time.time()
        if _is_int(self.detect_id_used):
            self.last_detect_id = max(self.last_detect_id, self.detect_id_used)
        self._dirty = True
        self._write(force=True)

    def _body(self):
        return {
            "mode": "auto", "live": bool(self.controller.live),
            "state": self.state, "current_id": self.current_id, "handled_id": self.handled_id,
            "outcome": self.outcome, "reason": self.reason,
            "detect_id_used": self.detect_id_used,
            "started_ts": self.started_ts, "finished_ts": self.finished_ts,
            "fault": self.reason if self.state == "fault" else None,
        }

    def _write(self, force=False):
        now = self.clock()
        if not (force or self._dirty or now - self._last_write >= HEARTBEAT_S):
            return
        try:
            self.writer.write(self._body())
            self._dirty = False
            self._last_write = now
        except OSError as e:
            self._dirty = True
            _log("dobot_status write failed (will retry): {}".format(e))

    # ---- one pass ----
    def step(self):
        if self._worker is not None:
            if not self._worker.is_alive():
                self._end_cycle()
            else:
                self._write()
            return
        if self.state == "fault":
            if self.readers.clear_flag():
                self.controller.clear_fault()
                self.state, self.outcome, self.reason = "idle", None, None
                self._dirty = True
                _log("fault cleared by operator")
            self._write()
            return

        conv, fresh = self.readers.conveyor()
        det = self.readers.detect()
        oid = conv.get("object_id") if isinstance(conv, dict) else None

        if not self._restart_checked and fresh:
            self._restart_checked = True
            if isinstance(conv, dict) and conv.get("phase") == "at_end" and _is_int(oid):
                _log("restart with an object at End and no old status: not picking {}".format(oid))
                self._finalize(oid, "skipped_rejected", "unknown_restart")
                return

        now = self.clock()
        waiting = (fresh and isinstance(conv, dict) and conv.get("phase") == "at_end"
                   and conv.get("pick_permission") and _is_int(oid) and oid > self.handled_id)
        if waiting:
            if self._wait_oid != oid:
                self._wait_oid, self._wait_since = oid, now
            waited = now - self._wait_since
        else:
            self._wait_oid, waited = None, 0.0

        d = decide(conv, fresh, det, self.handled_id, False, waited,
                   self.detect_wait_s, self.last_detect_id)
        if d.action == "wait":
            self._write()
        elif d.action == "skip":
            self.detect_id_used = None
            _log("object {}: skip ({}, {})".format(oid, d.outcome, d.reason))
            self._finalize(oid, d.outcome, d.reason)
        else:
            self._start_cycle(oid, det)

    def _start_cycle(self, oid, det):
        self.state, self.current_id = "busy", oid
        self.outcome = self.reason = None
        self.detect_id_used = det.get("detect_id")
        self.started_ts, self.finished_ts = time.time(), None
        self._dirty = True
        self._write(force=True)             # busy is visible BEFORE any motion
        self._result = None

        def work():
            try:
                self._result = self.controller.run_cycle(det, "auto")
            except Exception as e:           # run_cycle guards itself; belt and braces
                self._result = {"status": "fault", "error": "{}: {}".format(type(e).__name__, e)}

        self._worker = threading.Thread(target=work, daemon=True, name="dobot-cycle")
        self._worker.start()

    def _end_cycle(self):
        self._worker = None
        rec = self._result or {"status": "fault", "error": "no result"}
        oid, status = self.current_id, rec.get("status")
        if status == "dry_run_ok":
            self._finalize(oid, "dry_run", None)
        elif status == "done":
            self._finalize(oid, "picked_placed", None)
        elif status == "refused":
            self._finalize(oid, "skipped_rejected", "; ".join(rec.get("violations") or [])[:200])
        else:
            self._finalize(oid, "failed", str(rec.get("error") or status)[:200], state="fault")

    def run(self, stop=None, sleep=time.sleep):
        self._write(force=True)
        while stop is None or not stop.is_set():
            self.step()
            sleep(POLL_S)


# =====================================
# MANUAL MODE + MAIN
# =====================================

MENU = """
Commands:
  pick <height_mm> [length_mm]   Run one cycle at the fixed point (dry run unless --live)
  home                           Move to the HOME pose (robot must be enabled; refused on a fault)
  show                           Show mode, fault and outputs
  dclear                         Clear fault (only after checking the arm / vacuum)
  dstop                          Stop Dobot motion immediately (latches fault)
  enable                         Send EnableRobot() (refused while a fault is latched)
  disable                        Send DisableRobot()
  ralarm                         Reset the ROBOT alarm: ClearError() + ResetRobot().
                                 Does NOT clear the software fault (use dclear for that)
  do <index> <0|1>               Set digital output <index> (1..16) off/on, for testing
  speed <percent>                Set SpeedFactor (1..30, hard cap); used by later cycles too
  q                              Quit
Manual commands are refused while a cycle is running. In dry run they only log
what would be sent. Vacuum/blow are never released automatically on a fault:
use 'do 1 0' yourself once the part is safe.
"""


def manual_result(height, length):
    return {"object_height_mm": height, "height_status": "ok",
            "along_belt_mm": length, "along_status": "ok"}


def interactive(ctl):
    print(MENU)
    while True:
        try:
            parts = input("> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not parts:
            continue
        cmd = parts[0].lower()
        if cmd == "q":
            break
        elif cmd == "pick":
            try:
                h = float(parts[1])
                length = float(parts[2]) if len(parts) > 2 else 100.0
            except (IndexError, ValueError):
                print("Usage: pick <height_mm> [length_mm]")
                continue
            threading.Thread(target=ctl.run_cycle, args=(manual_result(h, length), "manual"),
                             daemon=True, name="dobot-cycle").start()
        elif cmd == "home":
            ctl.go_home()
        elif cmd == "show":
            print("[DOBOT] mode={} busy={} {} outputs={}".format(
                ctl.mode, ctl.busy, ctl.fault_text(), ctl.outputs))
        elif cmd == "dclear":
            ctl.clear_fault()
        elif cmd == "dstop":
            ctl.stop_now()
        elif cmd == "enable":
            ctl.enable()
        elif cmd == "disable":
            ctl.disable()
        elif cmd == "ralarm":
            ctl.reset_alarm()
        elif cmd == "do":
            try:
                idx, val = int(parts[1]), int(parts[2])
            except (IndexError, ValueError):
                print("Usage: do <index> <0|1>")
                continue
            ctl.set_do(idx, val)
        elif cmd == "speed":
            try:
                ctl.set_speed(int(parts[1]))
            except (IndexError, ValueError):
                print("Usage: speed <percent 1..{}>".format(ctl.cfg["speed_cap_pct"]))
        else:
            print("Unknown command.")


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Dobot MG400 pick and place (fixed point)")
    parser.add_argument("--auto", action="store_true", help="Auto mode: follow the conveyor status file")
    parser.add_argument("--live", action="store_true",
                        help="Send REAL motion/DO commands (default: dry run). "
                             "Also needs dobot_settings.json dobot_live=true")
    parser.add_argument("--once", action="store_true", help="One cycle with --height, then exit")
    parser.add_argument("--height", type=float, default=None, help="Object height mm (--once)")
    parser.add_argument("--length", type=float, default=100.0, help="Along-belt length mm (--once)")
    parser.add_argument("--ip", default=None, help="Dobot IP (default from dobot_settings.json)")
    parser.add_argument("--state-dir", default=None, help="Shared state folder")
    parser.add_argument("--clear-fault", action="store_true",
                        help="Create state/clear_dobot.flag and exit")
    args = parser.parse_args(argv)

    state_dir = state_io.resolve_state_dir(args.state_dir)
    if args.clear_fault:
        flag = os.path.join(state_dir, "clear_dobot.flag")
        open(flag, "w").close()
        print("[CLEAR] wrote {}".format(flag))
        return 0

    settings = load_settings()
    live = bool(args.live)
    if live and not settings.get("dobot_live", False):
        _log("--live given but dobot_settings.json dobot_live is false -> staying in DRY RUN")
        live = False
    cfg = default_config()
    cfg["ip"] = args.ip or settings.get("dobot_ip", DOBOT_IP)
    cfg["last_cycle_file"] = os.path.join(state_dir, "dobot_last_cycle.json")
    ctl = PickPlaceController(cfg, live=live)
    _log("mode={} ip={}".format(ctl.mode, cfg["ip"]))

    if args.once:
        if args.height is None:
            parser.error("--once needs --height")
        rec = ctl.run_cycle(manual_result(args.height, args.length), "manual")
        _log("result: {}".format(rec.get("status")))
        return 0 if rec.get("status") in ("dry_run_ok", "done") else 1

    if not args.auto:
        interactive(ctl)
        return 0

    lock = state_io.acquire_single_instance("dobot", state_dir)
    if lock is None:
        print("[ERROR] dobot.py --auto is already running (lock held)")
        return 1
    writer = state_io.StateWriter("dobot_status", "dobot.py", state_dir, durable=True)
    loop = AutoLoop(ctl, StateReaders(state_dir), writer, detect_wait_s=cfg["detect_wait_s"])
    try:
        import signal
        signal.signal(signal.SIGTERM, _raise_interrupt)
    except (ImportError, ValueError):
        pass
    _log("auto mode, state dir {}".format(state_dir))
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        _log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())