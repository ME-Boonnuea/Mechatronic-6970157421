"""
piSerial.py
Launcher and supervisor for the cell. It ONLY starts the other programs and
restarts them when they crash. It has no sequencing logic, reads no state
files, imports none of the other programs and does not open the serial port
(conveyor.py does that).

Starts, in this order:   conveyor.py --auto, camera.py --auto, dobot.py --auto
Each gets   --state-dir <absolute folder>   so they share the same state files.

Usage:
    python3 piSerial.py
    python3 piSerial.py --only conveyor,camera       # skip the robot
    python3 piSerial.py --dobot-live                  # also needs dobot_settings.json dobot_live=true
    python3 piSerial.py --state-dir /dev/shm/cell_state

Restart: a child that exits is started again after 1, 2, 4 ... up to 30 s
(back to 1 s after a run of 60 s or more). Never gives up.
Logs: logs/piSerial.log, and each child's output in logs/<name>.log (1 MB x 3).
Stop: Ctrl+C or SIGTERM -> dobot, camera, conveyor are terminated (5 s, then killed).
"""

import argparse
import logging
import logging.handlers
import os
import signal
import subprocess
import sys
import threading
import time

import state_io

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")

START_ORDER = ("conveyor", "camera", "dobot")
BACKOFF_START_S = 1
BACKOFF_MAX_S = 30
BACKOFF_RESET_AFTER_S = 60
STOP_WAIT_S = 5.0


def build_argv(name, state_dir, dobot_live=False, python=None):
    """Command line of one child."""
    argv = [python or sys.executable, os.path.join(BASE_DIR, name + ".py"),
            "--auto", "--state-dir", state_dir]
    if name == "dobot" and dobot_live:
        argv.append("--live")
    return argv


class Supervised:
    """One child process with restart and backoff. `popen` and `clock` are
    injectable so the restart logic is testable without real processes."""

    def __init__(self, name, argv, popen=subprocess.Popen, clock=time.monotonic,
                 log=print, on_start=None):
        self.name = name
        self.argv = argv
        self.popen = popen
        self.clock = clock
        self.log = log
        self.on_start = on_start          # called with (name, proc) after each start
        self.proc = None
        self.started_at = None
        self.next_start = 0.0
        self.backoff = BACKOFF_START_S
        self.starts = 0

    def poll(self):
        """Start the child if it is due, or notice that it exited. Call often."""
        now = self.clock()
        if self.proc is None:
            if now >= self.next_start:
                self._start(now)
            return
        rc = self.proc.poll()
        if rc is None:
            return
        if now - self.started_at >= BACKOFF_RESET_AFTER_S:
            self.backoff = BACKOFF_START_S
        delay = self.backoff
        self.backoff = min(self.backoff * 2, BACKOFF_MAX_S)
        self.log("{} exited with code {} after {:.1f}s, restarting in {}s".format(
            self.name, rc, now - self.started_at, delay))
        self.proc = None
        self.next_start = now + delay

    def _start(self, now):
        try:
            self.proc = self.popen(self.argv)
        except OSError as e:
            delay = self.backoff
            self.backoff = min(self.backoff * 2, BACKOFF_MAX_S)
            self.next_start = now + delay
            self.log("{} could not start: {} (retry in {}s)".format(self.name, e, delay))
            return
        self.started_at = now
        self.starts += 1
        self.log("{} started (pid {}, start #{})".format(
            self.name, getattr(self.proc, "pid", "?"), self.starts))
        if self.on_start:
            self.on_start(self.name, self.proc)

    def terminate(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()

    def kill(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()

    def alive(self):
        return self.proc is not None and self.proc.poll() is None


def stop_all(children, wait_s=STOP_WAIT_S, clock=time.monotonic, sleep=time.sleep, log=print):
    """Terminate in the given order, wait wait_s in total, then kill the rest."""
    for c in children:
        c.terminate()
    deadline = clock() + wait_s
    while clock() < deadline and any(c.alive() for c in children):
        sleep(0.1)
    for c in children:
        if c.alive():
            log("{} did not stop in {}s, killing".format(c.name, wait_s))
            c.kill()


# =====================================
# LOGGING
# =====================================

def make_logger(name, console=False):
    os.makedirs(LOG_DIR, exist_ok=True)
    lg = logging.getLogger("cell." + name)
    lg.setLevel(logging.INFO)
    lg.propagate = False
    if not lg.handlers:
        fh = logging.handlers.RotatingFileHandler(
            os.path.join(LOG_DIR, name + ".log"), maxBytes=1_000_000, backupCount=3,
            encoding="utf-8")
        fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
        lg.addHandler(fh)
        if console:
            ch = logging.StreamHandler()
            ch.setFormatter(logging.Formatter("[piSerial] %(message)s"))
            lg.addHandler(ch)
    return lg


def pump_output(name, proc):
    """Copy the child's stdout/stderr into logs/<name>.log (own thread per child)."""
    stream = getattr(proc, "stdout", None)
    if stream is None:
        return
    lg = make_logger(name)

    def run():
        for line in stream:
            lg.info(line.rstrip())

    threading.Thread(target=run, daemon=True, name="log-" + name).start()


def piped_popen(argv):
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    return subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace", bufsize=1, env=env)


# =====================================
# MAIN
# =====================================

def main(argv=None):
    parser = argparse.ArgumentParser(description="Start and supervise conveyor, camera, dobot")
    parser.add_argument("--only", default=None,
                        help="Comma list of programs to run (default: {})".format(",".join(START_ORDER)))
    parser.add_argument("--dobot-live", action="store_true",
                        help="Pass --live to dobot.py (REAL robot motion; also needs dobot_settings.json dobot_live=true)")
    parser.add_argument("--state-dir", default=None, help="Shared state folder for all programs")
    args = parser.parse_args(argv)

    names = list(START_ORDER)
    if args.only:
        wanted = [n.strip() for n in args.only.split(",") if n.strip()]
        bad = [n for n in wanted if n not in START_ORDER]
        if bad:
            parser.error("unknown program(s): {} (choose from {})".format(
                ", ".join(bad), ", ".join(START_ORDER)))
        names = [n for n in START_ORDER if n in wanted]

    state_dir = state_io.resolve_state_dir(args.state_dir)
    log = make_logger("piSerial", console=True).info
    log("state dir {}".format(state_dir))
    if args.dobot_live:
        log("WARNING: --dobot-live given; dobot.py will move the real robot "
            "if dobot_settings.json has dobot_live=true")

    children = [Supervised(n, build_argv(n, state_dir, args.dobot_live), popen=piped_popen,
                           log=log, on_start=pump_output) for n in names]

    stop = threading.Event()

    def on_signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    try:
        signal.signal(signal.SIGTERM, on_signal)
    except (ValueError, AttributeError):
        pass

    try:
        while not stop.is_set():
            for c in children:
                c.poll()
            stop.wait(0.5)
    finally:
        log("stopping ...")
        stop_all(list(reversed(children)), log=log)
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
