"""
state_io.py
Small stdlib-only helper for the shared JSON state files (see STATE_CONTRACT.md).

One writer per file, atomic replace, readers get None for anything unusable.
"""

import json
import os
import time
import uuid

DEFAULT_STATE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "state")


def resolve_state_dir(arg=None):
    """--state-dir beats env CELL_STATE_DIR beats <Project>/state. Absolute, created."""
    path = arg or os.environ.get("CELL_STATE_DIR") or DEFAULT_STATE_DIR
    path = os.path.abspath(path)
    os.makedirs(path, exist_ok=True)
    return path


def write_json_atomic(path, obj, durable=False):
    """Write obj as JSON to path atomically. Retries PermissionError 3 times (Windows)."""
    tmp = "{}.tmp.{}".format(path, os.getpid())
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f)
        f.flush()
        if durable:
            os.fsync(f.fileno())
    for attempt in range(3):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            if attempt == 2:
                try:
                    os.remove(tmp)
                except OSError:
                    pass
                raise
            time.sleep(0.01)


def read_json(path, schema):
    """Return the dict, or None if missing/empty/corrupt/wrong schema major version."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(doc, dict):
        return None
    got = doc.get("schema")
    if not isinstance(got, str) or got.split("/")[0] != schema.split("/")[0]:
        return None
    if got.split("/")[-1] != schema.split("/")[-1]:
        return None
    return doc


class Freshness:
    """A doc is fresh while a new (boot_id, seq) was seen within max_age_s (monotonic)."""

    def __init__(self, max_age_s, clock=time.monotonic):
        self.max_age_s = max_age_s
        self.clock = clock
        self._key = None
        self._seen = None

    def update(self, doc):
        """Feed the latest read. Returns True if it was a new (boot_id, seq)."""
        if not isinstance(doc, dict):
            return False
        key = (doc.get("boot_id"), doc.get("seq"))
        if key == self._key:
            return False
        self._key = key
        self._seen = self.clock()
        return True

    def is_fresh(self):
        return self._seen is not None and (self.clock() - self._seen) <= self.max_age_s


class StateWriter:
    """Adds the common header and bumps seq on every write."""

    def __init__(self, name, writer, state_dir, durable=False):
        self.name = name
        self.writer = writer
        self.path = os.path.join(state_dir, name + ".json")
        self.durable = durable
        self.boot_id = uuid.uuid4().hex[:8]
        self.seq = 0

    def write(self, body):
        self.seq += 1
        doc = {
            "schema": "{}/1".format(self.name),
            "writer": self.writer,
            "boot_id": self.boot_id,
            "seq": self.seq,
            "ts": time.time(),
        }
        doc.update(body)
        write_json_atomic(self.path, doc, durable=self.durable)
        return doc


def acquire_single_instance(name, state_dir):
    """Take an OS lock on <state_dir>/<name>.lock. Returns the open file (keep it
    referenced) or None if another process holds it."""
    f = open(os.path.join(state_dir, name + ".lock"), "a+")
    try:
        if os.name == "nt":
            import msvcrt
            f.seek(0)
            msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return None
    return f
