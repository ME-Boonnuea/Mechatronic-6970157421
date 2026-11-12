# STATE CONTRACT (v1)

Design for the split into independent programs. Every program runs ALONE in manual mode and cooperates in auto mode only through the small JSON files below. This file is the single source of truth for the implementers.

Programs: `firmware_conveyor.py` (Motion 2350 Pro), `conveyor.py` (Pi), `camera.py` (+ `web_stream.py` as a library inside the camera process), `detector.py` (pure library), `dobot.py`, `piSerial.py` (launcher only), and a new stdlib-only helper `state_io.py`.

Tech stack (locked): Python 3 on the Pi and the PC, MicroPython on the board, pyserial, Flask, OpenCV, numpy, pyrealsense2 (camera process only), stdlib `json`/`socket`/`subprocess`/`unittest`.

## 1. Decisions on the open points

| Point | Decision |
|---|---|
| Firmware file | Repo name `firmware_conveyor.py`. It is copied to the board as `main.py` (the board runs `main.py` at power-up). The name `conveyor.py` is freed for the Pi program. |
| Deploy | Step 0 of the refactor: copy the current MicroPython `conveyor.py` to `firmware_conveyor.py` before the new Pi `conveyor.py` overwrites it. Then: `mpremote ls` (see what is on the board), `mpremote cp firmware_conveyor.py :main.py`, `mpremote rm :conveyor.py` only if it is listed and nothing imports it, then `mpremote reset`. Thonny works as well: save the file to the device as `main.py`. After reset the board prints `BOOT fw=2`. |
| Dobot go-ahead | A STATUS FILE, not piSerial. `conveyor_status.json` carries `pick_permission` and `object_id`. piSerial holds no sequencing and never reads state files. |
| piSerial.py | Launch and supervise only (section 8). It does not import conveyor, camera, dobot or detector. It does not open the serial port. |
| scanner.py | RETIRED and deleted at the end of the migration. Its frame loop and running-best logic move to `camera.py`; the pure parts go to `detector.py`; its constants go to `camera_settings.json`. The time-based width (`width_time_based_mm`) is dropped, because the object is stationary at End. |
| conveyor_only.py | RETIRED and deleted. Its serial, settings and menu code is merged into the new `conveyor.py` (manual mode). No shim is left behind. |
| Mode switching | By start-up flag only: no flag = manual, `--auto` = auto. There is no mode or command file. The only in-process exception is the web page's camera auto/manual toggle. |
| Operator commands between programs | None, except two "touch" files used to clear a latched fault (section 4.5). |
| State folder | `<Project>/state/`. Override: `--state-dir` beats env `CELL_STATE_DIR` beats the default. It is converted to an absolute path once at start-up. On the Pi, setting `CELL_STATE_DIR=/dev/shm/cell_state` is recommended, because heartbeats write about 10 times per second and would wear the SD card. piSerial resolves the path once and passes `--state-dir <abs>` to every child. |
| Settings vs state | Settings (hand-edited config) stay in the Project folder: `conveyor_settings.json`, `camera_settings.json`, `dobot_settings.json`. State (runtime) lives in the state folder. `dobot_live` moves out of `conveyor_settings.json` into `dobot_settings.json`. Unknown keys in settings files are preserved on save. |

## 2. Rules for all state files

- One writer per file. Readers never write.
- Atomic write: write `<name>.tmp.<pid>` in the same folder, flush, `os.replace` onto the target. Use `fsync` before the replace for `detect_result.json` and `dobot_status.json`. On `PermissionError` (Windows), retry 3 times with 10 ms between tries.
- All files are UTF-8 JSON objects with this common header:

| Key | Type | Meaning |
|---|---|---|
| `schema` | str | `"<file>/1"`. A reader rejects a different major version. |
| `writer` | str | program file name |
| `boot_id` | str | random 8 hex characters, new on every writer start |
| `seq` | int | +1 on every write of this file, restarts at 1 per boot |
| `ts` | float | `time.time()`, epoch seconds. For logs and the UI only. |

- Heartbeat: status files are rewritten at least every 200 ms (conveyor) or 500 ms (camera, dobot), even when nothing changed. A changing `seq` is the proof that the writer is alive.
- Staleness is judged by the reader, not by comparing wall clocks. The Pi may have no RTC and NTP can jump. `state_io.Freshness` remembers the monotonic time at which it last saw a new `(boot_id, seq)`. The file is stale if nothing changed for longer than the threshold.

| File | Stale after (no new seq) |
|---|---|
| `conveyor_status.json` | 1.0 s |
| `camera_status.json` | 3.0 s |
| `dobot_status.json` | 3.0 s |
| `detect_result.json` | never stale by time. It is valid only through the id match in section 5. |

- A missing, empty, corrupt, wrong-schema or wrong-type file reads as `None` ("unknown"). Readers treat unknown as "do not act". They log at WARNING once per change of condition, not once per poll.
- Ids are monotonic integers and never reused.
  - `object_id` is allocated only by `conveyor.py`: `max(previous last_object_id + 1, int(time.time()))`. The previous value is read from its own old file at start. The epoch seed keeps ids rising even if the state folder was wiped.
  - `detect_id` is allocated only by `camera.py`, with the same rule.
- `state_io.py` API: `resolve_state_dir(arg)`, `write_json_atomic(path, obj, durable=False)`, `read_json(path, schema)`, `class Freshness(max_age_s, clock=time.monotonic)` with `update(doc)->bool` and `is_fresh()`, `class StateWriter(name, writer, state_dir)` with `write(body)` (adds the header, bumps `seq`), `acquire_single_instance(name, state_dir)` (OS file lock `state/<name>.lock`, `fcntl.flock` on Linux and `msvcrt.locking` on Windows, released automatically on a crash).

## 3. State folder layout

```
state/
  conveyor_status.json    writer conveyor.py
  camera_status.json      writer camera.py
  detect_result.json      writer camera.py
  dobot_status.json       writer dobot.py
  dobot_last_cycle.json   writer dobot.py   (was Project/dobot_last_cycle.json)
  captures/obj_<id>.jpg   writer camera.py  (best crop, keep newest 50)
  clear_conveyor.flag     writer: operator CLI (section 4.5)
  clear_dobot.flag        writer: operator CLI
  <name>.lock             lock files
logs/<name>.log           writer piSerial.py (child stdout and stderr)
```

## 4. Schemas

Body fields are shown after the common header. A key marked `?` may be `null`.

### 4.1 `conveyor_status.json` (writer `conveyor.py`)

```json
{
  "schema": "conveyor_status/1", "writer": "conveyor.py", "boot_id": "a1b2c3d4", "seq": 4121, "ts": 1760000000.12,
  "mode": "auto",
  "link": {"connected": true, "port": "/dev/ttyACM0", "fw_fresh": true},
  "sensors": {"start": false, "end": true, "raw_s1": 0, "raw_s2": 1, "valid": true},
  "motor": {"running": false, "speed": 0, "direction": "FORWARD", "fw_speed": 0, "encoder": 1234, "watchdog": false},
  "phase": "at_end",
  "object_id": 1760000011,
  "last_object_id": 1760000011,
  "object_at_end": true,
  "pick_permission": true,
  "cycle": {"started_ts": 1760000011.0, "end_ts": 1760000019.2, "stopped_ts": 1760000019.3, "released_ts": null},
  "hold_reason": null,
  "fault": null
}
```

- `mode`: `manual` or `auto`.
- `phase`:
  - `manual`: manual mode. There is never a pick permission and no cycle.
  - `idle`: auto, armed, motor stopped, waiting for Start. `object_id` is null.
  - `running`: an object was seen at Start and the belt is running toward End. `object_id` is set.
  - `at_end`: End seen, motor stopped, object waiting for the robot. `object_at_end` is true.
  - `blocked`: auto, End sensor is still active when a new cycle would arm. The conveyor does not run.
  - `fault`: see `fault`.
- `sensors.start` and `sensors.end` are logical (raw value equals `sensor_active_level`). They are debounced (Start 0.2 s, End 0.05 s). `valid` is false when the firmware link has no fresh data. While invalid the machine does not advance.
- `motor.speed` and `motor.direction` are what the Pi commanded. `fw_speed` is the last `SPD=` reported by the board.
- `pick_permission` is true only when ALL of these hold: `phase=="at_end"`, `link.connected`, `link.fw_fresh`, commanded speed is 0, reported `fw_speed` is 0, and at least `stop_settle_s` (0.5) has passed since `stopped_ts`. It is false everywhere else.
- `hold_reason` (while `at_end`): null, `"waiting_dobot"`, `"dobot_fault"` or `"dobot_stale"`.
- `fault` is one of `"serial_lost"`, `"end_timeout"`, `"fw_watchdog"`, `"state_write_failed"`, or null. A fault latches until `clear_conveyor.flag` or a restart.

### 4.2 `camera_status.json` (writer `camera.py`)

```json
{ "...header": "...", "mode": "auto",
  "camera": {"state": "on", "fps": 29.8, "error": null},
  "tracking": {"state": "detecting", "object_id": 1760000011, "frames_seen": 61, "frames_with_object": 18,
               "best_area_px": 15234, "best_height_mm": 41.2},
  "last_result": {"detect_id": 1760000007, "object_id": 1760000004, "status": "ok"},
  "web": {"port": 5000} }
```

- `camera.state`: `off`, `starting`, `on`, `error`.
- `tracking.state`: `idle`, `detecting` (auto, `conveyor.phase=="running"`), `frozen` (result written, waiting for the next cycle), `manual`.

### 4.3 `detect_result.json` (writer `camera.py`)

The flat `object_height_mm`, `along_belt_mm`, `along_status` and `height_status` keys keep the names `dobot.extract_scan` already reads.

```json
{
  "schema": "detect_result/1", "writer": "camera.py", "boot_id": "e5f6a7b8", "seq": 12, "ts": 1760000019.4,
  "detect_id": 1760000020,
  "source": "auto",
  "object_id": 1760000011,
  "status": "ok",
  "reason": null,
  "object_height_mm": 41.2, "height_status": "ok",
  "along_belt_mm": 123.4, "along_status": "ok",
  "best": {
    "width_mm": 80.1, "length_mm": 120.0, "rotated_long_mm": 123.4, "rotated_short_mm": 79.0,
    "height_mm": 41.2, "median_depth_mm": 383.8, "bbox_px": [210, 120, 180, 150],
    "area_px": 15234.0, "depth_samples": 14800, "touches_border": false, "frame_ts": 1760000015.2
  },
  "stats": {"frames_seen": 61, "frames_with_object": 18, "good_frames": 12},
  "classifier": null,
  "image": "captures/obj_1760000011.jpg",
  "yolo": {
    "status": "ok", "model": "yolov8n.pt", "count": 1, "time_ms": 212.5, "error": null,
    "top": {"label": "box", "class_id": 0, "confidence": 0.91, "bbox_px": [4.0, 6.0, 170.0, 140.0]},
    "objects": [{"label": "box", "class_id": 0, "confidence": 0.91, "bbox_px": [4.0, 6.0, 170.0, 140.0]}],
    "annotated": "captures/obj_1760000011_yolo.jpg"
  },
  "belt_depth_mm": 425
}
```

- `height_method` (only when `camera_settings.json` has `"height_method"` `plane` or `yolo_ransac`): `yolo_ransac` (a YOLO mask was used, on the full frame or on the crop: see `seg_height.yolo_input`), `plane_depth_blob` (depth blob mask) or `depth_threshold` (the plane height failed). Accepted `camera_settings.json` values are `depth_threshold`, `plane`, `crop_seg` and `yolo_ransac`.
  With `yolo_ransac`, YOLO-seg runs on the best FULL colour frame (`captures/obj_<id>_full.jpg`, annotated copy `..._full_yolo.jpg`), the depth pixels under the object mask are projected to 3D, the belt plane is fitted by RANSAC from the belt pixels around the object, and `object_height_mm` becomes the median distance of the object points above that plane (outliers removed by median/MAD). The depth-threshold value is kept in `height_threshold_mm`.
  `seg_height` explains the outcome: `{"ok", "reason", "label", "overlap", "height_mm", "height_p95_mm", "samples", "rejected", "plane": {"normal", "d_mm", "inliers", "inlier_ratio", "rms_mm"}}`.
  With `plane`, or when YOLO has no mask that covers the depth blob (YOLO down, a detect-only model, a box on the wrong area), the depth blob is the mask (`seg_height.source: "depth_blob"`, the YOLO reason in `seg_height.yolo_reason`). If the plane height also fails (plane not found, no depth), the depth-threshold height stays and `height_method` is `depth_threshold` with the reason in `seg_height.reason`. `along_belt_mm` always comes from the depth blob. The mask polygons are not stored (`polygon_points` only).
- `yolo` (optional, added by `yolo_detect.py`): YOLO run on the SAVED capture file (`image`), after it was written. `null` when `yolo_enabled` is false or there is no image. `yolo.status` is `ok`, `no_object`, `error` or `unavailable` (ultralytics or the model is missing). `bbox_px` here is `[x1,y1,x2,y2]` in the crop image, not the camera frame. The Dobot ignores this key: a YOLO result never allows or blocks a pick.

- `source`: `auto` or `manual`. Manual (web trigger) results have `object_id: null` and are NEVER acted on by the Dobot.
- `status`:
  - `ok`: at least `min_good_frames` (3) frames without a border touch.
  - `no_object`: nothing was found.
  - `border_only`: found, but every candidate touched the image border.
  - `cam_off`: the camera was off or in error during the cycle.
- `along_belt_mm` = `rotated_long_mm` (the longest side, because the belt orientation of the bbox axes is unknown). `along_status` is `ok` for `status ok`, otherwise `unknown`.
- Only `status == "ok"` allows a pick.
- Best frame rule (in `detector.py`): among frames whose bbox does not touch the border, keep the one with the largest `area_px`. Ties keep the earlier frame.

### 4.4 `dobot_status.json` (writer `dobot.py`)

```json
{ "...header": "...", "mode": "auto", "live": false,
  "state": "done", "current_id": null, "handled_id": 1760000011,
  "outcome": "dry_run", "reason": null, "detect_id_used": 1760000020,
  "started_ts": 1760000019.9, "finished_ts": 1760000031.2, "fault": null }
```

- `state`: `idle` (nothing handled this session or after restart), `busy` (`current_id` set), `done`, `fault`.
- `outcome`: `picked_placed`, `dry_run`, `skipped_no_detection`, `skipped_rejected` (the existing validation said no, nothing moved), `failed`.
- `handled_id` is a high-water mark: the last `object_id` that reached a final outcome. It is written, durable, BEFORE the conveyor can see `done`.
- `outcome` `failed` always goes with `state: "fault"`. The skip outcomes go with `state: "done"`.

### 4.5 Clear flags

`python conveyor.py --clear-fault` and `python dobot.py --clear-fault` create `state/clear_<name>.flag` and exit. The running auto program deletes the flag and clears its latched fault. This is the only cross-process command. The operator CLI is the single writer of the flag; the owning program is the single remover. Run these only when the cell is safe.

## 5. Handshake: how `object_id` flows

```
conveyor: idle --Start active 0.2 s--> allocate object_id N, phase=running, belt runs
camera:   sees phase=running + object_id=N -> BestTracker.reset(N), analyse each frame (every segment_every_n)
conveyor: End active 0.05 s -> STOP sent at once, phase=at_end, object_at_end=true, stopped_ts=now
camera:   sees phase=at_end + object_id=N (and was tracking N) -> freeze best, write detect_result{object_id:N}
conveyor: confirms motor stopped + settle 0.5 s -> pick_permission=true
dobot:    pick_permission && object_id=N > handled_id && detect_result.object_id==N && status ok && source auto
          -> write busy(current_id=N) -> pick and place at the fixed point -> write done(handled_id=N, outcome)
conveyor: dobot_status.state in (done, idle) && handled_id == N -> phase=idle, object_id=null, released_ts=now
```

Dobot decision, pure function `decide(conveyor, conveyor_fresh, detect, handled_id, state_busy, waited_s)`. It returns `wait`, `pick` or `skip(outcome, reason)`:

1. Not `conveyor_fresh` or `conveyor is None`: wait.
2. `phase != at_end`, `not pick_permission` or `object_id <= handled_id`: wait.
3. `detect` is None, or `detect.object_id != object_id`, or `detect.source != "auto"`: wait. After `detect_wait_s` (3.0) of waiting under a valid permission, skip with `skipped_no_detection`.
4. `detect.status != "ok"`: skip with `skipped_no_detection` (reason = the status).
5. The existing `validate_plan` and gating (height, L, envelope) fails: skip with `skipped_rejected`.
6. Otherwise pick. Dry run is the default, so a pick without `--live` plus `dobot_settings.json: dobot_live=true` gives outcome `dry_run`. It still reports `done`, so the cycle completes.

Why a stale result can never be used: the `object_id` is unique and rising, a result matches only the current `at_end` object, and `handled_id` blocks any repeat. A result is never judged by its timestamp.

Release modes in `conveyor_settings.json`:
- `release_mode`: `"dobot"` (default), or `"timer"` with `release_timer_s` (5) for bring-up without a robot.
- In `"dobot"` mode, an unfresh `dobot_status` sets `hold_reason: "dobot_stale"` and the belt stays stopped. It never auto-releases. `state: "fault"` sets `hold_reason: "dobot_fault"` and stays stopped until the Dobot clears the fault.

Restart rules:
- Dobot starts, reads its own old `dobot_status` for `handled_id`. If the old state was `busy`, it starts in `fault` with reason `restart_after_busy` and `handled_id = current_id` (the vacuum may hold a part). If there is no valid old file and the conveyor is `at_end` for N, it sets `handled_id = N` with outcome `skipped_rejected` and reason `unknown_restart`. It will not pick N.
- Camera restart mid-cycle: no tracking state, so on `at_end` for an id it never tracked it writes `status: "no_object"`, reason `not_tracked`.
- Conveyor restart: starts in `idle` (auto) or `manual`. An object left on the belt needs an operator.
- The end sensor still active when a cycle would arm: `blocked`. It returns to `idle` when End is clear.
- A cycle with no End within `run_timeout_s` (60): stop, `fault: "end_timeout"`.
- Phase `running` with Start clear is normal (the object has left Start).

## 6. Serial protocol (Pi `conveyor.py` <-> board). KEPT, with additions

115200 baud, newline-terminated ASCII, USB CDC. Board VID `0x2E8A` auto-detect, `--port` override.

Pi to board (unchanged commands):
- `SPEED:<n>` (accepts `S <n>`, `SPEED:n`), `DIR:FORWARD|BACKWARD` (also `F`, `B`), `STOP`, `STATUS`.
- Added: `PING` (no reply, only refreshes the watchdog).

Board to Pi:
- `EVENT S1=<0|1> S2=<0|1> SENSOR=DETECTED|CLEAR` (unchanged, sent on a sensor change only).
- `STATUS S1=.. S2=.. SENSOR=.. ENC=.. SPD=.. DIR=.. WD=<0|1>` (`WD=` is new and appended; parsers read `key=value` pairs and ignore unknown keys).
- `EVENT WATCHDOG` (new): sent once when the watchdog zeroes the speed.
- `BOOT fw=2` (new): sent once at start-up. Anything else is a log line.

Firmware behaviour:
- Pure reporter and driver. The `OBJECT_IN` start/stop logic is REMOVED. The motor runs whenever `target_speed > 0`.
- Unchanged: `MIN_SPEED = 50`, `MAX_SPEED = 1800`, 1/16 microstepping, pin map (STEP GP0, DIR GP1, sensors GP26 and GP27, encoder GP4 and GP5), `PULSE_WIDTH_US`, `SENSOR_ACTIVE = 1`. Speed limits are NOT changed.
- Watchdog 1.5 s: any complete command line refreshes it. If `target_speed > 0` and 1.5 s pass with no line, set `target_speed = 0` and print `EVENT WATCHDOG`. The speed stays 0 until the Pi sends a new `SPEED`.
- Boot state is speed 0 and no watchdog countdown until the first command.
- Pi side keep-alive: while running, resend `SPEED:<n>` every 0.5 s. Poll `STATUS` every 0.2 s (`fw_fresh` is true if a STATUS or EVENT arrived within 1.0 s). On `EVENT WATCHDOG` or `SPD` below the commanded value, the Pi resends.
- The Pi clamps speed to `min(max_speed setting, 1800)`. The board stays the authority. `conveyor_settings.json: max_speed 2000` is a leftover that the board already clamps; the Pi uses 1800.
- Sensor mapping: `S1` = Start (GP26), `S2` = End (GP27), set by `start_sensor` and `end_sensor` in `conveyor_settings.json`. `sensor_active_level` defaults to 1.

## 7. Modules and test boundaries (all testable on a PC with fakes)

Each program has a `main(argv)` that only wires parts together. Hardware access sits behind a small interface that tests replace.

`state_io.py` (stdlib): unit tests with a temp folder. Check atomic replace, corrupt JSON gives None, schema mismatch gives None, `Freshness` with an injected clock, `seq` increments, the lock refuses a second holder.

`firmware_conveyor.py`: command parsing and the watchdog are pure functions (`parse_command(line, state)->state`, `watchdog_expired(now, last_cmd, speed, limit)`). The main loop is split from them. Tests inject fake `machine`, `select` and `time` modules through `sys.modules` before import. The step generation is not tested on a PC.

`conveyor.py`:
- `parse_firmware_line(line)->dict|None` (pure).
- `class FirmwareLink(serial_like)` with `send(line)`, `poll()->events`, `fw_fresh(now)`. Tests use `FakeSerial`.
- `class SensorDebounce(clock)`.
- `class CycleMachine(settings, clock, id_source)` with `step(sensors, dobot_doc, dobot_fresh, link_ok)->(MotorCommand, status_body)`. Pure: it holds the whole phase logic. Tests walk scripted sensor sequences with a fake clock: start, run, end, stop, permission, release, timeout, blocked, serial loss, fault, clear.
- `run_manual(link, settings)` (the former `interactive`) and `run_auto(link, writer, readers, clock)` (the loop).
- It is the only program that opens the serial port. It exits with a clear message if the port is busy.

`detector.py` (numpy and cv2 only, no rs and no camera import):
- Existing: `segment_object`, `measure_object`, `crop_to_bbox`, `classify_crop`.
- Added: `analyze_frame(depth_raw, params, intrinsics)->candidate|None` (segment, border check and measure in one call). `class BestTracker` with `reset(object_id)`, `offer(candidate, frame_ts)`, `freeze()->best|None`. `make_detection_result(tracker_snapshot, object_id, source, detect_id, params)->dict` builds the body in section 4.3, including `status`, `along_belt_mm` and `along_status`. `classify_crop` stays an optional hook and writes `classifier`.
- Tests: extend `test_detector.py` with synthetic depth images, border rejection, largest area wins, `border_only` and `no_object` results.

`camera.py`:
- Keeps `FrameBuffer`, `CameraThread` (RealSense) and the crop and overlay code. Hardware code is not tested on a PC.
- `FrameSource` interface: `latest()->(color, depth_raw, intrinsics, frame_ts)|None`. Tests use `FakeFrameSource`.
- `class AutoCoordinator(frame_source, tracker, writer_detect, settings, clock)` with `step(conveyor_doc, conveyor_fresh)`. It implements section 5 for the camera side.
- `class CameraController` offers `camera_on()`, `camera_off()`, `set_mode()`, `trigger_detect()` (manual only; refused in auto with a clear error), `status()`. The web layer calls only this.
- `python camera.py` = manual mode: camera starts off (or on with `--on`), web page on `--web-port 5000`. `python camera.py --auto` = auto mode.

`web_stream.py` (library, runs inside the camera process; `python web_stream.py` simply calls `camera.main`):
- `create_app(controller, state_dir)->Flask`. One page. Routes: `/`, `/stream/*`, `/snapshot`, `/api/camera` (POST on/off/mode/crop/trigger), `/api/state` (aggregates the four state files read-only: conveyor, camera, detect, dobot).
- Tests: Flask `test_client` with a fake controller and a temp state folder.

`dobot.py`:
- Keeps all pure planning and safety code and `DobotClient`, `PickPlaceController` (dry-run default, emergency stop, validation, fault latch). Pick X and Y are the fixed `(300, 0)` and `Z` is from `belt_z` plus the measured height, as the current planner does. This is an open point for the implementer to confirm against the user's `-100` statement; it is not decided by this contract.
- `extract_scan(result, cfg)` is simplified to read the flat keys of section 4.3. The `along_source` options based on time and bbox are removed.
- Added: `decide(...)` (pure, see section 5), `class AutoLoop(controller, readers, writer, clock)`.
- `python dobot.py` = manual test mode: menu, `--once --height H` for a dry-run pick without any state files, `--live` as before. `--auto` = the loop.
- Tests: `test_dobot.py` keeps the fake TCP robot tests. Add `decide` tests (stale, wrong id, repeat id, wrong source, `no_object`, timeout) and `AutoLoop` tests with fake readers. The `SensorEdgeHandler` tests are moved to the `CycleMachine` tests.

## 8. piSerial.py (launcher and supervisor only)

- Starts, in this order, `conveyor.py --auto`, `camera.py --auto`, `dobot.py --auto` as `subprocess.Popen([sys.executable, script, "--state-dir", abs, ...])`. Options: `--only a,b`, `--dobot-live` (passed through), `--state-dir`.
- Restarts a child that exits, with backoff 1, 2, 4, ... up to 30 s. The backoff resets after 60 s of uptime. Never gives up. Logs each start and exit code to the console and `logs/piSerial.log`. Child output goes to `logs/<name>.log` (rotating, 1 MB x 3).
- On SIGINT or SIGTERM: terminate the children (dobot first, camera, then conveyor), wait 5 s, then kill.
- No state reading, no sequencing, no serial, no imports of the other programs, no interactive menu. The old settings handling, `SensorEdgeHandler` and the scanner and dobot callbacks leave this file.
- Test: unit-test the restart and backoff logic with a fake process factory.

## 9. Error handling summary

| Operation | Failure | Class | Caller / result | Log |
|---|---|---|---|---|
| State read | missing, corrupt, wrong schema | recoverable | `None` ("unknown"), the reader does not act | WARNING once per change |
| State write | `PermissionError` or `OSError` | recoverable | retry 3 times, then skip, the next heartbeat retries. 10 failures in a row: conveyor goes to `fault state_write_failed` and stops the motor | WARNING, ERROR at 10 |
| Serial open | busy or missing | fatal for conveyor | exit 1 with a message. piSerial restarts it with backoff | ERROR |
| Serial lost mid-run | read or write error | recoverable | `link.connected=false`, `running` becomes `fault serial_lost`, the board watchdog stops the motor within 1.5 s, reconnect every 2 s | ERROR |
| Unknown firmware line | garbage | ignorable | dropped | DEBUG |
| Settings file | missing or bad value | recoverable | defaults, numbers clamped (`speed` 0 to 1800, durations above 0) | WARNING |
| Camera start | no device or pipeline error | recoverable | `camera.state=error`, retry every 3 s, results with `cam_off` | ERROR once |
| Trigger in auto mode | refused | n/a | HTTP 409 with a message | INFO |
| Dobot TCP failure | refused, timeout, bad reply | existing behaviour | `state fault`, outcome `failed`, `handled_id=N`, the conveyor holds (`dobot_fault`) | ERROR |
| Second instance | lock held | fatal | exit 1, "already running" | ERROR |

## 10. Assumptions to confirm in review

- The belt moves from Start toward End and the robot places the part downstream of End (existing `CONVEYOR_FORWARD_AXIS`). A placed part that still covers End gives `blocked` until it is cleared.
- Camera depth is aligned to color as today, and the camera is on a fixed mount, so `belt_depth_mm` is a camera setting (425 as in `scanner.py`).
- The pick Z formula is left to `dobot.py` (section 7). The contract only supplies `object_height_mm`.
- Heartbeat rates and thresholds are starting values in each program's settings; they can be tuned without changing the schemas.
