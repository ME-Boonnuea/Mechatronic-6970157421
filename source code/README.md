# Conveyor cell: Raspberry Pi control group and Docker YOLO group

This repository contains the Raspberry Pi control software for a conveyor cell.
The physical system is:

- Raspberry Pi: runs the conveyor, camera, web UI, YOLO client, and Dobot control.
- RP2350 Pro (Pico Zero): runs the conveyor firmware, reads the sensors, and drives the conveyor motor.
- Intel RealSense D435i: provides colour and depth frames to the Raspberry Pi.
- Dobot MG400: receives pick-and-place commands from the Raspberry Pi.

The Raspberry Pi programs coordinate through small JSON files in `state/`. The
file formats and handshake are defined in `STATE_CONTRACT.md`.

| File | Runs on | Job |
|---|---|---|
| `conveyor.py` | Raspberry Pi | Only process that opens the serial port to the RP2350 Pro. Runs the belt and writes conveyor state. |
| `camera.py` + `web_stream.py` | Raspberry Pi | Captures from the RealSense D435i, performs depth analysis, serves the web page, and requests YOLO results. |
| `detector.py` | Raspberry Pi library | Pure OpenCV/numpy depth segmentation, measurement, cropping, and height calculation. It does not call YOLO. |
| `dobot.py` | Raspberry Pi | Validates the result and controls the Dobot MG400 pick-and-place cycle. |
| `piSerial.py` | Raspberry Pi | Starts and supervises the three automatic Pi processes. |
| `state_io.py` | Raspberry Pi library | Shared JSON state-file and freshness helpers. |

The RP2350 Pro firmware is a separate board-side dependency and is not included
in this repository. Only the Pi-side `conveyor.py` belongs in this group.

## Docker YOLO group

| File | Runs in | Job |
|---|---|---|
| `yolo_server.py` | Ultralytics Docker container | Loads one YOLO model and exposes `/health` and `/predict`. |
| `yolo_detect.py` | Imported inside the container | Performs Ultralytics inference and returns labels, confidence, boxes, and segmentation polygons. |

In the deployed split, Ultralytics and the `.pt` model are installed/mounted in
Docker only. `camera.py` stays on the Raspberry Pi and sends a saved capture's
relative path to `yolo_server.py` over `127.0.0.1`. The shared state folder is
mounted into both environments so the container can read captures and write an
annotated image.

Important: `detector.py` is not the YOLO caller. It consumes the YOLO result
when a YOLO-based height method is selected, but the inference call is made by
`yolo_detect.py`, normally through `yolo_server.py`.

## RP2350 Pro board boundary

The board firmware is maintained separately. It reads the Start and End
sensors and drives the conveyor motor. The Raspberry Pi communicates with it
through the serial protocol implemented by `conveyor.py`; `conveyor.py` does
not contain the motor-control firmware and must not be copied to the board.

## Install (Pi)

```
sudo apt install python3-serial python3-flask python3-opencv python3-numpy
pip install pyrealsense2        # or the Intel librealsense build for your Pi
```

Copy this project to the Raspberry Pi. Keep the state files in RAM when possible
(they are written frequently):

```
export CELL_STATE_DIR=/dev/shm/cell_state
```

Order of choice for the state folder: `--state-dir`, then `CELL_STATE_DIR`,
then `Project/state/`.

## Run the Raspberry Pi control group (auto mode)

```
python3 piSerial.py                    # conveyor, camera, dobot (dobot in DRY RUN)
python3 piSerial.py --only conveyor,camera       # no robot yet
python3 piSerial.py --dobot-live       # real robot, see "Live robot" below
```

To run WITHOUT the robot, list only the programs you want with `--only`
(names: `conveyor`, `camera`, `dobot`; order is always conveyor, camera, dobot).
`python3 piSerial.py --only conveyor,camera` does not start `dobot.py` at all.
To run only the robot loop: `--only dobot`. No separate `--no-dobot` flag exists.

Open the web page at `http://rc.local:5000`. Logs: `logs/piSerial.log` and
`logs/<program>.log`. Stop with Ctrl+C: dobot, camera, conveyor are stopped in
that order.

What happens for one object (belt moves Start to End, camera is between them):

1. The RP2350 Pro reports the Start sensor. `conveyor.py` assigns an object id and runs the belt.
2. `camera.py` reads RealSense frames and passes depth arrays to `detector.py`, which keeps the best valid object frame.
3. Object reaches the End sensor. The belt stops at once.
4. `camera.py` freezes the best frame and writes `detect_result.json` with the same object id.
5. If enabled, `camera.py` asks the Docker YOLO server to analyse the saved capture. YOLO is supplementary and does not authorize a pick.
6. After the belt is settled, `conveyor.py` sets `pick_permission`.
7. `dobot.py` validates the matching result, then commands the MG400 to pick at the fixed point and place.
8. `dobot.py` writes `done` with the object id. `conveyor.py` releases and waits for the next object.

Test one object at a time first.

## Run each program alone (manual mode)

Each program has no state-file input in manual mode, so the others do not need to run.

**conveyor.py** (needs the board connected; stop `piSerial.py` first, only one program may open the port)
```
python3 conveyor.py                    # menu: s <speed>, f, b, stop, status, show, save, q
python3 conveyor.py --port /dev/ttyACM0
python3 conveyor.py --no-autostart     # do not send the saved speed on connect
python3 conveyor.py --auto             # auto mode alone: writes state/conveyor_status.json
python3 conveyor.py --clear-fault      # after a latched fault, once the cell is safe
```

**camera.py** (manual: on/off by hand, still shown on the web page)
```
python3 camera.py                      # camera off; press "Camera ON" on the page
python3 camera.py --on                 # camera on at start
python3 camera.py --web-port 5000
python3 camera.py --auto               # auto mode alone: follows state/conveyor_status.json
```
`python3 web_stream.py` does the same as `python3 camera.py`.
The page has: live colour and depth stream, Camera ON/OFF, mode switch, crop toggle,
"Trigger detection" (manual mode only, refused in auto mode), the latest detection
result with its crop, and the state of conveyor and dobot (read only).
A manual detection is saved with `source: manual` and no object id, and the Dobot never acts on it.
Camera settings (belt depth, thresholds, port) are in `camera_settings.json`.

**YOLO on the saved capture** is handled by the Docker YOLO group below.
After `camera.py` saves `captures/obj_<id>.jpg`, it requests inference from
`yolo_server.py`. The returned `yolo` block contains labels, confidence, boxes,
and optional segmentation polygons. Settings in `camera_settings.json` include
`yolo_enabled`, `yolo_mode`, `yolo_url`, `yolo_timeout_s`, `yolo_conf`,
`yolo_imgsz`, and `yolo_save_annotated`. If the server is unavailable, the result
says `unavailable` and the rest of the cell keeps working. YOLO never allows or
blocks a pick.

### Height from the belt plane (`"height_method"` in `camera_settings.json`)

| Value | Mask used for the height | YOLO runs on |
|---|---|---|
| `depth_threshold` | none: fixed `belt_depth_mm` minus the mean depth | the saved crop |
| `crop_seg` (now set) | the YOLO-seg mask found on the crop, moved back to full-image coordinates; if there is none, the depth blob | the saved crop |
| `plane` | the depth blob (the same area as the crop) | the saved crop, label only |
| `yolo_ransac` | the YOLO-seg mask on the object; if there is none, the depth blob | the full colour frame |

`crop_seg` is the pipeline: the depth blob finds the object, OpenCV crops it (with `crop_margin_px`),
`yolo11n-seg.pt` segments only that crop, the mask is moved back by the crop origin, and the depth
under the mask gives the height. YOLO gives only the outline, the depth comes from the camera. A mask
is accepted only if it covers at least 30 % of the depth blob. Run the container with
`--model /models/yolo11n-seg.pt`. A COCO model may not know your objects; then the blob is used and
`seg_height.yolo_reason` says why.

For `plane` and `yolo_ransac` the steps are: depth pixels inside the mask (shrunk 3 px) become 3D
points with the camera intrinsics; a belt plane is fitted by RANSAC to the belt pixels around the
object; the height is the median distance of the object points above that plane, outliers dropped.
`object_height_mm` in `detect_result.json` is that value, the old one is kept in
`height_threshold_mm`, and `height_method` / `seg_height` say which mask was used and why.

**Belt reference** (`"belt_reference"`, applies to `plane` and `yolo_ransac`):
`"fixed"` (now set) uses `belt_depth_mm` as a flat reference, so height = `belt_depth_mm` minus the
object depth (median of the mask, outliers dropped). It needs no belt pixels around the object, so it
works on a narrow belt, but it is only right if the camera looks straight down and `belt_depth_mm`
is the real distance to the empty belt (read it from the crosshair on the web stream with the belt
empty). A 20 mm error in `belt_depth_mm` is a 20 mm error in every height.
`"fit"` fits the belt plane from the pixels around the object, so it follows a tilted camera or a
moved belt, but needs enough visible belt. `"auto"` tries the fit and falls back to `"fixed"`.
`seg_height.plane.reference` in the result says which was used (`fixed` or `fitted`).

`plane` does not depend on YOLO, so use it while the YOLO model is not trained for your objects
(a COCO model often boxes the wrong area). Switch to `yolo_ransac` after you train a `-seg` model
(for example `yolo26n-seg.pt`, `--model /models/yolo26n-seg.pt` in the `docker run` command).
A YOLO mask is only accepted when it covers at least 30 % of the depth blob, so a box on the wrong
area is ignored. `"seg_fallback": "threshold"` makes `yolo_ransac` keep the old height instead of
using the blob. The Dobot reads `object_height_mm`, so a new height changes what it uses.
Tuning values (`plane_band_mm`, `plane_ring_px`, `plane_inlier_tol_mm`, `mask_erode_px`,
`height_mad_k`, `seg_min_overlap`, `seg_fallback`) have defaults and can be added to the file.
Check a few known objects against a ruler before trusting the numbers.

## YOLO in Docker (`"yolo_mode": "server"`)

Use this deployment so Ultralytics and the model are only in Docker, not on the
Raspberry Pi. `yolo_server.py` runs inside the container and keeps the model
loaded. `camera.py` sends the path of the saved capture (relative to the state
folder) and gets the result back. The container sees only the state folder, the
project folder (read only), and the model folder.

1. Create the folders FIRST. If a mounted folder does not exist, Docker creates it
   owned by root and `camera.py` then cannot write to it:
   ```
   mkdir -p /dev/shm/cell_state ~/models
   cp best.pt ~/models/
   ```
2. Start the container (once; it restarts by itself after a reboot):
   ```
   sudo docker run -d --name yolo --restart unless-stopped --network host --ipc=host \
     -v /dev/shm/cell_state:/state \
     -v ~/Project:/app:ro \
     -v ~/models:/models:ro \
     ultralytics/ultralytics:latest-arm64 \
     python3 /app/yolo_server.py --model /models/best.pt --root /state --port 8765
   ```
   Use your own state folder in the first `-v` (the same one `camera.py` uses:
   `--state-dir`, `CELL_STATE_DIR`, or `~/Project/state`). `~/Project` is the Pi
   folder that holds `yolo_server.py` and `yolo_detect.py`.
3. Check it: `sudo docker logs yolo` should end with `listening on http://127.0.0.1:8765`,
   and `curl http://127.0.0.1:8765/health` should print `"ok": true`.
4. In `camera_settings.json`: `"yolo_enabled": true, "yolo_mode": "server"`
   (`yolo_url` and `yolo_timeout_s` are optional). `yolo_conf` and `yolo_imgsz` are sent
   to the server with every request.

If the container is down, `camera.py` keeps working and the result says `unavailable`.
The server listens on `127.0.0.1` only. Restart after changing the model:
`sudo docker restart yolo`.

The request path is:

```
camera.py  ->  HTTP POST /predict  ->  yolo_server.py
                                      ->  yolo_detect.py / Ultralytics
                                      <-  YOLO result JSON
```

The normal depth result is still produced by `detector.py` on the Raspberry Pi.
Depending on `height_method`, the returned YOLO segmentation mask may refine the
height estimate, but the Dobot safety and pick decision remain in `dobot.py`.

**dobot.py** (dry run unless `--live`)
```
python3 dobot.py                       # menu: pick, show, dclear, dstop, enable, disable, ralarm, do, q
python3 dobot.py --once --height 40    # one dry-run cycle without any state files
python3 dobot.py --once --height 40 --length 120
python3 dobot.py --auto                # auto mode alone: follows the conveyor status file
python3 dobot.py --clear-fault         # clear a latched fault (check the arm and vacuum first)
```

## Dobot: fixed pick point

The robot does not move to the object. It always picks at the fixed point
(X, Y, Z) = (300, 0, -100) (the End sensor position; `pick_z` in the dobot
config). The height from the detection result does NOT change the arm Z; it is
only checked against limits. The length only moves the place point
(`Y = 0 + length + 40`, same fixed Z). A result that fails the checks is
skipped with outcome `skipped_rejected`; nothing moves.
(X+ left, X- right, Y+ forward, Y- back, Z+ up.)

### Manual commands in the `dobot.py` menu

| Command | What it sends (live) | Notes |
|---|---|---|
| `enable` | `EnableRobot()` | Refused while a fault is latched (use `dclear` first). |
| `disable` | `DisableRobot()` | Allowed during a fault. Does not release vacuum/blow. |
| `ralarm` | `ClearError()` then `ResetRobot()` | Resets the ROBOT's alarm only. It does NOT clear the software fault latch; run `dclear` for that (it also sends `ClearError()`). |
| `do <index> <0\|1>` | `DO(index,value)` | Index must be an integer 1..16 (`manual_do_max`), value 0 or 1; anything else is refused and nothing is sent. Refused while a fault is latched. `do 1 0` releases the vacuum by hand. |

All of them: dry run (default) only logs `DRY RUN would send ...` and opens no
socket; live mode uses the dashboard port 29999 only; they are refused while a
pick cycle is running (use `dstop`). The automatic cycle is unchanged and still
allows only the vacuum and blow outputs. The DO range (16 terminals) and the
`ResetRobot()` behaviour are not verified on a real MG400.

### Live robot

Dry run is the default everywhere. Real motion needs BOTH `--live` (or
`piSerial.py --dobot-live`) and `"dobot_live": true` in `dobot_settings.json`.
The Dobot IP is `dobot_ip` in the same file (or `--ip`).

## Files in `state/`

| File | Written by | Read by |
|---|---|---|
| `conveyor_status.json` | conveyor.py | camera.py, dobot.py, web page |
| `camera_status.json` | camera.py | web page |
| `detect_result.json` | camera.py | dobot.py, web page |
| `dobot_status.json` | dobot.py | conveyor.py, web page |
| `dobot_last_cycle.json` | dobot.py | (record of the last cycle) |
| `captures/obj_<id>.jpg` | camera.py | web page (newest 50 kept) |
| `clear_conveyor.flag`, `clear_dobot.flag` | `--clear-fault` | the running program deletes it |

Settings you edit by hand stay next to the programs: `conveyor_settings.json`,
`camera_settings.json`, `dobot_settings.json`.

## State files

The main files are `conveyor_status.json`, `camera_status.json`,
`detect_result.json`, `dobot_status.json`, and `dobot_last_cycle.json`.
Captures are stored under `state/captures/`. See `STATE_CONTRACT.md` for the
complete producer/consumer table and object-id handshake.
