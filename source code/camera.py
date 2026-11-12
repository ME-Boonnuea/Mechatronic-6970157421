"""
camera.py
RealSense camera capture module for Raspberry Pi.
Runs headless — no display needed.

Connects to the RealSense, holds the camera settings, and (in auto mode) tracks
an object while it crosses the camera area. Depth segmentation and measurement
are pure functions in detector.py and run in this Raspberry Pi process.

YOLO is a separate optional stage. In the deployed setup this process saves a
capture, asks yolo_server.py in the Docker container to analyse it, and merges
the returned result into detect_result.json. The Pi does not load Ultralytics
when yolo_mode is "server". web_stream.py is the web page served by this process.

Manual mode (no flag):  python camera.py [--on]
    Camera starts off (or on with --on); turn it on/off and trigger a
    detection by hand on the web page.
Auto mode:              python camera.py --auto
    Reads state/conveyor_status.json. Detects every analysed frame from
    phase "running" (after Start sensor) until phase "at_end" (End sensor
    stopped the object), keeps the best frame, then freezes it and writes
    state/detect_result.json (see STATE_CONTRACT.md).

Features:
  - Color stream with optional crop
  - Depth stream as JET colormap heatmap
  - Side-by-side combined view (color + depth)
  - Depth distance value overlay at crop center

The hardware code (CameraThread) needs pyrealsense2; everything else imports
fine on a PC without it (tests use FakeFrameSource).
"""

import argparse
import glob
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:          # PC without the SDK: tests, web page layout work
    rs = None

import detector
import state_io
import yolo_detect

# =====================================
# CONFIGURATION
# =====================================

WIDTH  = 640
HEIGHT = 480
FPS    = 30

JPEG_QUALITY = 55    # lowered from 80 — less CPU, smaller frames, lower latency

# ---- Crop settings ----
# Set CROP_ENABLE = True to crop all streams
# Crop box defined as (x, y, width, height) in pixels
# x, y = top-left corner of crop region
CROP_ENABLE = False
CROP_X      = 160   # start x  (e.g. 160 crops 160px from left)
CROP_Y      = 120   # start y  (e.g. 120 crops 120px from top)
CROP_W      = 320   # crop width
CROP_H      = 240   # crop height

# ---- Depth overlay ----
# Draw distance value (mm) at center of the frame/crop
DEPTH_OVERLAY = True

# =====================================
# CROP HELPER
# =====================================

def apply_crop(frame):
    """Crop a frame if CROP_ENABLE is True."""
    if not CROP_ENABLE:
        return frame
    x1 = CROP_X
    y1 = CROP_Y
    x2 = CROP_X + CROP_W
    y2 = CROP_Y + CROP_H
    # Clamp to frame size
    h, w = frame.shape[:2]
    x1 = max(0, min(x1, w))
    y1 = max(0, min(y1, h))
    x2 = max(0, min(x2, w))
    y2 = max(0, min(y2, h))
    return frame[y1:y2, x1:x2]


def draw_crop_box(frame):
    """Draw crop region outline on the full frame for reference."""
    if not CROP_ENABLE:
        return frame
    out = frame.copy()
    cv2.rectangle(
        out,
        (CROP_X, CROP_Y),
        (CROP_X + CROP_W, CROP_Y + CROP_H),
        (0, 255, 0), 2
    )
    cv2.putText(out, "Crop region", (CROP_X + 4, CROP_Y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out


def get_center_depth_mm(depth_frame_raw):
    """Return depth in mm at center of frame (or crop center)."""
    if depth_frame_raw is None:
        return 0
    h, w = depth_frame_raw.shape[:2]
    if CROP_ENABLE:
        cx = CROP_X + CROP_W // 2
        cy = CROP_Y + CROP_H // 2
    else:
        cx = w // 2
        cy = h // 2
    cx = max(0, min(cx, w - 1))
    cy = max(0, min(cy, h - 1))
    return int(depth_frame_raw[cy, cx])


def draw_depth_overlay(frame, depth_mm):
    """Draw crosshair and depth value at center of frame."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    # Crosshair
    cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
    cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)
    # Distance text
    if depth_mm > 0:
        text = "{} mm".format(depth_mm)
    else:
        text = "-- mm"
    cv2.putText(frame, text, (cx + 18, cy + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
    return frame

# =====================================
# SHARED FRAME BUFFER
# =====================================

class FrameBuffer:
    """Thread-safe latest-frame buffer."""

    def __init__(self):
        self._lock          = threading.Lock()
        self._color_jpeg    = None
        self._depth_jpeg    = None
        self._combined_jpeg = None
        self._depth_raw     = None
        self._color_raw     = None
        self._depth_cmap    = None
        self.frame_count    = 0
        self.fps            = 0.0
        # Camera intrinsics {fx, fy, ppx, ppy, width, height}, set by CameraThread.
        # Kept after the camera stops so a late freeze can still read it.
        self.intrinsics     = None
        self.center_depth   = 0
        self.camera_active  = False   # True when camera is running
        self.frame_ts       = 0.0     # time.time() of the latest frame

        # Pre-generate a dark "camera off" frame
        self._dark_jpeg = self._make_dark_frame()

    def _make_dark_frame(self):
        """Black frame with 'Camera Off' text — served when camera is stopped."""
        w = CROP_W if CROP_ENABLE else WIDTH
        h = CROP_H if CROP_ENABLE else HEIGHT
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        text  = "Camera Off"
        font  = cv2.FONT_HERSHEY_SIMPLEX
        scale = 1.0
        thick = 2
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        cx = (w - tw) // 2
        cy = (h + th) // 2
        cv2.putText(frame, text, (cx, cy), font, scale, (80, 80, 80), thick)
        _, jpeg = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return jpeg.tobytes()

    def update(self, color_bgr, depth_raw, depth_colormap):
        """
        color_bgr      : full-size color frame (numpy BGR)
        depth_raw      : raw 16-bit depth array (numpy), or None
        depth_colormap : depth rendered as JET colormap (numpy BGR), or None
        """
        # --- center depth value ---
        center_depth = get_center_depth_mm(depth_raw)

        # --- apply crop ---
        color_cropped = apply_crop(color_bgr)
        depth_cropped = apply_crop(depth_colormap) if depth_colormap is not None else None

        # --- depth overlay on cropped frames ---
        if DEPTH_OVERLAY and depth_raw is not None:
            color_out = draw_depth_overlay(color_cropped.copy(), center_depth)
            depth_out = draw_depth_overlay(depth_cropped.copy(), center_depth) \
                        if depth_cropped is not None else None
        else:
            color_out = color_cropped
            depth_out = depth_cropped

        # --- encode color JPEG ---
        _, color_jpeg = cv2.imencode(
            ".jpg", color_out,
            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        )

        # --- encode depth JPEG ---
        depth_jpeg = None
        if depth_out is not None:
            _, depth_jpeg = cv2.imencode(
                ".jpg", depth_out,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

        # --- combined side-by-side view ---
        combined_jpeg = None
        if depth_out is not None:
            # resize depth to match color if sizes differ after crop
            if color_out.shape != depth_out.shape:
                depth_out = cv2.resize(depth_out, (color_out.shape[1], color_out.shape[0]))
            combined = np.hstack([color_out, depth_out])
            # add labels
            cv2.putText(combined, "Color", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.putText(combined, "Depth", (color_out.shape[1] + 10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            _, combined_jpeg = cv2.imencode(
                ".jpg", combined,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

        with self._lock:
            self._color_jpeg    = color_jpeg.tobytes()
            self._depth_jpeg    = depth_jpeg.tobytes() if depth_jpeg is not None else None
            self._combined_jpeg = combined_jpeg.tobytes() if combined_jpeg is not None else None
            # depth_raw / depth_colormap are fresh arrays owned by us and never
            # mutated afterwards, so readers can share them without copying.
            # color_bgr may alias librealsense memory, so copy it.
            self._depth_raw     = depth_raw if depth_raw is not None else None
            self._color_raw     = color_bgr.copy()
            self._depth_cmap    = depth_colormap
            self.center_depth   = center_depth
            self.frame_count   += 1
            self.frame_ts       = time.time()
            self.camera_active  = True

    def get_color_jpeg(self):
        with self._lock:
            return self._color_jpeg if self._color_jpeg is not None else self._dark_jpeg

    def get_depth_jpeg(self):
        with self._lock:
            return self._depth_jpeg if self._depth_jpeg is not None else self._dark_jpeg

    def get_combined_jpeg(self):
        with self._lock:
            return self._combined_jpeg if self._combined_jpeg is not None else self._dark_jpeg

    def get_depth_raw(self):
        """Return the latest raw depth array (mm, uint16), or None.
        No copy: the array is never modified after update(). Do not write to it."""
        with self._lock:
            return self._depth_raw

    def get_frames(self):
        """Return (frame_count, color_bgr, depth_raw_mm, depth_colormap) of the
        latest frame (arrays are shared, read-only), or None if camera is off."""
        with self._lock:
            if self._depth_raw is None or self._color_raw is None:
                return None
            return (self.frame_count, self._color_raw,
                    self._depth_raw, self._depth_cmap)

    def get_frame_with_ts(self):
        """Return (color_bgr, depth_raw_mm, frame_ts) of the latest frame, or None."""
        with self._lock:
            if self._depth_raw is None or self._color_raw is None:
                return None
            return (self._color_raw, self._depth_raw, self.frame_ts)

    def clear(self):
        """Clear frames when camera stops — web stream will show dark frame."""
        with self._lock:
            self._color_jpeg    = None
            self._depth_jpeg    = None
            self._combined_jpeg = None
            self._depth_raw     = None
            self._color_raw     = None
            self._depth_cmap    = None
            self.camera_active  = False


# Singleton buffer shared with web_stream.py
frame_buffer = FrameBuffer()

# =====================================
# CAMERA INFO HELPERS
# =====================================

def _get_info(device, info):
    return device.get_info(info) if device.supports(info) else "Unknown"


def find_realsense_cameras():
    """Return list of connected RealSense camera info dicts."""
    if rs is None:
        return []
    ctx     = rs.context()
    devices = ctx.query_devices()
    cameras = []
    for d in devices:
        cameras.append({
            "name":     _get_info(d, rs.camera_info.name),
            "serial":   _get_info(d, rs.camera_info.serial_number),
            "firmware": _get_info(d, rs.camera_info.firmware_version),
        })
    return cameras

# =====================================
# CAPTURE THREAD
# =====================================

class CameraThread(threading.Thread):
    """
    Background thread that continuously reads from the RealSense camera
    and pushes frames into frame_buffer.
    """

    def __init__(self, enable_depth=True):
        super().__init__(daemon=True)
        self.enable_depth = enable_depth
        self.running      = False
        self.pipeline     = None
        self.error        = None
        self._depth_to_mm = 1.0
        self._fps_time    = time.time()
        self._fps_count   = 0

    def run(self):
        if rs is None:
            self.error = "pyrealsense2 is not installed"
            print("[CAMERA] Error: {}".format(self.error))
            return
        self.running = True
        pipeline     = rs.pipeline()
        config       = rs.config()

        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        if self.enable_depth:
            config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

        # align depth to color frame
        align = rs.align(rs.stream.color)

        try:
            profile = pipeline.start(config)
            self.pipeline = pipeline

            # Intrinsics: depth is aligned TO COLOR (rs.align(rs.stream.color)),
            # so aligned depth pixels live in the colour camera's geometry.
            # The correct intrinsics are therefore the COLOR stream's, not depth's.
            intr = profile.get_stream(rs.stream.color) \
                          .as_video_stream_profile().get_intrinsics()
            frame_buffer.intrinsics = {
                "fx": intr.fx, "fy": intr.fy,
                "ppx": intr.ppx, "ppy": intr.ppy,
                "width": intr.width, "height": intr.height,
            }
            if self.enable_depth:
                # Raw z16 units * depth_scale = metres. Normalise to mm so the
                # rest of the code can treat raw values as mm (D405 uses 0.0001).
                self._depth_to_mm = \
                    profile.get_device().first_depth_sensor().get_depth_scale() * 1000.0
                print("[CAMERA] depth scale -> {} mm/unit".format(self._depth_to_mm))
            print("[CAMERA] Stream started  {}x{}  {} fps  depth={}  crop={}".format(
                WIDTH, HEIGHT, FPS, self.enable_depth, CROP_ENABLE))

            while self.running:
                frames = pipeline.wait_for_frames(timeout_ms=5000)

                if self.enable_depth:
                    frames = align.process(frames)

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_bgr = np.asanyarray(color_frame.get_data())

                depth_raw      = None
                depth_colormap = None

                if self.enable_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth_raw = np.asanyarray(depth_frame.get_data())
                        if abs(self._depth_to_mm - 1.0) > 1e-6:
                            depth_raw = (depth_raw * self._depth_to_mm).astype(np.uint16)
                        else:
                            depth_raw = depth_raw.copy()   # detach from librealsense buffer
                        depth_colormap = cv2.applyColorMap(
                            cv2.convertScaleAbs(depth_raw, alpha=0.03),
                            cv2.COLORMAP_JET
                        )

                frame_buffer.update(color_bgr, depth_raw, depth_colormap)

                # Track FPS
                self._fps_count += 1
                now = time.time()
                if now - self._fps_time >= 5.0:
                    frame_buffer.fps = self._fps_count / (now - self._fps_time)
                    self._fps_count  = 0
                    self._fps_time   = now

        except Exception as e:
            self.error = str(e)
            print("[CAMERA] Error: {}".format(e))
        finally:
            try:
                pipeline.stop()
            except Exception as e:
                print("[CAMERA] pipeline.stop error: {}".format(e))
            self.running = False
            frame_buffer.clear()   # only after the pipeline is really stopped
            print("[CAMERA] Stream stopped")

    def stop(self, timeout=5.0):
        """Request stop and wait for the pipeline to be released (so a quick
        restart does not hit 'Device or resource busy')."""
        self.running = False
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=timeout)




# =====================================
# SETTINGS (camera_settings.json, hand-edited; unknown keys are kept)
# =====================================

BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(BASE_DIR, "camera_settings.json")

DEFAULT_SETTINGS = {
    "belt_depth_mm":       425,    # camera to empty belt surface
    "object_threshold_mm": 20,     # pixel must be this many mm closer than the belt
    "min_valid_depth_mm":  50,     # ignore lens reflections
    "min_area_px":         300,    # smallest accepted blob
    "min_good_frames":     3,      # frames without border touch needed for status "ok"
    "segment_every_n":     1,      # analyse every N-th new frame (raise to save Pi CPU)
    "crop_margin_px":      10,     # margin around the saved best crop
    "max_saved_crops":     50,     # keep the newest N captures/obj_*.jpg
    "crop_jpeg_quality":   90,
    "web_port":            5000,
    "heartbeat_s":         0.5,    # camera_status.json rewrite period
    "retry_s":             3.0,    # camera restart period after an error
    "manual_trigger_frames":     5,
    "manual_trigger_timeout_s":  2.0,
    "yolo_enabled":        False,  # ask the configured YOLO service for each saved capture
    "yolo_model":          "yolov8n.pt",
    "yolo_conf":           0.25,
    "yolo_imgsz":          640,
    "yolo_save_annotated": True,   # also save captures/obj_<id>_yolo.jpg
    "height_method":       "depth_threshold",  # "crop_seg" = YOLO-seg on the crop, mask moved back, depth under it;
                                               # "plane" = height of the depth blob,
                                               # "yolo_ransac" = same with the YOLO-seg mask (needs a -seg model)
    "yolo_mode":           "local",  # deployment uses "server": yolo_server.py runs in Docker
    "yolo_url":            "http://127.0.0.1:8765",
    "yolo_timeout_s":      2.0,    # server mode: give up on one object after this long
}


def load_settings(path=None):
    """Settings dict; missing or unreadable file -> defaults (unknown keys kept)."""
    path = path or SETTINGS_FILE
    data = {}
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("not a JSON object")
        except Exception as e:
            print("[CAMERA] Cannot read {}: {} (using defaults)".format(path, e))
            data = {}
    for k, v in DEFAULT_SETTINGS.items():
        data.setdefault(k, v)
    return data


def detector_params(settings):
    """The subset of settings detector.py needs."""
    return {k: settings[k] for k in detector.DEFAULT_PARAMS}


def analyze_with_frame(depth, params, intrinsics):
    """detector.analyze_frame + keep the depth frame and intrinsics on the candidate,
    so the segmentation height can use them later (the arrays are shared, read-only)."""
    cand = detector.analyze_frame(depth, params, intrinsics)
    if cand is not None:
        cand["depth"], cand["intrinsics"] = depth, intrinsics
    return cand


def add_yolo_and_height(body, best, image_rel, name, saver, analyze_image, settings, params):
    """
    YOLO result (and optionally the segmentation height) for one finished detection.
      height_method "depth_threshold": YOLO runs on the saved crop (image_rel); the
                                       height stays as detector.py measured it.
      height_method "crop_seg":        YOLO-seg runs on the saved CROP (only the object in view);
                                       its mask is moved back by the crop origin and used for the
                                       plane height when it covers the depth blob, else the blob.
      height_method "plane":           YOLO runs on the saved crop (label only); the height is
                                       the belt-plane height of the DEPTH BLOB. YOLO plays no
                                       part in the number. Works with any model.
      height_method "yolo_ransac":     the best FULL colour frame is saved and YOLO-seg
                                       runs on it; the height becomes the plane height of
                                       its mask (detector.refine_result_with_seg). With no
                                       matching mask the depth blob is used, and if that
                                       fails the depth-threshold height stays.
    Never raises.
    """
    method = settings.get("height_method", "depth_threshold")
    seg_mode = method == "yolo_ransac"
    yolo = None
    if analyze_image is not None:
        full_saver = getattr(saver, "save_full", None)
        if seg_mode and best is not None and full_saver is not None \
                and best.get("payload") is not None:
            try:
                yolo = analyze_image(full_saver(name, best["payload"]))
            except Exception as e:
                print("[CAMERA] full-frame YOLO failed: {}".format(e))
        elif image_rel:
            yolo = analyze_image(image_rel)
        if image_rel or yolo is not None:
            body["yolo"] = yolo
    if method in ("yolo_ransac", "crop_seg", "plane"):
        try:
            offset, yolo_input = (0, 0), "full"
            if method == "crop_seg" and best is not None:
                # YOLO ran on the crop: move its mask back into full-image coordinates
                offset = detector.crop_origin(best["candidate"]["meas"]["bbox_px"],
                                              int(settings.get("crop_margin_px", 10)))
                yolo_input = "crop"
            detector.refine_result_with_seg(body, best, params, use_yolo=(method != "plane"),
                                            offset=offset, yolo_input=yolo_input)
        except Exception as e:                       # keep the depth-threshold height
            body["height_method"] = "depth_threshold"
            body["seg_height"] = {"ok": False, "reason": "error: {}".format(e)}
            print("[CAMERA] segmentation height failed: {}".format(e))
        sh = body.get("seg_height") or {}
        print("[CAMERA] height {}{}".format(
            body.get("height_method"),
            "" if sh.get("ok") else " (seg: {})".format(sh.get("reason"))))
    detector.strip_polygons(body)
    return body


# =====================================
# FRAME SOURCE (tests replace it with a fake)
# =====================================

class BufferFrameSource:
    """latest() -> (color_bgr, depth_raw_mm, intrinsics, frame_ts) | None"""

    def __init__(self, buffer):
        self.buffer = buffer

    def latest(self):
        fr = self.buffer.get_frame_with_ts()
        if fr is None:
            return None
        color, depth, ts = fr
        return (color, depth, self.buffer.intrinsics, ts)


class ImageSaver:
    """Saves the best crop to <state>/captures/obj_<name>.jpg, keeps the newest N."""

    def __init__(self, state_dir, margin=10, keep=50, quality=90):
        self.dir = os.path.join(state_dir, "captures")
        self.margin, self.keep, self.quality = margin, keep, quality

    def save_full(self, name, color_bgr):
        """Whole colour frame (for YOLO-seg) -> captures/obj_<name>_full.jpg."""
        os.makedirs(self.dir, exist_ok=True)
        fname = "obj_{}_full.jpg".format(name)
        if not cv2.imwrite(os.path.join(self.dir, fname), color_bgr,
                           [cv2.IMWRITE_JPEG_QUALITY, self.quality]):
            raise IOError("could not write " + fname)
        return "captures/" + fname

    def __call__(self, name, color_bgr, bbox):
        os.makedirs(self.dir, exist_ok=True)
        crop = detector.crop_to_bbox(color_bgr, bbox, self.margin)
        fname = "obj_{}.jpg".format(name)
        if not cv2.imwrite(os.path.join(self.dir, fname), crop,
                           [cv2.IMWRITE_JPEG_QUALITY, self.quality]):
            raise IOError("could not write " + fname)
        files = sorted(glob.glob(os.path.join(self.dir, "obj_*.jpg")), key=os.path.getmtime)
        for old in files[:-self.keep]:
            try:
                os.remove(old)
            except OSError:
                pass
        return "captures/" + fname


# =====================================
# AUTO MODE: camera side of the handshake (STATE_CONTRACT.md section 5)
# =====================================

class AutoCoordinator:
    """
    conveyor phase "running" (+ object_id N)  -> reset tracker(N), analyse new frames
    conveyor phase "at_end"  (+ object_id N)  -> freeze the best frame, write
                                                 detect_result{object_id: N} once
    Anything else ends tracking. Unknown / stale conveyor status -> do nothing.
    A result is written only for the object id it tracked (or "not_tracked").
    """

    def __init__(self, frame_source, tracker, detect_writer, settings, clock=time.time,
                 id_source=None, save_image=None, last_detect_id=0, analyze_image=None):
        self.analyze_image = analyze_image   # fn(rel_image_path) -> dict | None (YOLO)
        self.source = frame_source
        self.tracker = tracker
        self.writer = detect_writer
        self.settings = settings
        self.params = detector_params(settings)
        self.seg_params = dict(self.params, **detector.seg_params(settings))
        self.id_source = id_source or (lambda: int(clock()))
        self.save_image = save_image
        self.last_detect_id = int(last_detect_id or 0)
        self.state = "idle"            # idle | detecting | frozen
        self.tracking_id = None
        self.written_object_id = None  # object id of the last auto result written
        self.last_result = None        # {"detect_id", "object_id", "status"}
        self._pending = None           # result body whose write failed (retried)
        self._last_ts = None
        self._frame_n = 0
        self._cam_missing = 0

    def reset(self):
        """Forget tracking (mode switch). Keeps written_object_id."""
        self.state, self.tracking_id = "idle", None
        self.tracker.active = False

    def step(self, conveyor_doc, conveyor_fresh):
        self._flush_pending()
        if not conveyor_fresh or not isinstance(conveyor_doc, dict):
            return
        phase = conveyor_doc.get("phase")
        oid = conveyor_doc.get("object_id")
        if phase == "running" and oid is not None:
            if self.tracking_id != oid:
                self._begin(oid)
            elif self.state == "detecting":
                self._analyse_new_frame()
        elif phase == "at_end" and oid is not None:
            if self.tracking_id == oid and self.state == "detecting":
                self._finish(oid)
            elif self.tracking_id != oid and self.written_object_id != oid:
                self.tracking_id = oid
                self.state = "frozen"
                self._write(None, oid, status="no_object", reason="not_tracked")
        else:
            if self.state == "detecting":      # cycle aborted (fault, blocked...)
                self.reset()
            elif self.state == "frozen" and phase == "idle":
                self.state, self.tracking_id = "idle", None

    # ---- internals ----
    def _begin(self, oid):
        self.tracker.reset(oid)
        self.tracking_id, self.state = oid, "detecting"
        self._last_ts, self._frame_n, self._cam_missing = None, 0, 0

    def _analyse_new_frame(self):
        fr = self.source.latest()
        if fr is None:
            self._cam_missing += 1
            return
        color, depth, intrinsics, ts = fr
        if ts == self._last_ts:
            return
        self._last_ts = ts
        self._frame_n += 1
        if self._frame_n % max(1, int(self.settings.get("segment_every_n", 1))):
            return
        cand = analyze_with_frame(depth, self.params, intrinsics)
        self.tracker.offer(cand, ts, color)

    def _finish(self, oid):
        snap = self.tracker.freeze()
        self.state = "frozen"
        if snap["frames_seen"] == 0 and self._cam_missing > 0:
            self._write(snap, oid, status="cam_off", reason="camera_off")
        else:
            self._write(snap, oid)

    def _write(self, snap, oid, status=None, reason=None):
        detect_id = max(int(self.id_source()), self.last_detect_id + 1)
        self.last_detect_id = detect_id
        image = None
        best = snap["best"] if snap else None
        if best is not None and self.save_image is not None:
            try:
                image = self.save_image(str(oid), best["payload"],
                                        best["candidate"]["meas"]["bbox_px"])
            except Exception as e:
                print("[CAMERA] could not save image: {}".format(e))
        body = detector.make_detection_result(snap, oid, "auto", detect_id, self.params,
                                              status=status, reason=reason, image=image)
        add_yolo_and_height(body, best, image, str(oid), self.save_image, self.analyze_image,
                            self.settings, self.seg_params)
        self.written_object_id = oid
        self.last_result = {"detect_id": detect_id, "object_id": oid, "status": body["status"]}
        self._pending = body
        self._flush_pending()
        print("[CAMERA] object {} -> {} (detect_id {})".format(oid, body["status"], detect_id))

    def _flush_pending(self):
        if self._pending is None:
            return
        try:
            self.writer.write(self._pending)
            self._pending = None
        except OSError as e:
            print("[CAMERA] detect_result write failed (will retry): {}".format(e))


# =====================================
# CONTROLLER (the web layer and main() call only this)
# =====================================

class ControllerError(Exception):
    """Refused request. `code` is the HTTP status the web layer should answer."""

    def __init__(self, msg, code=409):
        super().__init__(msg)
        self.code = code


class CameraController:
    def __init__(self, state_dir, settings=None, mode="manual", buffer=None,
                 thread_factory=None, source=None, clock=time.time):
        self.state_dir = state_dir
        self.settings = settings or load_settings()
        self.mode = mode
        self.buffer = buffer or frame_buffer
        self.source = source or BufferFrameSource(self.buffer)
        self.thread_factory = thread_factory or (lambda: CameraThread(enable_depth=True))
        self.clock = clock
        self.web_port = self.settings.get("web_port")
        self.lock = threading.RLock()
        self.thread = None
        self.want_on = False
        self.error = None
        self._retry_at = 0.0
        self._last_status_write = 0.0

        self.status_writer = state_io.StateWriter("camera_status", "camera.py", state_dir)
        self.detect_writer = state_io.StateWriter("detect_result", "camera.py", state_dir,
                                                  durable=True)
        old = state_io.read_json(os.path.join(state_dir, "detect_result.json"), "detect_result/1")
        last_id = old.get("detect_id", 0) if old else 0
        self.params = detector_params(self.settings)
        self.saver = ImageSaver(state_dir, self.settings["crop_margin_px"],
                                self.settings["max_saved_crops"], self.settings["crop_jpeg_quality"])
        self.tracker = detector.BestTracker()
        self.yolo = yolo_detect.make_from_settings(self.settings, self.state_dir)
        if self.yolo is not None:
            self.yolo.load()          # load now so the first object is not delayed
        self.coordinator = AutoCoordinator(
            self.source, self.tracker, self.detect_writer, self.settings, clock=clock,
            save_image=self.saver, last_detect_id=last_id if isinstance(last_id, int) else 0,
            analyze_image=self.analyze_image)
        self.manual_last = None

    def analyze_image(self, rel_image):
        """YOLO on a saved capture (path relative to the state folder), or None if off."""
        return yolo_detect.analyze_capture(self.yolo, self.state_dir, rel_image)

    # ---- camera on/off ----
    def camera_on(self):
        with self.lock:
            self.want_on = True
            self._retry_at = 0.0
            self._ensure_camera(self.clock())
        return {"ok": True, "msg": "camera starting"}

    def camera_off(self):
        with self.lock:
            self.want_on = False
            thread, self.thread = self.thread, None
            self.error = None
        if thread is not None:
            thread.stop()
        return {"ok": True, "msg": "camera stopped"}

    def _ensure_camera(self, now):
        if not self.want_on:
            return
        if self.thread is not None and self.thread.is_alive():
            return
        if self.thread is not None:          # it died
            msg = self.thread.error or "camera thread ended"
            if msg != self.error:
                print("[CAMERA] {} (retry in {}s)".format(msg, self.settings["retry_s"]))
            self.error = msg
            self.thread = None
            self._retry_at = now + float(self.settings["retry_s"])
        if now >= self._retry_at:
            self.thread = self.thread_factory()
            self.thread.start()

    def camera_state(self):
        if not self.want_on:
            return "off"
        if self.thread is not None and self.thread.is_alive():
            return "on" if self.buffer.camera_active else "starting"
        return "error" if self.error else "starting"

    # ---- mode / crop ----
    def set_mode(self, mode):
        if mode not in ("manual", "auto"):
            raise ControllerError("mode must be 'manual' or 'auto'", 400)
        with self.lock:
            if mode != self.mode:
                self.coordinator.reset()
                self.mode = mode
        if mode == "auto":
            self.camera_on()
        return {"ok": True, "mode": self.mode}

    def set_crop(self, enable):
        globals()["CROP_ENABLE"] = bool(enable)
        return {"ok": True, "crop": bool(enable)}

    # ---- manual detection (web trigger) ----
    def trigger_detect(self):
        if self.mode != "manual":
            raise ControllerError("trigger is refused in auto mode (the conveyor drives it)", 409)
        if self.camera_state() != "on":
            raise ControllerError("camera is not on", 409)
        n = int(self.settings["manual_trigger_frames"])
        deadline = time.monotonic() + float(self.settings["manual_trigger_timeout_s"])
        params = dict(self.params, min_good_frames=1)
        tracker = detector.BestTracker()
        tracker.reset(None)
        last_ts = None
        while tracker.frames_seen < n and time.monotonic() < deadline:
            fr = self.source.latest()
            if fr is None or fr[3] == last_ts:
                time.sleep(0.01)
                continue
            color, depth, intrinsics, ts = fr
            last_ts = ts
            tracker.offer(analyze_with_frame(depth, params, intrinsics), ts, color)
        snap = tracker.freeze()
        with self.lock:
            detect_id = max(int(self.clock()), self.coordinator.last_detect_id + 1)
            self.coordinator.last_detect_id = detect_id
            image = None
            best = snap["best"]
            if best is not None:
                try:
                    image = self.saver("manual_{}".format(detect_id), best["payload"],
                                       best["candidate"]["meas"]["bbox_px"])
                except Exception as e:
                    print("[CAMERA] could not save image: {}".format(e))
            status = "cam_off" if snap["frames_seen"] == 0 else None
            body = detector.make_detection_result(snap, None, "manual", detect_id, params,
                                                  status=status, image=image)
            add_yolo_and_height(body, best, image, "manual_{}".format(detect_id), self.saver,
                                self.analyze_image, self.settings,
                                dict(params, **detector.seg_params(self.settings)))
            self.detect_writer.write(body)
        self.manual_last = {"detect_id": detect_id, "object_id": None, "status": body["status"]}
        return body

    # ---- periodic work (main loop) ----
    def tick(self, conveyor_doc=None, conveyor_fresh=False):
        now = self.clock()
        with self.lock:
            self._ensure_camera(now)
            if self.mode == "auto":
                self.coordinator.step(conveyor_doc, conveyor_fresh)
            if now - self._last_status_write >= float(self.settings["heartbeat_s"]):
                self.write_status()
                self._last_status_write = now

    def tracking_state(self):
        return "manual" if self.mode == "manual" else self.coordinator.state

    def status(self):
        cam_state = self.camera_state()
        tracking = {"state": self.tracking_state(), "object_id": None, "frames_seen": 0,
                    "frames_with_object": 0, "best_area_px": None, "best_height_mm": None}
        if self.mode == "auto" and self.coordinator.tracking_id is not None:
            tracking["object_id"] = self.coordinator.tracking_id
            tracking.update(self.tracker.summary())
        last = self.coordinator.last_result if self.mode == "auto" else self.manual_last
        return {
            "mode": self.mode,
            "camera": {"state": cam_state, "fps": round(self.buffer.fps, 1),
                       "error": self.error if cam_state == "error" else None},
            "tracking": tracking,
            "last_result": last or self.coordinator.last_result or self.manual_last,
            "web": {"port": self.web_port},
        }

    def write_status(self):
        try:
            self.status_writer.write(self.status())
        except OSError as e:
            print("[CAMERA] camera_status write failed: {}".format(e))

    def jpeg(self, kind):
        if kind == "depth":
            return self.buffer.get_depth_jpeg()
        if kind == "combined":
            return self.buffer.get_combined_jpeg()
        return self.buffer.get_color_jpeg()

    def shutdown(self):
        self.camera_off()


# =====================================
# MAIN
# =====================================

def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt()


def main(argv=None):
    parser = argparse.ArgumentParser(description="RealSense camera + web page")
    parser.add_argument("--auto", action="store_true",
                        help="Auto mode: follow state/conveyor_status.json")
    parser.add_argument("--on", action="store_true", help="Manual mode: start with the camera on")
    parser.add_argument("--web-port", type=int, default=None, help="Web page port (default from settings)")
    parser.add_argument("--no-web", action="store_true", help="Do not start the web page")
    parser.add_argument("--state-dir", default=None, help="Shared state folder (see STATE_CONTRACT.md)")
    args = parser.parse_args(argv)

    state_dir = state_io.resolve_state_dir(args.state_dir)
    lock = state_io.acquire_single_instance("camera", state_dir)
    if lock is None:
        print("[ERROR] camera.py is already running (lock held)")
        return 1

    settings = load_settings()
    ctl = CameraController(state_dir, settings, mode="auto" if args.auto else "manual")
    ctl.web_port = args.web_port or settings["web_port"]
    if args.auto or args.on:
        ctl.camera_on()
    if not args.no_web:
        import web_stream
        web_stream.start_server(ctl, port=ctl.web_port)

    try:
        import signal
        signal.signal(signal.SIGTERM, _raise_interrupt)
    except (ImportError, ValueError):
        pass

    conv_path = os.path.join(state_dir, "conveyor_status.json")
    conv_fresh = state_io.Freshness(1.0)
    print("[CAMERA] {} mode, state dir {}".format(ctl.mode, state_dir))
    was_fresh = None
    try:
        while True:
            doc = state_io.read_json(conv_path, "conveyor_status/1")
            conv_fresh.update(doc)
            fresh = conv_fresh.is_fresh()
            if ctl.mode == "auto" and fresh != was_fresh:
                print("[CAMERA] conveyor status is {}".format("fresh" if fresh else "missing/stale"))
            was_fresh = fresh
            ctl.tick(doc, fresh)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        ctl.shutdown()
        print("[CAMERA] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
