"""
yolo_detect.py
YOLO object detection (Ultralytics) on a saved capture image.

Deployment boundary: this module is the Docker-side inference implementation.
The Raspberry Pi normally uses RemoteYolo below, so camera.py does not load
Ultralytics or the model in the Pi process.

camera.py saves the best crop to <state>/captures/obj_<id>.jpg. This module
reads that file back, runs the YOLO model and returns a small plain dict that
camera.py puts under the "yolo" key of detect_result.json. It also saves an
annotated copy next to the capture (obj_<id>_yolo.jpg).

Rules:
  * It never raises. Any problem (ultralytics missing, model file missing,
    unreadable image, inference error) comes back as {"status": "error", ...}
    or {"status": "unavailable", ...}, so the camera, conveyor and Dobot keep
    running without it.
  * It does NOT decide the pick. The Dobot still uses only the depth result.
  * ultralytics is imported lazily, so this file imports fine on a PC that
    does not have it (tests inject a fake model).

Settings (camera_settings.json):
    yolo_enabled          false by default in code
    yolo_model            path to a .pt model (or a name Ultralytics can download)
    yolo_conf             minimum confidence, 0..1
    yolo_imgsz            inference size in pixels
    yolo_save_annotated   save obj_<id>_yolo.jpg with the boxes drawn

Run alone to test a model on any image:
    python3 yolo_detect.py path/to/image.jpg [--model yolov8n.pt] [--conf 0.25]
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np      # needed by _polygon (seg masks); without it every mask is silently dropped


def _log(msg):
    print("[YOLO] {}".format(msg))


def _polygon(xy, epsilon_px=1.5, max_points=300):
    """Mask outline (N x 2 pixel coordinates) -> short [[x, y], ...] list, or None."""
    try:
        pts = np.asarray(xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 3:
            return None
        approx = cv2.approxPolyDP(pts.reshape(-1, 1, 2), epsilon_px, True).reshape(-1, 2)
        if len(approx) < 3:
            approx = pts
        if len(approx) > max_points:
            approx = approx[np.linspace(0, len(approx) - 1, max_points).astype(int)]
        return [[round(float(x), 1), round(float(y), 1)] for x, y in approx]
    except Exception as e:
        _log("mask outline dropped: {}: {}".format(type(e).__name__, e))
        return None


class YoloDetector:
    """One loaded model. `model_loader(path)` is injectable for tests."""

    def __init__(self, model_path, conf=0.25, imgsz=640, annotate=True, model_loader=None):
        self.model_path = model_path
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.annotate = bool(annotate)
        self._loader = model_loader or self._default_loader
        self.model = None
        self.error = None
        self._tried = False

    @staticmethod
    def _default_loader(path):
        from ultralytics import YOLO          # lazy: only needed when enabled
        return YOLO(path)

    def load(self):
        """Load the model once. Returns True when ready. Never raises."""
        if self.model is not None:
            return True
        if self._tried:
            return False
        self._tried = True
        try:
            t0 = time.monotonic()
            self.model = self._loader(self.model_path)
            _log("model {} loaded in {:.1f}s".format(self.model_path, time.monotonic() - t0))
            return True
        except Exception as e:                # ImportError, missing file, bad weights ...
            self.error = "{}: {}".format(type(e).__name__, e)
            _log("model not available: {}".format(self.error))
            return False

    def predict_file(self, image_path, annotated_path=None, conf=None, imgsz=None):
        """
        Read image_path, run YOLO, return a dict:
          status        "ok" (at least one object) | "no_object" | "error" | "unavailable"
          model         model path/name
          count         number of objects
          top           the most confident object or None
          objects       list of {"label", "class_id", "confidence", "bbox_px": [x1,y1,x2,y2]}
          time_ms       inference time
          annotated     annotated_path if it was written, else None
          error         text when status is error/unavailable
        """
        out = {"status": "error", "model": self.model_path, "count": 0, "top": None,
               "objects": [], "time_ms": None, "annotated": None, "error": None}
        if not self.load():
            out["status"], out["error"] = "unavailable", self.error
            return out
        try:
            if not os.path.isfile(image_path):
                out["error"] = ("image file not found: {} (if YOLO runs in Docker, the state "
                                "folder must be mounted at the server's --root)".format(image_path))
                return out
            img = cv2.imread(image_path)
            if img is None:
                out["error"] = "file exists but cannot be decoded as an image: {}".format(
                    image_path)
                return out
            t0 = time.monotonic()
            results = self.model.predict(img, conf=self.conf if conf is None else conf,
                                         imgsz=self.imgsz if imgsz is None else imgsz,
                                         verbose=False)
            out["time_ms"] = round((time.monotonic() - t0) * 1000.0, 1)
            r = results[0]
            names = getattr(r, "names", None) or {}
            xyxy = r.boxes.xyxy.tolist()
            confs = r.boxes.conf.tolist()
            clss = r.boxes.cls.tolist()
            masks = getattr(r, "masks", None)                  # only a -seg model has masks
            polys = getattr(masks, "xy", None) if masks is not None else None
            objs = []
            for i, (b, c, k) in enumerate(zip(xyxy, confs, clss)):
                k = int(k)
                o = {"label": str(names.get(k, k)), "class_id": k,
                     "confidence": round(float(c), 3),
                     "bbox_px": [round(float(v), 1) for v in b]}
                poly = _polygon(polys[i]) if polys is not None and i < len(polys) else None
                if poly:
                    o["polygon"] = poly         # mask outline in image pixels (stage 1)
                objs.append(o)
            objs.sort(key=lambda o: o["confidence"], reverse=True)
            out["objects"], out["count"] = objs, len(objs)
            out["top"] = objs[0] if objs else None
            out["status"] = "ok" if objs else "no_object"
            out["error"] = None
            if self.annotate and annotated_path:
                try:
                    if cv2.imwrite(annotated_path, r.plot()):
                        out["annotated"] = annotated_path
                except Exception as e:
                    _log("could not save annotated image: {}".format(e))
        except Exception as e:
            out["status"], out["error"] = "error", "{}: {}".format(type(e).__name__, e)
            _log("inference failed: {}".format(out["error"]))
        return out


class RemoteYolo:
    """
    Same interface as YoloDetector, but the model runs in another process,
    for example inside the Ultralytics Docker container (yolo_server.py).
    Only paths relative to the shared state folder cross the connection, so the
    container needs the state folder mounted (docker -v <state>:/state).
    `transport(url, payload, timeout) -> dict` is injectable for tests.
    """

    def __init__(self, url, state_dir, conf=0.25, imgsz=640, annotate=True,
                 timeout_s=2.0, transport=None):
        self.url = url.rstrip("/")
        self.state_dir = state_dir
        self.conf, self.imgsz, self.annotate = float(conf), int(imgsz), bool(annotate)
        self.timeout_s = float(timeout_s)
        self.model_path = self.url
        self.error = None
        self._transport = transport or self._http

    @staticmethod
    def _http(url, payload, timeout):
        import urllib.request
        if payload is None:
            req = urllib.request.Request(url)
        else:
            req = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"),
                                         headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))

    def load(self):
        """Health check of the server. Returns True when it answers. Never raises."""
        try:
            h = self._transport(self.url + "/health", None, self.timeout_s)
            ok = bool(h.get("ok"))
            self.error = None if ok else "server not ready"
            if ok:
                self.model_path = h.get("model") or self.model_path
                _log("server {} ready, model {}".format(self.url, self.model_path))
            else:
                _log("server {} answered but is not ready".format(self.url))
            return ok
        except Exception as e:
            self.error = "{}: {}".format(type(e).__name__, e)
            _log("server {} not reachable yet: {} (will retry for every object)".format(
                self.url, self.error))
            return False

    def predict_file(self, image_path, annotated_path=None, conf=None, imgsz=None):
        out = {"status": "error", "model": self.model_path, "count": 0, "top": None,
               "objects": [], "time_ms": None, "annotated": None, "error": None}
        rel = self._rel(image_path)
        if rel is None:
            out["error"] = "image is not inside the state folder: {}".format(image_path)
            return out
        payload = {"image": rel,
                   "annotated": self._rel(annotated_path) if annotated_path else None,
                   "conf": self.conf if conf is None else conf,
                   "imgsz": self.imgsz if imgsz is None else imgsz}
        try:
            res = self._transport(self.url + "/predict", payload, self.timeout_s)
        except Exception as e:                    # refused, timeout, bad JSON, HTTP error
            out["status"] = "unavailable"
            out["error"] = "{}: {}".format(type(e).__name__, e)
            _log("server call failed: {}".format(out["error"]))
            return out
        if not isinstance(res, dict) or "status" not in res:
            out["error"] = "bad reply from server"
            return out
        out.update({k: res[k] for k in ("status", "model", "count", "top", "objects",
                                        "time_ms", "error") if k in res})
        out["annotated"] = annotated_path if res.get("annotated") else None
        return out

    def _rel(self, path):
        rel = os.path.relpath(path, self.state_dir).replace(os.sep, "/")
        return None if rel.startswith("..") or os.path.isabs(rel) else rel


def make_from_settings(settings, state_dir=None):
    """
    Return a detector, or None when yolo_enabled is false.
    yolo_mode "local"  -> YoloDetector (ultralytics imported in this process)
    yolo_mode "server" -> RemoteYolo (yolo_server.py, e.g. in the Docker container)
    """
    if not settings.get("yolo_enabled", False):
        return None
    if settings.get("yolo_mode", "local") == "server":
        if state_dir is None:
            raise ValueError("server mode needs the state folder")
        return RemoteYolo(settings.get("yolo_url", "http://127.0.0.1:8765"), state_dir,
                          settings.get("yolo_conf", 0.25), settings.get("yolo_imgsz", 640),
                          settings.get("yolo_save_annotated", True),
                          settings.get("yolo_timeout_s", 2.0))
    return YoloDetector(settings.get("yolo_model", "yolov8n.pt"),
                        settings.get("yolo_conf", 0.25),
                        settings.get("yolo_imgsz", 640),
                        settings.get("yolo_save_annotated", True))


def analyze_capture(detector, state_dir, rel_image):
    """
    rel_image is the path camera.py wrote in detect_result["image"],
    e.g. "captures/obj_17.jpg" (relative to the state folder).
    Returns the dict for detect_result["yolo"], or None when YOLO is off or
    there is no image. Never raises.
    """
    if detector is None or not rel_image:
        return None
    src = os.path.join(state_dir, rel_image)
    base, ext = os.path.splitext(rel_image)
    ann_rel = base + "_yolo" + ext
    res = detector.predict_file(src, os.path.join(state_dir, ann_rel) if detector.annotate else None)
    if res.get("annotated"):
        res["annotated"] = ann_rel
    _log("{} -> {} object(s){}".format(
        rel_image, res["count"],
        ", top {} {:.2f}".format(res["top"]["label"], res["top"]["confidence"]) if res["top"] else ""))
    return res


def main(argv=None):
    p = argparse.ArgumentParser(description="Run YOLO on one image file")
    p.add_argument("image")
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--save", default=None, help="write the annotated image here")
    a = p.parse_args(argv)
    det = YoloDetector(a.model, a.conf, a.imgsz, annotate=bool(a.save))
    res = det.predict_file(a.image, a.save)
    print(json.dumps(res, indent=2))
    return 0 if res["status"] in ("ok", "no_object") else 1


if __name__ == "__main__":
    sys.exit(main())
