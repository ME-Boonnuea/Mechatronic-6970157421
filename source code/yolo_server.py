"""
yolo_server.py
Small HTTP server that holds one YOLO model. It runs INSIDE the Ultralytics
Docker container, so camera.py (on the Raspberry Pi, outside Docker) can ask for
a detection on a saved capture without installing Ultralytics on the Pi.

This is the boundary between the Raspberry Pi control group and the Docker YOLO
group. Only the shared state folder crosses the boundary; requests contain a
relative capture path and responses contain plain JSON detection data.

The container only sees the shared state folder, mounted as --root. Requests
carry a path RELATIVE to that folder (for example captures/obj_5.jpg). Paths
that leave the folder, or that are not .jpg/.jpeg/.png, are refused.

    GET  /health    {"ok": true, "model": "...", "loaded": true}
    POST /predict   {"image": "captures/obj_5.jpg",
                     "annotated": "captures/obj_5_yolo.jpg" | null,
                     "conf": 0.25, "imgsz": 640}
                    -> the dict from yolo_detect.YoloDetector.predict_file,
                       "annotated" is the relative path, or null

It listens on 127.0.0.1 by default. With `docker run --network host` that is the
Pi's own localhost, so nothing else on the network can reach it.

Start it from the container, see README.md "YOLO in Docker":
    python3 /app/yolo_server.py --model /models/best.pt --root /state --port 8765
"""

import argparse
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yolo_detect

ALLOWED_EXT = (".jpg", ".jpeg", ".png")
MAX_BODY = 64 * 1024


def safe_path(root, rel):
    """Absolute path of `rel` inside `root`, or ValueError."""
    if not isinstance(rel, str) or not rel:
        raise ValueError("path must be a non-empty string")
    root_real = os.path.realpath(root)
    p = os.path.realpath(os.path.join(root_real, rel))
    if p == root_real or os.path.commonpath([root_real, p]) != root_real:
        raise ValueError("path is outside the state folder")
    if os.path.splitext(p)[1].lower() not in ALLOWED_EXT:
        raise ValueError("only .jpg/.jpeg/.png files")
    return p


def make_handler(detector, root):
    lock = threading.Lock()          # one inference at a time

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass                      # the detector already logs each call

        def _send(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"ok": detector.model is not None,
                                 "model": detector.model_path,
                                 "loaded": detector.model is not None})
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/predict":
                return self._send(404, {"error": "not found"})
            try:
                n = int(self.headers.get("Content-Length", "0"))
                if not 0 < n <= MAX_BODY:
                    raise ValueError("bad body size")
                req = json.loads(self.rfile.read(n).decode("utf-8"))
                if not isinstance(req, dict):
                    raise ValueError("body must be a JSON object")
                image = safe_path(root, req.get("image"))
                ann = safe_path(root, req["annotated"]) if req.get("annotated") else None
                conf, imgsz = req.get("conf"), req.get("imgsz")
                if conf is not None and not (isinstance(conf, (int, float)) and 0 < conf <= 1):
                    raise ValueError("conf must be in (0, 1]")
                if imgsz is not None and not (isinstance(imgsz, int) and 32 <= imgsz <= 2048):
                    raise ValueError("imgsz must be an integer 32..2048")
            except (ValueError, KeyError, json.JSONDecodeError) as e:
                return self._send(400, {"status": "error", "error": str(e)})
            with lock:
                res = detector.predict_file(image, ann, conf=conf, imgsz=imgsz)
            if res.get("annotated"):
                res["annotated"] = os.path.relpath(res["annotated"], os.path.realpath(root)) \
                    .replace(os.sep, "/")
            self._send(200, res)

    return Handler


def serve(detector, root, host="127.0.0.1", port=8765):
    server = ThreadingHTTPServer((host, port), make_handler(detector, root))
    return server


def main(argv=None):
    p = argparse.ArgumentParser(description="YOLO server for camera.py")
    p.add_argument("--model", required=True, help="path to the .pt model inside the container")
    p.add_argument("--root", default="/state", help="shared state folder inside the container")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=640)
    a = p.parse_args(argv)

    det = yolo_detect.YoloDetector(a.model, a.conf, a.imgsz, annotate=True)
    if not det.load():
        print("[YOLO-SERVER] cannot load the model: {}".format(det.error))
        return 1
    server = serve(det, a.root, a.host, a.port)
    print("[YOLO-SERVER] listening on http://{}:{}  root={}".format(a.host, a.port, a.root))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
