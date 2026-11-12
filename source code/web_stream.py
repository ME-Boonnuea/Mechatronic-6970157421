"""
web_stream.py
The ONE web page of the cell. A library that runs inside the camera.py process
(it does not open the camera itself; it only talks to the CameraController).

    python web_stream.py        same as   python camera.py   (manual mode)

Page: live colour + depth stream, manual camera controls (on / off, auto-manual
mode, crop, trigger detection) and the state of the other programs read from the
shared JSON files (read-only: conveyor, camera, detection result, dobot).

Routes:
    /                 the page
    /stream/<kind>    MJPEG, kind = color | depth | combined
    /snapshot         latest colour frame as JPEG
    /api/camera       POST {"action": "on"|"off"|"mode"|"crop"|"trigger", "value": ...}
    /api/state        JSON: conveyor, camera, detect, dobot (+ age_s of each file)
    /image/latest     best-object crop named in the latest detection result
    /image/latest?kind=yolo   the same capture with the YOLO boxes drawn
"""

import os
import threading
import time

from flask import Flask, Response, jsonify, request, send_file

import state_io

# =====================================
# HTML (plain string, not a Jinja template, so JS braces are safe)
# =====================================

MAIN_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
    <title>Conveyor Cell</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background:#1a1a1a; color:#eee; font-family:Arial,sans-serif;
               margin:0; padding:20px; }
        h1   { color:#4fc3f7; margin-bottom:4px; }
        .sub { color:#aaa; font-size:13px; margin-bottom:20px; }
        .row { display:flex; flex-wrap:wrap; gap:20px; margin-bottom:20px; }
        .box { background:#2a2a2a; border-radius:8px; padding:15px; min-width:260px; }
        .box h2 { margin:0 0 10px; color:#81c784; font-size:13px;
                  text-transform:uppercase; letter-spacing:1px; }
        img.stream { display:block; border-radius:4px; max-width:100%; }
        .kv  { display:flex; justify-content:space-between; gap:16px;
               border-bottom:1px solid #333; padding:4px 0; font-size:14px; }
        .kv .val { color:#fff; font-weight:bold; }
        .badge { display:inline-block; padding:2px 8px; border-radius:10px; font-size:12px; }
        .ok   { background:#2e7d32; }
        .warn { background:#e65100; }
        .off  { background:#555; }
        button { margin:8px 6px 0 0; padding:8px 14px; background:#1565c0; color:#fff;
                 border:none; border-radius:4px; font-size:13px; cursor:pointer; }
        button:hover, button:focus { background:#1976d2; outline:2px solid #fff; }
        #msg { margin-top:10px; font-size:13px; color:#ffb74d; min-height:1.2em; }
    </style>
</head>
<body>
    <h1>Conveyor Cell</h1>
    <div class="sub">Camera, detection result and the state of conveyor and dobot</div>

    <div class="row">
        <div class="box">
            <h2>Color + Depth</h2>
            <img class="stream" src="/stream/combined" width="1280" height="480"
                 alt="Live colour and depth camera stream">
        </div>
    </div>

    <div class="row">
        <div class="box">
            <h2>Camera</h2>
            <div class="kv"><span>Camera</span><span id="cam_state">--</span></div>
            <div class="kv"><span>Mode</span><span id="cam_mode">--</span></div>
            <div class="kv"><span>Tracking</span><span id="trk_state" class="val">--</span></div>
            <div class="kv"><span>FPS</span><span id="fps" class="val">--</span></div>
            <div>
                <button id="btn-on" type="button">Camera ON</button>
                <button id="btn-off" type="button">Camera OFF</button>
                <button id="btn-mode" type="button">Switch mode</button>
                <button id="btn-crop" type="button">Toggle crop</button>
                <button id="btn-trigger" type="button">Trigger detection</button>
                <a href="/snapshot" download="snapshot.jpg"><button type="button">Snapshot</button></a>
            </div>
            <div id="msg" role="status" aria-live="polite"></div>
        </div>

        <div class="box">
            <h2>Detection result</h2>
            <div class="kv"><span>Status</span><span id="d_status">--</span></div>
            <div class="kv"><span>Source</span><span id="d_source" class="val">--</span></div>
            <div class="kv"><span>Object id</span><span id="d_object" class="val">--</span></div>
            <div class="kv"><span>Height</span><span class="val"><span id="d_height">--</span> mm</span></div>
            <div class="kv"><span>Height method</span><span class="val" id="d_method">--</span></div>
            <div class="kv"><span>Along belt</span><span class="val"><span id="d_along">--</span> mm</span></div>
            <div class="kv"><span>Width x Length</span><span class="val" id="d_wl">--</span></div>
            <div class="kv"><span>Good frames</span><span class="val" id="d_frames">--</span></div>
            <div class="kv"><span>Reason</span><span class="val" id="d_reason">--</span></div>
        </div>

        <div class="box">
            <h2>Best crop</h2>
            <img id="crop" class="stream" alt="Best object crop" style="max-width:320px;display:none">
        </div>

        <div class="box">
            <h2>YOLO (on the saved capture)</h2>
            <div class="kv"><span>Status</span><span id="y_status">--</span></div>
            <div class="kv"><span>Top object</span><span class="val" id="y_top">--</span></div>
            <div class="kv"><span>Objects found</span><span class="val" id="y_count">--</span></div>
            <div class="kv"><span>Time</span><span class="val"><span id="y_time">--</span> ms</span></div>
            <div class="kv"><span>Model</span><span class="val" id="y_model">--</span></div>
            <div class="kv"><span>Note</span><span class="val" id="y_error">--</span></div>
            <img id="yolo_img" class="stream" alt="Capture with YOLO detections"
                 style="max-width:320px;display:none;margin-top:8px">
        </div>

        <div class="box">
            <h2>Conveyor</h2>
            <div class="kv"><span>Phase</span><span id="c_phase" class="val">--</span></div>
            <div class="kv"><span>Mode</span><span id="c_mode" class="val">--</span></div>
            <div class="kv"><span>Start / End sensor</span><span id="c_sens" class="val">--</span></div>
            <div class="kv"><span>Motor speed</span><span id="c_speed" class="val">--</span></div>
            <div class="kv"><span>Object id</span><span id="c_obj" class="val">--</span></div>
            <div class="kv"><span>Pick permission</span><span id="c_perm">--</span></div>
        </div>

        <div class="box">
            <h2>Dobot</h2>
            <div class="kv"><span>State</span><span id="b_state" class="val">--</span></div>
            <div class="kv"><span>Mode</span><span id="b_mode">--</span></div>
            <div class="kv"><span>Handled id</span><span id="b_handled" class="val">--</span></div>
            <div class="kv"><span>Outcome</span><span id="b_outcome" class="val">--</span></div>
            <div class="kv"><span>Reason</span><span id="b_reason" class="val">--</span></div>
        </div>
    </div>

    <script>
        var last = {};

        function setText(id, v) {
            document.getElementById(id).textContent = (v === undefined || v === null) ? '--' : v;
        }
        function setBadge(id, ok, yes, no) {
            var el = document.getElementById(id);
            el.textContent = ok ? yes : no;
            el.className = 'badge ' + (ok ? 'ok' : 'warn');
        }
        function say(text) { document.getElementById('msg').textContent = text; }

        function post(action, value) {
            return fetch('/api/camera', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action, value: value})
            }).then(function (r) {
                return r.json().then(function (d) { d.http = r.status; return d; });
            }).then(function (d) {
                say(d.ok ? (d.msg || 'ok') : ('Refused: ' + d.error));
                refresh();
            }).catch(function (e) { say('Request failed'); });
        }

        document.getElementById('btn-on').onclick = function () { post('on'); };
        document.getElementById('btn-off').onclick = function () { post('off'); };
        document.getElementById('btn-mode').onclick = function () {
            post('mode', last.cam_mode === 'auto' ? 'manual' : 'auto');
        };
        document.getElementById('btn-crop').onclick = function () { post('crop', !last.crop); last.crop = !last.crop; };
        document.getElementById('btn-trigger').onclick = function () { say('Detecting...'); post('trigger'); };

        function refresh() {
            fetch('/api/state').then(function (r) { return r.json(); }).then(function (s) {
                var cam = s.camera || {}, c = (cam.camera || {}), t = (cam.tracking || {});
                last.cam_mode = cam.mode;
                setBadge('cam_state', c.state === 'on', String(c.state).toUpperCase(),
                         String(c.state).toUpperCase() + (c.error ? ' (' + c.error + ')' : ''));
                setText('cam_mode', cam.mode);
                setText('trk_state', t.state);
                setText('fps', c.fps);

                var d = s.detect || {}, b = d.best || {}, st = d.stats || {};
                setBadge('d_status', d.status === 'ok', d.status || 'none', d.status || 'none');
                setText('d_source', d.source);
                setText('d_object', d.object_id);
                setText('d_height', d.object_height_mm);
                var sg = d.seg_height || null;
                setText('d_method', d.height_method ? (d.height_method +
                    ((sg && !sg.ok && sg.reason) ? ' (' + sg.reason + ')' : '')) : null);
                setText('d_along', d.along_belt_mm);
                setText('d_wl', (b.width_mm !== undefined) ? (b.width_mm + ' x ' + b.length_mm) : '--');
                setText('d_frames', (st.good_frames !== undefined) ? (st.good_frames + ' / ' + st.frames_seen) : '--');
                setText('d_reason', d.reason);
                var img = document.getElementById('crop');
                if (d.image) {
                    var src = '/image/latest?id=' + encodeURIComponent(d.detect_id);
                    if (img.getAttribute('src') !== src) img.src = src;
                    img.style.display = 'block';
                } else {
                    img.style.display = 'none';
                }

                var y = d.yolo || null;
                if (y) {
                    setBadge('y_status', y.status === 'ok', y.status, y.status);
                    setText('y_top', y.top ? (y.top.label + ' ' + Math.round(y.top.confidence * 100) + '%') : 'none');
                    setText('y_count', y.count);
                    setText('y_time', y.time_ms);
                    setText('y_model', y.model);
                    setText('y_error', y.error);
                } else {
                    var ye = document.getElementById('y_status');
                    ye.textContent = (d.detect_id === undefined) ? '--' : 'off';
                    ye.className = 'badge off';
                    ['y_top', 'y_count', 'y_time', 'y_model', 'y_error'].forEach(function (k) { setText(k, null); });
                }
                var yimg = document.getElementById('yolo_img');
                if (y && y.annotated) {
                    var ysrc = '/image/latest?kind=yolo&id=' + encodeURIComponent(d.detect_id);
                    if (yimg.getAttribute('src') !== ysrc) yimg.src = ysrc;
                    yimg.style.display = 'block';
                } else {
                    yimg.style.display = 'none';
                }

                var cv = s.conveyor || {}, sn = cv.sensors || {}, mo = cv.motor || {};
                setText('c_phase', cv.phase);
                setText('c_mode', cv.mode);
                setText('c_sens', (sn.start !== undefined) ? ((sn.start ? 'ON' : 'off') + ' / ' + (sn.end ? 'ON' : 'off')) : '--');
                setText('c_speed', (mo.speed !== undefined) ? (mo.speed + ' ' + mo.direction) : '--');
                setText('c_obj', cv.object_id);
                setBadge('c_perm', !!cv.pick_permission, 'YES', 'no');

                var db = s.dobot || {};
                setText('b_state', db.state);
                setBadge('b_mode', db.live === false, 'DRY RUN', db.live ? 'LIVE' : '--');
                setText('b_handled', db.handled_id);
                setText('b_outcome', db.outcome);
                setText('b_reason', db.reason);
            }).catch(function () {});
        }

        setInterval(refresh, 1000);
        refresh();
    </script>
</body>
</html>
"""

# =====================================
# MJPEG GENERATOR
# =====================================

def mjpeg_generator(get_frame_fn, fps=30):
    interval = 1.0 / fps
    while True:
        jpeg = get_frame_fn()
        if jpeg is None:
            time.sleep(0.05)
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg + b"\r\n"
        )
        time.sleep(interval)


_STATE_FILES = (("conveyor", "conveyor_status"), ("detect", "detect_result"),
                ("dobot", "dobot_status"))

# =====================================
# APP
# =====================================

def create_app(controller, state_dir):
    """One Flask app. `controller` is a camera.CameraController (or a fake)."""
    app = Flask(__name__)

    @app.route("/")
    def index():
        return Response(MAIN_PAGE, mimetype="text/html")

    @app.route("/stream/<kind>")
    def stream(kind):
        if kind not in ("color", "depth", "combined"):
            return "unknown stream", 404
        return Response(mjpeg_generator(lambda: controller.jpeg(kind)),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/snapshot")
    def snapshot():
        jpeg = controller.jpeg("color")
        if jpeg is None:
            return "No frame", 503
        return Response(jpeg, mimetype="image/jpeg",
                        headers={"Content-Disposition": "attachment; filename=snapshot.jpg"})

    @app.route("/api/state")
    def api_state():
        out = {}
        now = time.time()
        for key, name in _STATE_FILES:
            doc = state_io.read_json(os.path.join(state_dir, name + ".json"), name + "/1")
            if doc is not None:
                ts = doc.get("ts")
                doc["age_s"] = round(now - ts, 1) if isinstance(ts, (int, float)) else None
            out[key] = doc
        out["camera"] = controller.status()
        return jsonify(out)

    @app.route("/api/camera", methods=["POST"])
    def api_camera():
        data = request.get_json(silent=True) or {}
        action = data.get("action")
        value = data.get("value")
        try:
            if action == "on":
                res = controller.camera_on()
            elif action == "off":
                res = controller.camera_off()
            elif action == "mode":
                res = controller.set_mode(value)
            elif action == "crop":
                res = controller.set_crop(bool(value))
            elif action == "trigger":
                body = controller.trigger_detect()
                msg = "detection: " + str(body.get("status"))
                y = body.get("yolo")
                if isinstance(y, dict):
                    top = y.get("top")
                    msg += "; YOLO " + str(y.get("status")) + \
                        (" (" + str(top.get("label")) + ")" if top else "")
                res = {"ok": True, "msg": msg, "detect_id": body.get("detect_id")}
            else:
                return jsonify({"ok": False, "error": "unknown action"}), 400
        except Exception as e:
            code = getattr(e, "code", 500)
            return jsonify({"ok": False, "error": str(e)}), code
        return jsonify(res)

    @app.route("/image/latest")
    def image_latest():
        doc = state_io.read_json(os.path.join(state_dir, "detect_result.json"), "detect_result/1")
        if request.args.get("kind") == "yolo":
            yolo = doc.get("yolo") if doc else None
            rel = yolo.get("annotated") if isinstance(yolo, dict) else None
        else:
            rel = doc.get("image") if doc else None
        if not rel:
            return "No image", 404
        cap = os.path.realpath(os.path.join(state_dir, "captures"))
        path = os.path.realpath(os.path.join(state_dir, rel))
        if os.path.dirname(path) != cap or not os.path.isfile(path):
            return "No image", 404
        resp = send_file(path, mimetype="image/jpeg")
        resp.headers["Cache-Control"] = "no-store"
        return resp

    return app


def start_server(controller, port=5000):
    """Run Flask in a daemon thread inside the camera process."""
    app = create_app(controller, controller.state_dir)
    t = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port, threaded=True,
                               debug=False, use_reloader=False),
        daemon=True, name="flask-web-stream")
    t.start()
    print("[WEB] Page running at http://rc.local:{}".format(port))
    return t


if __name__ == "__main__":
    import sys
    import camera
    sys.exit(camera.main())
