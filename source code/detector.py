"""
detector.py
Pure depth-based object segmentation and measurement.

No dependency on pyrealsense2 or the camera thread: every function takes numpy
arrays and plain numbers, so it can be tested on any PC without the hardware
or the Docker YOLO service.

Pipeline:
    segment_object()  -> depth mask -> morphology -> largest contour -> bbox
    measure_object()  -> pixel extent -> millimetres (pinhole model)
    crop_to_bbox()    -> crop colour / depth image around the bbox

Pinhole model:
    size_mm = pixel_extent * depth_mm / f
    f = fx for horizontal extents, fy for vertical extents.
    depth_mm is the object TOP-SURFACE depth (median of masked depths).
"""

import numpy as np
import cv2

# =====================================
# CONFIGURATION
# =====================================

# Optional classifier hook. Disabled by default; this module has no YOLO runtime
# dependency. In the cell deployment, YOLO inference is performed by the Docker
# service and its polygons are passed back to the pure height functions below.
ENABLE_CLASSIFIER = False

# Morphology kernel size (px) for open (remove speckle) and close (fill holes)
MORPH_KERNEL_PX = 5


# =====================================
# SEGMENTATION
# =====================================

def segment_object(depth_raw, belt_depth_mm, threshold_mm, min_valid_mm, min_area_px):
    """
    Find the largest object blob closer to the camera than the belt.

    depth_raw : 2-D uint16 array, millimetres (0 = invalid)
    Returns None if nothing big enough, else a dict:
        mask            uint8 image (255 inside the blob, filled contour)
        contour         largest contour (Nx1x2 int32)
        bbox            (x, y, w, h) axis-aligned, pixels
        rect            cv2.minAreaRect result ((cx, cy), (w, h), angle)
        area_px         contour area (pixels)
        touches_border  True if bbox touches the image edge (object cut off)
    """
    raw = np.asarray(depth_raw)
    if raw.ndim != 2:
        return None

    obj = (raw > min_valid_mm) & (raw < (belt_depth_mm - threshold_mm))
    # Cheap early exit: not enough candidate pixels, skip morphology/contours
    if np.count_nonzero(obj) < min_area_px:
        return None

    mask = obj.astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_KERNEL_PX, MORPH_KERNEL_PX))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area_px:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    rect = cv2.minAreaRect(contour)

    # Keep only the chosen blob in the mask (drop other blobs)
    blob = np.zeros_like(mask)
    cv2.drawContours(blob, [contour], -1, 255, thickness=cv2.FILLED)

    img_h, img_w = raw.shape[:2]
    touches = (x <= 0) or (y <= 0) or (x + w >= img_w) or (y + h >= img_h)

    return {
        "mask": blob,
        "contour": contour,
        "bbox": (int(x), int(y), int(w), int(h)),
        "rect": rect,
        "area_px": area,
        "touches_border": bool(touches),
    }


# =====================================
# MEASUREMENT
# =====================================

def measure_object(depth_raw, seg, intrinsics, belt_depth_mm):
    """
    Convert a segmentation result to millimetres.

    intrinsics : dict with fx, fy (pixels). Must match the image the depth is in
                 (see camera.py: aligned-to-color frames use COLOR intrinsics).
    Returns dict:
        width_mm, length_mm   axis-aligned bbox (headline values):
                              width  = horizontal image extent (fx)
                              length = vertical image extent   (fy)
        rotated_long_mm, rotated_short_mm   minAreaRect sides (for rotated objects)
        height_mm             belt_depth_mm - mean masked depth
        avg_depth_mm, median_depth_mm, min_depth_mm
        bbox_px               [x, y, w, h]
        depth_samples         number of valid masked pixels
    or None if the blob has no valid depth pixels.
    """
    raw = np.asarray(depth_raw)
    sel = (seg["mask"] > 0) & (raw > 0)
    depths = raw[sel].astype(np.float64)
    if depths.size == 0:
        return None

    median_d = float(np.median(depths))   # top-surface depth for pinhole scaling
    avg_d = float(np.mean(depths))
    min_d = float(np.min(depths))

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    x, y, w, h = seg["bbox"]

    (_, _), (rw, rh), _ = seg["rect"]
    f_mean = (fx + fy) / 2.0
    r_a = rw * median_d / f_mean
    r_b = rh * median_d / f_mean

    return {
        "width_mm": round(w * median_d / fx, 1),
        "length_mm": round(h * median_d / fy, 1),
        "rotated_long_mm": round(max(r_a, r_b), 1),
        "rotated_short_mm": round(min(r_a, r_b), 1),
        "height_mm": round(belt_depth_mm - avg_d, 1),
        "avg_depth_mm": round(avg_d, 1),
        "median_depth_mm": round(median_d, 1),
        "min_depth_mm": round(min_d, 1),
        "bbox_px": [int(x), int(y), int(w), int(h)],
        "depth_samples": int(depths.size),
    }


# =====================================
# CROP
# =====================================

def crop_origin(bbox, margin=10):
    """(x1, y1): top-left corner, in the full image, of crop_to_bbox(image, bbox, margin)."""
    x, y = bbox[0], bbox[1]
    return (max(0, int(x) - margin), max(0, int(y) - margin))


def crop_to_bbox(image, bbox, margin=10):
    """Crop image to (x, y, w, h) plus margin px, clamped to the image."""
    x, y, w, h = bbox
    img_h, img_w = image.shape[:2]
    x1 = max(0, int(x) - margin)
    y1 = max(0, int(y) - margin)
    x2 = min(img_w, int(x) + int(w) + margin)
    y2 = min(img_h, int(y) + int(h) + margin)
    return image[y1:y2, x1:x2].copy()


# =====================================
# PER-FRAME ANALYSIS + BEST-FRAME TRACKING (STATE_CONTRACT.md sections 4.3, 7)
# =====================================

DEFAULT_PARAMS = {
    "belt_depth_mm": 425,        # camera to empty belt
    "object_threshold_mm": 20,   # pixel must be this much closer than the belt
    "min_valid_depth_mm": 50,    # ignore lens reflections
    "min_area_px": 300,          # smallest accepted blob
    "min_good_frames": 3,        # frames without border touch needed for status "ok"
}


def analyze_frame(depth_raw, params, intrinsics):
    """
    Segment + measure one depth frame. Returns None (nothing found, or no
    intrinsics to convert to mm), else a candidate dict:
        seg, meas, area_px, touches_border, bbox
    A candidate that touches the image border is still returned so the caller
    can tell "border_only" from "no_object".
    """
    if not intrinsics:
        return None
    seg = segment_object(depth_raw, params["belt_depth_mm"], params["object_threshold_mm"],
                         params["min_valid_depth_mm"], params["min_area_px"])
    if seg is None:
        return None
    meas = measure_object(depth_raw, seg, intrinsics, params["belt_depth_mm"])
    if meas is None:
        return None
    return {"seg": seg, "meas": meas, "area_px": seg["area_px"],
            "touches_border": seg["touches_border"], "bbox": seg["bbox"]}


class BestTracker:
    """
    Running best detection for ONE object. Pure (no camera, no files).
    Rule: among frames whose bbox does not touch the border keep the one with
    the largest area_px; a tie keeps the earlier frame.

        reset(object_id)            start tracking (drops everything before)
        offer(candidate, ts, extra) one analysed frame (candidate may be None)
        freeze() -> snapshot        end of object; later offers are ignored
    """

    def __init__(self):
        self.active = False
        self.frozen = False
        self.reset(None)
        self.active = False

    def reset(self, object_id):
        self.object_id = object_id
        self.frames_seen = 0
        self.frames_with_object = 0
        self.good_frames = 0
        self.border_frames = 0
        self.best = None            # {"candidate", "frame_ts", "payload"}
        self.frozen = False
        self.active = True
        self._snapshot = None

    def offer(self, candidate, frame_ts, payload=None):
        """Returns True if this frame became the new best."""
        if not self.active or self.frozen:
            return False
        self.frames_seen += 1
        if candidate is None:
            return False
        self.frames_with_object += 1
        if candidate["touches_border"]:
            self.border_frames += 1
            return False
        self.good_frames += 1
        if self.best is None or candidate["area_px"] > self.best["candidate"]["area_px"]:
            self.best = {"candidate": candidate, "frame_ts": frame_ts, "payload": payload}
            return True
        return False

    def freeze(self):
        """Snapshot dict (same object on repeated calls), or None if never reset."""
        if not self.active:
            return None
        if not self.frozen:
            self.frozen = True
            self._snapshot = {
                "object_id": self.object_id,
                "frames_seen": self.frames_seen,
                "frames_with_object": self.frames_with_object,
                "good_frames": self.good_frames,
                "border_frames": self.border_frames,
                "best": self.best,
            }
        return self._snapshot

    def summary(self):
        """Small live view for the status file."""
        m = self.best["candidate"]["meas"] if self.best else None
        return {"frames_seen": self.frames_seen,
                "frames_with_object": self.frames_with_object,
                "best_area_px": self.best["candidate"]["area_px"] if self.best else None,
                "best_height_mm": m["height_mm"] if m else None}


def make_detection_result(snapshot, object_id, source, detect_id, params,
                          status=None, reason=None, image=None):
    """
    Build the detect_result body (section 4.3, without the common header).
    status/reason override the computed values (used for cam_off / not_tracked).
    Only status "ok" allows a pick. along_belt_mm = rotated_long_mm.
    """
    snap = snapshot or {"frames_seen": 0, "frames_with_object": 0, "good_frames": 0,
                        "border_frames": 0, "best": None}
    best = snap["best"]
    if status is None:
        if snap["good_frames"] >= params["min_good_frames"] and best is not None:
            status = "ok"
        elif snap["frames_with_object"] == 0:
            status, reason = "no_object", reason or "nothing_found"
        elif snap["good_frames"] == 0:
            status, reason = "border_only", reason or "all_candidates_touch_border"
        else:
            status, reason = "no_object", reason or "too_few_good_frames"
    ok = status == "ok" and best is not None
    if status == "ok" and not ok:
        status, reason = "no_object", "no_best_frame"

    out_best, classifier = None, None
    if best is not None and status in ("ok", "border_only", "no_object"):
        m = best["candidate"]["meas"]
        out_best = {
            "width_mm": m["width_mm"], "length_mm": m["length_mm"],
            "rotated_long_mm": m["rotated_long_mm"], "rotated_short_mm": m["rotated_short_mm"],
            "height_mm": m["height_mm"], "median_depth_mm": m["median_depth_mm"],
            "bbox_px": m["bbox_px"], "area_px": best["candidate"]["area_px"],
            "depth_samples": m["depth_samples"],
            "touches_border": best["candidate"]["touches_border"],
            "frame_ts": best["frame_ts"],
        }
        if ENABLE_CLASSIFIER and best.get("payload") is not None:
            classifier = classify_crop(crop_to_bbox(best["payload"], m["bbox_px"]))

    return {
        "detect_id": detect_id,
        "source": source,
        "object_id": object_id,
        "status": status,
        "reason": reason,
        "object_height_mm": out_best["height_mm"] if ok else None,
        "height_status": "ok" if ok else "unknown",
        "along_belt_mm": out_best["rotated_long_mm"] if ok else None,
        "along_status": "ok" if ok else "unknown",
        "best": out_best,
        "stats": {"frames_seen": snap["frames_seen"],
                  "frames_with_object": snap["frames_with_object"],
                  "good_frames": snap["good_frames"]},
        "classifier": classifier,
        "image": image,
        "belt_depth_mm": params["belt_depth_mm"],
    }


# =====================================
# SEGMENTATION-BASED HEIGHT (returned YOLO mask + RANSAC belt plane)
#
#   YOLO-seg polygon -> mask -> depth pixels inside the mask (stage 2)
#   -> belt plane by RANSAC on 3D points around the object (stage 3)
#   -> height = distance of the object points above that plane,
#      with outliers rejected by median / MAD (stage 4)
#
# Everything here is pure (numpy / cv2). The YOLO call itself lives in
# yolo_detect.py / yolo_server.py. If anything fails the caller keeps the
# depth-threshold height (refine_result_with_seg sets height_method).
# =====================================

SEG_DEFAULTS = {
    "belt_reference": "fixed",      # "fixed" = flat plane at belt_depth_mm, "fit" = RANSAC plane, "auto" = fit, else fixed
    "plane_band_mm": 80,          # belt pixels must be within this of belt_depth_mm
    "plane_ring_px": 25,          # keep this far away from the object mask
    "plane_iterations": 200,      # RANSAC tries
    "plane_inlier_tol_mm": 3.0,   # point-to-plane distance counted as inlier
    "plane_min_inliers": 500,     # fewer plane inliers -> plane failed
    "plane_max_points": 4000,     # random subsample of belt points
    "mask_erode_px": 3,           # shrink the mask so edge pixels do not mix belt depth in
    "min_object_px": 50,          # smallest number of depth pixels inside the mask
    "height_mad_k": 3.0,          # reject points further than k * sigma from the median
    "seg_min_overlap": 0.3,       # YOLO mask must cover this share of the depth blob
    "seg_fallback": "depth_blob", # no matching YOLO mask: use the depth blob ("depth_blob") or keep the old height ("threshold")
}


def seg_params(settings):
    """The subset of settings used by the segmentation height (defaults filled in)."""
    return {k: settings.get(k, v) for k, v in SEG_DEFAULTS.items()}


def polygon_to_mask(polygon, shape):
    """Boolean mask (h, w) of a polygon [[x, y], ...] in pixel coordinates."""
    h, w = shape[:2]
    m = np.zeros((h, w), np.uint8)
    pts = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    if len(pts) >= 3:
        cv2.fillPoly(m, [np.round(pts).astype(np.int32).reshape(-1, 1, 2)], 1)
    return m > 0


def deproject(depth_raw, ys, xs, intrinsics):
    """Pixels (rows ys, columns xs) with depth in mm -> N x 3 camera points in mm."""
    z = np.asarray(depth_raw)[ys, xs].astype(np.float64)
    x = (xs.astype(np.float64) - float(intrinsics["ppx"])) * z / float(intrinsics["fx"])
    y = (ys.astype(np.float64) - float(intrinsics["ppy"])) * z / float(intrinsics["fy"])
    return np.column_stack([x, y, z])


def fit_plane_ransac(points, iterations=200, tol_mm=3.0, seed=0):
    """
    RANSAC plane fit with a least-squares refit on the inliers.
    Plane: normal . p + d = 0 with |normal| = 1, oriented so that d > 0, i.e.
    the camera (origin) is on the positive side. A point nearer to the camera
    than the plane therefore has a POSITIVE signed distance normal . p + d.
    Returns None (fewer than 3 points / degenerate), else a dict:
        normal (3,), d, inliers, n, inlier_ratio, rms_mm
    """
    pts = np.asarray(points, dtype=np.float64)
    n = len(pts)
    if n < 3:
        return None
    rng = np.random.default_rng(seed)
    best_cnt, best = 0, None
    for _ in range(max(1, int(iterations))):
        a, b, c = pts[rng.choice(n, 3, replace=False)]
        nrm = np.cross(b - a, c - a)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        d = -float(nrm @ a)
        cnt = int(np.count_nonzero(np.abs(pts @ nrm + d) < tol_mm))
        if cnt > best_cnt:
            best_cnt, best = cnt, (nrm, d)
    if best is None:
        return None
    nrm, d = best
    inl = np.abs(pts @ nrm + d) < tol_mm
    for _ in range(2):                                   # refit on the inliers
        if np.count_nonzero(inl) < 3:
            break
        P = pts[inl]
        cen = P.mean(axis=0)
        nrm = np.linalg.svd(P - cen, full_matrices=False)[2][-1]
        d = -float(nrm @ cen)
        inl = np.abs(pts @ nrm + d) < tol_mm
    if d < 0:
        nrm, d = -nrm, -d
    dist = pts[inl] @ nrm + d
    return {"normal": nrm, "d": float(d), "inliers": int(np.count_nonzero(inl)), "n": n,
            "inlier_ratio": float(np.count_nonzero(inl)) / n,
            "rms_mm": float(np.sqrt(np.mean(dist ** 2))) if dist.size else None}


def estimate_belt_plane(depth_raw, object_mask, intrinsics, params):
    """
    Belt plane from the pixels AROUND the object: valid depth, outside the
    object mask (dilated by plane_ring_px) and within plane_band_mm of
    belt_depth_mm. Returns (plane_dict, None) or (None, reason).
    """
    raw = np.asarray(depth_raw)
    ring = int(params["plane_ring_px"])
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * ring + 1, 2 * ring + 1))
    near_obj = cv2.dilate(object_mask.astype(np.uint8), k) > 0
    d32 = raw.astype(np.int32)
    ok = (raw > params["min_valid_depth_mm"]) & ~near_obj & \
         (np.abs(d32 - int(params["belt_depth_mm"])) <= int(params["plane_band_mm"]))
    ys, xs = np.nonzero(ok)
    if len(ys) < int(params["plane_min_inliers"]):
        return None, "too_few_belt_pixels"
    cap = int(params["plane_max_points"])
    if len(ys) > cap:
        pick = np.random.default_rng(1).choice(len(ys), cap, replace=False)
        ys, xs = ys[pick], xs[pick]
    plane = fit_plane_ransac(deproject(raw, ys, xs, intrinsics),
                             params["plane_iterations"], params["plane_inlier_tol_mm"])
    if plane is None:
        return None, "plane_fit_failed"
    if plane["inliers"] < int(params["plane_min_inliers"]) or plane["inlier_ratio"] < 0.5:
        return None, "plane_not_supported"
    return plane, None


def height_from_mask(depth_raw, polygon, intrinsics, params):
    """
    Object height above the fitted belt plane, from the depth pixels inside a
    YOLO mask polygon. `params` = detector_params + seg_params (needs
    belt_depth_mm and min_valid_depth_mm).
    Returns {"ok": False, "reason": ...} or
      {"ok": True, "height_mm", "height_p95_mm", "samples", "rejected", "plane": {...}}
    """
    p = dict(SEG_DEFAULTS)
    p.update(params)
    raw = np.asarray(depth_raw)
    if raw.ndim != 2 or not intrinsics:
        return {"ok": False, "reason": "no_depth_or_intrinsics"}
    obj = polygon_to_mask(polygon, raw.shape)
    if not obj.any():
        return {"ok": False, "reason": "empty_mask"}
    core = obj
    er = int(p["mask_erode_px"])
    if er > 0:
        shrunk = cv2.erode(obj.astype(np.uint8),
                           cv2.getStructuringElement(cv2.MORPH_RECT, (2 * er + 1, 2 * er + 1))) > 0
        if np.count_nonzero(shrunk) >= int(p["min_object_px"]):
            core = shrunk
    valid = core & (raw > p["min_valid_depth_mm"])
    if np.count_nonzero(valid) < int(p["min_object_px"]):
        return {"ok": False, "reason": "too_few_depth_pixels_in_mask"}
    ref = p.get("belt_reference", "fit")
    plane, why, used = None, None, None
    if ref != "fixed":                      # "fit", "auto" (and any unknown value) try the fit
        plane, why = estimate_belt_plane(raw, obj, intrinsics, p)
        used = "fitted" if plane is not None else None
    if plane is None and ref in ("fixed", "auto"):
        # flat reference at belt_depth_mm, perpendicular to the optical axis:
        # signed distance = belt_depth_mm - Z
        plane = {"normal": np.array([0.0, 0.0, -1.0]), "d": float(p["belt_depth_mm"]),
                 "inliers": None, "inlier_ratio": None, "rms_mm": None}
        used = "fixed"
    if plane is None:
        return {"ok": False, "reason": why}
    ys, xs = np.nonzero(valid)
    s = deproject(raw, ys, xs, intrinsics) @ plane["normal"] + plane["d"]
    med = float(np.median(s))
    sigma = max(1.4826 * float(np.median(np.abs(s - med))), 1.0)
    keep = np.abs(s - med) <= float(p["height_mad_k"]) * sigma
    kept = s[keep]
    if kept.size < 20:
        return {"ok": False, "reason": "too_few_inlier_depth_pixels"}
    height = float(np.median(kept))
    if height < 1.0:
        return {"ok": False, "reason": "object_not_above_plane"}
    return {"ok": True, "reason": None,
            "height_mm": round(height, 1),
            "height_p95_mm": round(float(np.percentile(kept, 95)), 1),
            "samples": int(kept.size), "rejected": int(np.count_nonzero(~keep)),
            "plane": {"reference": used,
                      "normal": [round(float(v), 4) for v in plane["normal"]],
                      "d_mm": round(plane["d"], 1), "inliers": plane["inliers"],
                      "inlier_ratio": (round(plane["inlier_ratio"], 3)
                                       if plane["inlier_ratio"] is not None else None),
                      "rms_mm": round(plane["rms_mm"], 2) if plane["rms_mm"] is not None else None}}


def shift_polygons(objects, offset):
    """Copy of YOLO objects with every polygon moved by offset = (dx, dy) pixels
    (YOLO run on a crop returns crop coordinates; adding the crop origin gives full-image ones)."""
    dx, dy = offset
    out = []
    for o in objects or []:
        o = dict(o)
        if o.get("polygon"):
            o["polygon"] = [[x + dx, y + dy] for x, y in o["polygon"]]
        out.append(o)
    return out


def pick_yolo_object(objects, blob_mask, min_overlap=0.3):
    """
    The YOLO object (with a "polygon") whose mask covers most of the depth blob.
    Returns (object, overlap) or (None, reason).
    """
    blob = np.asarray(blob_mask) > 0
    area = max(1, int(np.count_nonzero(blob)))
    best, best_ov, any_poly = None, 0.0, False
    for o in objects or []:
        poly = o.get("polygon")
        if not poly:
            continue
        any_poly = True
        ov = np.count_nonzero(polygon_to_mask(poly, blob.shape) & blob) / area
        if ov > best_ov:
            best, best_ov = o, ov
    if not any_poly:
        return None, "model_has_no_masks"
    if best is None or best_ov < float(min_overlap):
        return None, "no_yolo_mask_on_object"
    return best, best_ov


def blob_polygon(contour):
    """The depth blob's contour (N x 1 x 2) as [[x, y], ...]."""
    return np.asarray(contour).reshape(-1, 2).astype(float).tolist()


def refine_result_with_seg(body, best, params, use_yolo=True, offset=(0, 0), yolo_input="full"):
    """
    Plane height for a finished detection `body` (from make_detection_result).
    `best` is the tracker's best frame; its candidate carries "depth",
    "intrinsics" and the depth "seg" (mask contour).

    `offset` is added to the YOLO polygons: (0, 0) when YOLO ran on the full frame,
    the crop origin (crop_origin) when it ran on the crop. `yolo_input` ("full" or "crop")
    is only recorded in seg_height.

    The object mask comes from, in this order:
      1. the YOLO-seg mask that covers the depth blob (use_yolo, body["yolo"]
         taken on the full frame or on the crop, see offset)
                                          -> height_method "yolo_ransac"
      2. the depth blob itself, when YOLO has no matching mask and
         params["seg_fallback"] == "depth_blob" (default), or when
         use_yolo is False                -> height_method "plane_depth_blob"
    On success and status "ok": object_height_mm is replaced by the plane
    height and the old value is kept in height_threshold_mm. If nothing works
    the depth-threshold height stays (height_method "depth_threshold").
    seg_height records the source, the reason and, when YOLO was skipped, why.
    """
    seg = {"ok": False, "reason": None, "source": None, "label": None, "overlap": None,
           "yolo_reason": None, "yolo_input": yolo_input if use_yolo else None}
    cand = best["candidate"] if best else None
    res = None
    if cand is None:
        seg["reason"] = "no_best_frame"
    elif cand.get("depth") is None or not cand.get("intrinsics"):
        seg["reason"] = "no_depth_frame"
    else:
        if use_yolo:
            yolo = body.get("yolo")
            if not isinstance(yolo, dict) or yolo.get("status") != "ok":
                seg["yolo_reason"] = "yolo_" + str(yolo.get("status") if isinstance(yolo, dict) else "off")
            else:
                obj, ov = pick_yolo_object(shift_polygons(yolo.get("objects"), offset),
                                           cand["seg"]["mask"],
                                           params.get("seg_min_overlap", SEG_DEFAULTS["seg_min_overlap"]))
                if obj is None:
                    seg["yolo_reason"] = ov
                else:
                    r = height_from_mask(cand["depth"], obj["polygon"], cand["intrinsics"], params)
                    if r["ok"]:
                        res = r
                        seg.update(source="yolo_mask", label=obj.get("label"), overlap=round(float(ov), 3))
                    else:
                        seg["yolo_reason"] = "yolo_mask_height_failed:" + str(r["reason"])
        if res is None and (not use_yolo or
                            params.get("seg_fallback", SEG_DEFAULTS["seg_fallback"]) == "depth_blob"):
            r = height_from_mask(cand["depth"], blob_polygon(cand["seg"]["contour"]),
                                 cand["intrinsics"], params)
            if r["ok"]:
                res = r
                seg["source"] = "depth_blob"
            else:
                seg["reason"] = str(r["reason"])
        if res is None and seg["reason"] is None:
            seg["reason"] = seg["yolo_reason"] or "no_mask"
    if res is not None:
        seg.update(res)
    body["seg_height"] = seg
    if seg["ok"] and body.get("status") == "ok":
        body["height_threshold_mm"] = body.get("object_height_mm")
        body["object_height_mm"] = seg["height_mm"]
        body["height_method"] = "yolo_ransac" if seg["source"] == "yolo_mask" else "plane_depth_blob"
    else:
        body["height_method"] = "depth_threshold"
    return body

def strip_polygons(body):
    """Replace the (large) mask polygons in body["yolo"] by their point count."""
    yolo = body.get("yolo")
    if not isinstance(yolo, dict):
        return body
    items = list(yolo.get("objects") or [])
    if isinstance(yolo.get("top"), dict):
        items.append(yolo["top"])
    for o in items:
        poly = o.pop("polygon", None)
        if poly is not None:
            o["polygon_points"] = len(poly)
    return body


# =====================================
# OPTIONAL NEURAL CLASSIFIER HOOK
# =====================================

def classify_crop(crop_bgr):
    """
    Extension hook: return a label/dict for the cropped object, or None.
    Disabled unless ENABLE_CLASSIFIER is True. A nano YOLO (NCNN or TFLite)
    could be loaded here and run on crop_bgr. Not implemented.
    """
    if not ENABLE_CLASSIFIER:
        return None
    return None
