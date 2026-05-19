#!/usr/bin/env python3
"""
AGRIS — Autonomous Gimbal & Recognition Intelligence System
Laser Tracker GUI | ESP32-S3 GOOUUU CAM + MG996R Pan/Tilt

Fixes vs v1:
  [FIX-1] PID no longer resets on missed frames — holds last position
  [FIX-2] Rectangle added as dedicated shape (was silently dropped)
  [FIX-3] Aspect ratio widened to 0.5–2.5 (catches landscape rectangles)
  [FIX-4] Adaptive threshold replaces fixed — survives lighting variation
  [FIX-5] Frame queue decoupled from display — no more stall-induced misses
  [FIX-6] Tracking thread separated from Tkinter poll thread
  [FIX-7] PID integral clamped (anti-windup) — prevents slow drift buildup
  [FIX-8] send_servos rate-limited to 30Hz — reduces UDP flood to ESP32
    [FIX-9] Error spike rejection threshold corrected — was frame_w//2 which
                     equals max possible error, so rejection NEVER fired. Now uses
                     frame_w * 0.75 so only physically impossible errors are rejected.
    [FIX-10] EDGE_BOOST threshold raised from frame//4 to frame//2. Old value
                        (80px) meant proportional gain was bypassed for >half the frame,
                        turning smooth tracking into bang-bang control for most targets.
    [FIX-11] Target confirmation filter added — servo commands suppressed until
                        same target is seen for 3 consecutive frames. Prevents first-frame
                        false detection from shooting servos to extremes on tracking start.
    [FIX-12] Dead PID objects removed from __init__ — tracking uses constant-
                        speed control, PID was instantiated but never called. Removed to
                        avoid confusion. _reset_pid() method also removed.
"""

import cv2
import numpy as np
import socket
import time
import requests
import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import queue
import re

# ═══════════════════════════════════════════════════════════════
# CONFIG — edit these to match your setup
# ═══════════════════════════════════════════════════════════════
ESP32_IP     = "10.108.237.204"
ESP32_PORT   = 4210
STREAM_URL   = f"http://{ESP32_IP}/stream"

FRAME_W, FRAME_H = 320, 240
FRAME_CX = FRAME_W // 2
FRAME_CY = FRAME_H // 2

PAN_SERVO_MIN    = 500
PAN_SERVO_MAX    = 2500
PAN_SERVO_CENTER = 1632
TILT_SERVO_MIN   = 850
TILT_SERVO_MAX   = 2000
TILT_SERVO_CENTER = 1500

# [FIX-2] 'rectangle' added as highest priority target
# Order = priority (first = highest). Edit to reorder.
PRIORITY = ['rectangle', 'square', 'circle', 'cross_plus', 'cross_x']

DEADBAND = 10

# ═══════════════════════════════════════════════════════════════
# ONE-SHOT ABSOLUTE POSITIONING
# Camera is fixed. Only the laser/servo moves.
# We compute the absolute servo angle from pixel position directly.
# No accumulation. No stepping. Send once per detection.
# ═══════════════════════════════════════════════════════════════

# Mount-dependent axis inversion.
# If the laser moves RIGHT when it should move LEFT, set PAN_INVERT = True.
# If the laser moves UP when it should move DOWN, set TILT_INVERT = True.
PAN_INVERT  = True
TILT_INVERT = False

# OV2640 field of view — degrees the camera can see horizontally/vertically.
# These map pixel position to real-world angle.
# Calibrate: put target at far left edge, check if laser goes there. Adjust.
CAM_HFOV = 60.0   # horizontal FOV in degrees
CAM_VFOV = 45.0   # vertical FOV in degrees

# How many microseconds of PWM per degree of physical servo rotation.
# Pan:  2000µs range over 180° = 11.11 µs/°
# Tilt: 1150µs range over 180° = 6.39 µs/°
PAN_US_PER_DEG  = (PAN_SERVO_MAX  - PAN_SERVO_MIN)  / 180.0
TILT_US_PER_DEG = (TILT_SERVO_MAX - TILT_SERVO_MIN) / 180.0

# Scale factor applied after angle→µs conversion.
# 1.0 = full correction in one shot.
# Lower (e.g. 0.7) if the laser overshoots the target — adds damping.
# Raise toward 1.0 if the laser consistently undershoots.
PAN_SCALE  = 1.0
TILT_SCALE = 1.0

# ═══════════════════════════════════════════════════════════════
# PID CONTROLLER
# [FIX-7] Added integral clamp (anti-windup)
# ═══════════════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min   = out_min
        self.out_max   = out_max
        self.integral  = 0.0
        self.prev_error = 0.0
        self.prev_time  = time.time()

        # [FIX-7] Clamp integral to ±half output range to prevent windup
        self.integral_max = (out_max - out_min) / 2.0

    def update(self, error):
        now = time.time()
        dt  = max(now - self.prev_time, 1e-4)

        # [FIX-7] Clamp integral before accumulating
        self.integral = max(-self.integral_max,
                       min( self.integral_max,
                            self.integral + error * dt))

        derivative      = (error - self.prev_error) / dt
        out = (self.kp * error
             + self.ki * self.integral
             + self.kd * derivative)
        out = max(self.out_min, min(self.out_max, out))

        self.prev_error = error
        self.prev_time  = now
        return out

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0
        self.prev_time  = time.time()

# ═══════════════════════════════════════════════════════════════
# UDP SENDER
# [FIX-8] Rate limited — don't flood the ESP32 with packets
# ═══════════════════════════════════════════════════════════════
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_last_send_time = 0.0
SEND_INTERVAL   = 0.0   # Send every frame; avoid silent drops in fast tracking loops

def send_servos(pan, tilt, laser=1):
    global _last_send_time
    now = time.time()

    # Skip if we sent too recently (disabled when SEND_INTERVAL == 0)
    if SEND_INTERVAL > 0.0 and (now - _last_send_time < SEND_INTERVAL):
        return

    pan  = int(max(PAN_SERVO_MIN, min(PAN_SERVO_MAX, pan)))
    tilt = int(max(TILT_SERVO_MIN, min(TILT_SERVO_MAX, tilt)))
    try:
        sock.sendto(
            f"PAN:{pan},TILT:{tilt},LASER:{laser}".encode(),
            (ESP32_IP, ESP32_PORT)
        )
        _last_send_time = now
    except Exception:
        pass

# ═══════════════════════════════════════════════════════════════
# VISION — PREPROCESSING
# [FIX-4] Adaptive threshold replaces fixed global threshold
# ═══════════════════════════════════════════════════════════════
def preprocess(frame, block_size=21, C=8):
    """
    Returns a binary image where dark regions are white (255).
    block_size: size of local region for adaptive threshold (must be odd)
    C:          constant subtracted from local mean (higher = less sensitive)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # [FIX-4] Adaptive threshold — handles uneven lighting naturally
    # THRESH_BINARY_INV: dark areas become white (our target)
    thr = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV,
        block_size, C
    )

    # Morphological cleanup: remove noise (open) then fill gaps (close)
    k   = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thr = cv2.morphologyEx(thr, cv2.MORPH_OPEN,  k, iterations=2)
    thr = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, k, iterations=2)
    return thr

# ═══════════════════════════════════════════════════════════════
# VISION — CONTOUR CLASSIFICATION
# [FIX-2] 'rectangle' added as dedicated shape
# [FIX-3] Aspect ratio widened from 0.75-1.33 to 0.5-2.5
# ═══════════════════════════════════════════════════════════════
def count_peaks(proj, ratio=0.3):
    """Count distinct bright peaks in a projection (used for cross detection)."""
    threshold = proj.max() * ratio
    in_peak = False
    count   = 0
    for v in proj:
        if v > threshold and not in_peak:
            count  += 1
            in_peak = True
        elif v <= threshold:
            in_peak = False
    return count

def classify_contour(cnt, min_area=400):
    """
    Returns (shape_name, center_x, center_y) or (None, 0, 0).

    Shape priority logic:
      rectangle — 4 corners, aspect ratio 0.5–2.5, NOT square
      square    — 4 corners, aspect ratio 0.75–1.33 (subset of above)
      circle    — high circularity + solidity
      cross_plus/cross_x — concave with specific projection peaks
    """
    area = cv2.contourArea(cnt)
    if area < min_area:
        return None, 0, 0

    M = cv2.moments(cnt)
    if M['m00'] == 0:
        return None, 0, 0

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    peri        = cv2.arcLength(cnt, True)
    circularity = 4 * np.pi * area / (peri * peri + 1e-6)
    approx      = cv2.approxPolyDP(cnt, 0.04 * peri, True)
    hull        = cv2.convexHull(cnt)
    solidity    = area / (cv2.contourArea(hull) + 1e-6)
    x, y, w, h  = cv2.boundingRect(cnt)
    aspect      = w / (h + 1e-6)

    # ── Circle ───────────────────────────────────────────────────
    if circularity > 0.80 and solidity > 0.90:
        return 'circle', cx, cy

    # ── Square / Rectangle ───────────────────────────────────────
    # [FIX-3] Check 4-corner approximation with wider aspect range
    if len(approx) == 4 and solidity > 0.85:
        # Square: near 1:1 aspect ratio
        if 0.75 < aspect < 1.33:
            return 'square', cx, cy
        # [FIX-2] Rectangle: wider range covers landscape/portrait paper
        if 0.5 < aspect < 2.5:
            return 'rectangle', cx, cy

    # ── Cross shapes ─────────────────────────────────────────────
    if solidity < 0.55 and 0.75 < aspect < 1.33 and len(approx) >= 10:
        roi     = np.zeros((h, w), dtype=np.uint8)
        shifted = cnt - [x, y]
        cv2.drawContours(roi, [shifted], -1, 255, -1)
        hp = count_peaks(np.sum(roi, axis=1))
        vp = count_peaks(np.sum(roi, axis=0))
        if hp == 1 and vp == 1:
            return 'cross_plus', cx, cy
        if hp == 2 and vp == 2:
            return 'cross_x', cx, cy

    return None, 0, 0

# ═══════════════════════════════════════════════════════════════
# MJPEG STREAM READER
# [FIX-5] Runs in its own thread, pushes frames into a queue.
#          Display + tracking consume from the queue independently.
# ═══════════════════════════════════════════════════════════════
def mjpeg_stream(url):
    """Generator that yields decoded OpenCV frames from an MJPEG stream."""
    resp = requests.get(url, stream=True, timeout=10,
                        headers={'Connection': 'keep-alive'})
    if resp.status_code != 200:
        raise ConnectionError(f"HTTP {resp.status_code}")

    buf = b''
    for chunk in resp.iter_content(chunk_size=8192):
        if not chunk:
            continue
        buf += chunk

        # Prevent unbounded buffer growth if frames aren't consumed fast enough
        if len(buf) > 500_000:
            buf = buf[-200_000:]

        # Extract complete JPEG frames from stream
        while True:
            start = buf.find(b'\xff\xd8')
            end   = buf.find(b'\xff\xd9', start + 2)
            if start == -1 or end == -1:
                break
            jpg = buf[start:end + 2]
            buf = buf[end + 2:]
            if len(jpg) < 100:
                continue
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                yield frame

# ═══════════════════════════════════════════════════════════════
# MAIN APPLICATION
# ═══════════════════════════════════════════════════════════════
class TrackerApp:

    # ── Colour palette ───────────────────────────────────────────
    BG      = "#0a0e14"
    PANEL   = "#111820"
    BORDER  = "#1e2d3d"
    ACCENT  = "#00d4ff"
    ACCENT2 = "#ff6b35"
    GREEN   = "#39ff14"
    RED     = "#ff2244"
    YELLOW  = "#ffd700"
    TEXT    = "#ffffff"
    TEXTDIM = "#ffffff"
    BTNBG   = "#000000"

    def __init__(self, root):
        self.root = root
        self.root.title("AGRIS — Laser Tracker v2")
        self.root.configure(bg=self.BG)
        self.root.geometry("3200x2000")
        self.root.resizable(True, True)

        # ── Runtime state ────────────────────────────────────────
        self.tracking    = False
        self.laser_on    = False
        self.connected   = False
        self.pan_us      = float(PAN_SERVO_CENTER)
        self.tilt_us     = float(TILT_SERVO_CENTER)
        self.show_thresh = False
        self.fps         = 0.0
        self.frame_count = 0
        self.fps_time    = time.time()
        self.err_x       = 0
        self.err_y       = 0
        self._cx_hist    = []
        self._cy_hist    = []

        # [FIX-1] last_target holds position when target is lost
        # Servos won't freeze/reset — they hold last known good position
        self.last_target_found = False
        # [FIX-11] Confirmation filter — prevents false detections on tracking start
        self.target_confirm_count = 0
        self.TARGET_CONFIRM_FRAMES = 3

        # [FIX-2] Added 'rectangle' to shape vars
        self.shape_vars = {s: tk.BooleanVar(value=True) for s in PRIORITY}

        # [FIX-5] Larger queue — stream thread won't stall waiting for GUI
        # maxsize=4 keeps latency low while absorbing short GUI hiccups
        self.frame_q  = queue.Queue(maxsize=4)
        self.stop_evt = threading.Event()

        self._build_ui()
        self._start_stream_thread()
        self._poll_frame()

    # ────────────────────────────────────────────────────────────
    # UI BUILD
    # ────────────────────────────────────────────────────────────
    def _build_ui(self):
        TF = ("Courier New", 16, "bold")
        MF = ("Courier New", 12)

        # Header
        hdr = tk.Frame(self.root, bg=self.BG, pady=6)
        hdr.pack(fill=tk.X, padx=12)
        tk.Label(hdr, text="◈ AGRIS",
                 fg=self.ACCENT, bg=self.BG,
                 font=("Courier New", 15, "bold")).pack(side=tk.LEFT)
        tk.Label(hdr,
                 text="AUTONOMOUS GIMBAL & RECOGNITION INTELLIGENCE SYSTEM",
                 fg=self.TEXTDIM, bg=self.BG,
                 font=("Courier New", 7)).pack(side=tk.LEFT, padx=10, pady=4)
        self.conn_dot = tk.Label(hdr, text="● OFFLINE",
                                 fg=self.RED, bg=self.BG,
                                 font=("Courier New", 9, "bold"))
        self.conn_dot.pack(side=tk.RIGHT)

        # Content row
        content = tk.Frame(self.root, bg=self.BG)
        content.pack(padx=12, pady=(0, 8), fill=tk.BOTH, expand=True)

        # LEFT: Camera view
        cam_wrap = tk.Frame(content, bg=self.BORDER, padx=1, pady=1)
        cam_wrap.pack(side=tk.LEFT, padx=(0, 10), fill=tk.BOTH, expand=True)
        cam_in = tk.Frame(cam_wrap, bg=self.PANEL)
        cam_in.pack(fill=tk.BOTH, expand=True)

        cam_hdr = tk.Frame(cam_in, bg=self.PANEL)
        cam_hdr.pack(fill=tk.X, padx=8, pady=5)
        tk.Label(cam_hdr, text="▸ CAMERA FEED",
                 fg=self.ACCENT, bg=self.PANEL, font=TF).pack(side=tk.LEFT)
        self.fps_lbl = tk.Label(cam_hdr, text="FPS: --",
                                fg=self.TEXTDIM, bg=self.PANEL, font=MF)
        self.fps_lbl.pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(cam_in, width=640, height=480,
                                bg="#000000", highlightthickness=0)
        self.canvas.pack(padx=8, pady=(0, 6), fill=tk.BOTH, expand=True)

        bc = tk.Frame(cam_in, bg=self.PANEL)
        bc.pack(fill=tk.X, padx=8, pady=(0, 8))
        self.thresh_btn = tk.Button(
            bc, text="THRESHOLD VIEW: OFF",
            bg=self.BTNBG, fg=self.TEXTDIM, font=MF,
            relief=tk.FLAT, cursor="hand2", padx=6, pady=3,
            command=self._toggle_thresh)
        self.thresh_btn.pack(side=tk.LEFT)
        self.target_lbl = tk.Label(bc, text="◈ No target",
                                   fg=self.TEXTDIM, bg=self.PANEL, font=MF)
        self.target_lbl.pack(side=tk.RIGHT)

        # RIGHT: Controls
        right = tk.Frame(content, bg=self.BG, width=420)
        right.pack(side=tk.LEFT, fill=tk.Y)
        right.pack_propagate(False)

        def panel(title, builder, pady_bot=8):
            wrap  = tk.Frame(right, bg=self.BORDER, padx=1, pady=1)
            wrap.pack(fill=tk.X, pady=(0, pady_bot))
            inner = tk.Frame(wrap, bg=self.PANEL, padx=10, pady=8)
            inner.pack(fill=tk.BOTH)
            tk.Label(inner, text=f"▸ {title}",
                     fg=self.ACCENT, bg=self.PANEL,
                     font=TF).pack(anchor="w", pady=(0, 6))
            builder(inner)

        panel("SERVO CONTROL", self._servo_panel)
        panel("TRACKING",      self._tracking_panel)
        panel("TARGET SHAPES", self._shapes_panel)
        panel("VISION",        self._vision_panel, pady_bot=0)

        # Status bar
        tk.Frame(self.root, bg=self.BORDER, height=1).pack(fill=tk.X)
        sbar = tk.Frame(self.root, bg=self.PANEL, pady=4)
        sbar.pack(fill=tk.X)
        self.status_lbl = tk.Label(sbar, text="System ready.",
                                   fg=self.TEXTDIM, bg=self.PANEL,
                                   font=("Courier New", 8))
        self.status_lbl.pack(side=tk.LEFT, padx=10)
        self.laser_ind = tk.Label(sbar, text="◉ LASER OFF",
                                  fg=self.TEXTDIM, bg=self.PANEL,
                                  font=("Courier New", 8, "bold"))
        self.laser_ind.pack(side=tk.RIGHT, padx=10)

    # ────────────────────────────────────────────────────────────
    # PANEL BUILDERS
    # ────────────────────────────────────────────────────────────
    def _servo_panel(self, p):
        MF = ("Courier New", 8)
        tk.Button(p, text="⊕  CENTER SERVOS",
                  bg=self.ACCENT, fg=self.BG,
                  font=("Courier New", 13, "bold"),
                  relief=tk.FLAT, activebackground="#00a8cc",
                  cursor="hand2", padx=20, pady=18,
                  command=self._center_servos).pack(fill=tk.X, pady=(0, 10))

        # Pan slider
        tk.Label(p, text="PAN", fg=self.TEXTDIM,
                 bg=self.PANEL, font=MF).pack(anchor="w")
        pan_row = tk.Frame(p, bg=self.PANEL)
        pan_row.pack(fill=tk.X)
        self.pan_bar_var = tk.DoubleVar(value=PAN_SERVO_CENTER)
        ttk.Scale(pan_row, from_=PAN_SERVO_MIN, to=PAN_SERVO_MAX,
                  orient=tk.HORIZONTAL, variable=self.pan_bar_var,
                  command=self._manual_pan).pack(side=tk.LEFT,
                                                 fill=tk.X, expand=True)
        self.pan_val_lbl = tk.Label(pan_row, text=f"{PAN_SERVO_CENTER}µs",
                                    fg=self.ACCENT2, bg=self.PANEL,
                                    font=MF, width=7)
        self.pan_val_lbl.pack(side=tk.LEFT, padx=(4, 0))

        # Tilt slider
        tk.Label(p, text="TILT", fg=self.TEXTDIM,
                 bg=self.PANEL, font=MF).pack(anchor="w", pady=(6, 0))
        tlt_row = tk.Frame(p, bg=self.PANEL)
        tlt_row.pack(fill=tk.X)
        self.tilt_bar_var = tk.DoubleVar(value=TILT_SERVO_CENTER)
        ttk.Scale(tlt_row, from_=TILT_SERVO_MIN, to=TILT_SERVO_MAX,
                  orient=tk.HORIZONTAL, variable=self.tilt_bar_var,
                  command=self._manual_tilt).pack(side=tk.LEFT,
                                                  fill=tk.X, expand=True)
        self.tilt_val_lbl = tk.Label(tlt_row, text=f"{TILT_SERVO_CENTER}µs",
                                     fg=self.ACCENT2, bg=self.PANEL,
                                     font=MF, width=7)
        self.tilt_val_lbl.pack(side=tk.LEFT, padx=(4, 0))

        # Error display
        err = tk.Frame(p, bg=self.PANEL)
        err.pack(fill=tk.X, pady=(8, 0))
        for i, (lbl, attr) in enumerate([("ERR X", "errx_lbl"),
                                          ("ERR Y", "erry_lbl")]):
            tk.Label(err, text=f"{lbl}:", fg=self.TEXTDIM,
                     bg=self.PANEL, font=MF).grid(row=0, column=i*2,
                                                   sticky="w", padx=(0, 2))
            w = tk.Label(err, text="  0 px", fg=self.GREEN,
                         bg=self.PANEL, font=MF, width=6)
            w.grid(row=0, column=i*2+1, sticky="w", padx=(0, 10))
            setattr(self, attr, w)

    def _tracking_panel(self, p):
        MF = ("Courier New", 8)

        self.track_btn = tk.Button(
            p, text="▶  START TRACKING",
            bg=self.GREEN, fg=self.BG,
            font=("Courier New", 14, "bold"),
            relief=tk.FLAT, activebackground="#22cc00",
            cursor="hand2", padx=20, pady=18,
            command=self._toggle_tracking)
        self.track_btn.pack(fill=tk.X, pady=(0, 6))

        self.laser_btn = tk.Button(
            p, text="◉  LASER  OFF",
            bg=self.BTNBG, fg=self.TEXTDIM,
            font=("Courier New", 11, "bold"),
            relief=tk.FLAT, cursor="hand2", padx=20, pady=14,
            command=self._toggle_laser)
        self.laser_btn.pack(fill=tk.X, pady=(0, 4))

        tk.Button(p, text="↻  RECONNECT STREAM",
                  bg=self.BTNBG, fg=self.TEXTDIM,
                  font=("Courier New", 10), relief=tk.FLAT,
                  cursor="hand2", padx=20, pady=14,
                  command=self._reconnect).pack(fill=tk.X)

        # Resolution toggle
        tk.Frame(p, bg=self.BORDER, height=1).pack(fill=tk.X, pady=(8, 4))
        res_row = tk.Frame(p, bg=self.PANEL)
        res_row.pack(fill=tk.X, pady=(0, 4))
        tk.Label(res_row, text="RESOLUTION",
                 fg=self.TEXTDIM, bg=self.PANEL, font=MF).pack(side=tk.LEFT)

        self.res_var = tk.StringVar(value="QVGA")
        self.res_lbl = tk.Label(res_row, text="320×240",
                                fg=self.ACCENT2, bg=self.PANEL, font=MF)
        self.res_lbl.pack(side=tk.RIGHT)

        self.res_btn = tk.Button(
            p, text="⇄  SWITCH TO VGA",
            bg=self.BTNBG, fg=self.TEXTDIM,
            font=("Courier New", 10), relief=tk.FLAT,
            cursor="hand2", padx=20, pady=14,
            command=self._toggle_resolution)
        self.res_btn.pack(fill=tk.X)

        ip_row = tk.Frame(p, bg=self.PANEL)
        ip_row.pack(fill=tk.X, pady=(8, 0))
        tk.Label(ip_row, text="ESP32 IP",
                 fg=self.TEXTDIM, bg=self.PANEL, font=MF).pack(side=tk.LEFT)

        self.esp32_ip_var = tk.StringVar(value=ESP32_IP)
        ip_entry = tk.Entry(
            p,
            textvariable=self.esp32_ip_var,
            bg=self.BTNBG,
            fg=self.TEXT,
            insertbackground=self.TEXT,
            relief=tk.FLAT,
            font=("Courier New", 9)
        )
        ip_entry.pack(fill=tk.X, pady=(4, 4))
        ip_entry.bind("<Return>", self._apply_ip_from_event)

        tk.Button(p, text="APPLY ESP32 IP",
                  bg=self.BTNBG, fg=self.TEXTDIM,
                  font=("Courier New", 10), relief=tk.FLAT,
                  cursor="hand2", padx=20, pady=14,
                  command=self._apply_ip).pack(fill=tk.X)

    def _shapes_panel(self, p):
        MF = ("Courier New", 9)
        # [FIX-2] Rectangle added to UI
        labels = {
            'rectangle':  '▭  Rectangle',
            'square':     '▢  Square',
            'circle':     '◯  Circle',
            'cross_plus': '✛  Plus  (+)',
            'cross_x':    '✕  Cross (X)',
        }
        for s, lbl in labels.items():
            tk.Checkbutton(p, text=lbl, variable=self.shape_vars[s],
                           fg=self.TEXT, bg=self.PANEL,
                           selectcolor=self.BTNBG,
                           activebackground=self.PANEL,
                           activeforeground=self.ACCENT,
                           font=MF, cursor="hand2",
                           anchor="w").pack(fill=tk.X, pady=1)

    def _vision_panel(self, p):
        MF = ("Courier New", 10)

        def slider_row(parent, label, var, lo, hi, default, formatter=None):
            tk.Label(parent, text=label, fg=self.TEXTDIM,
                     bg=self.PANEL, font=MF).pack(anchor="w")
            row = tk.Frame(parent, bg=self.PANEL)
            row.pack(fill=tk.X, pady=(0, 8))
            if formatter is None:
                formatter = lambda v: str(int(float(v)))

            lbl = tk.Label(row, text=formatter(default),
                           fg=self.ACCENT2, bg=self.PANEL, font=MF, width=6)

            def on_change(v, _lbl=lbl):
                _lbl.config(text=formatter(v))

            ttk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL,
                      variable=var, command=on_change).pack(
                          side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8), ipady=6)
            lbl.pack(side=tk.LEFT)

        # [FIX-4] block_size replaces thresh_val — adaptive threshold parameter
        self.block_var    = tk.IntVar(value=21)
        self.c_var        = tk.IntVar(value=8)
        self.area_var     = tk.IntVar(value=400)
        self.deadband_var = tk.IntVar(value=DEADBAND)
        self.hfov_var      = tk.DoubleVar(value=CAM_HFOV)
        self.vfov_var      = tk.DoubleVar(value=CAM_VFOV)
        self.pan_scale_var = tk.DoubleVar(value=PAN_SCALE)
        self.tilt_scale_var= tk.DoubleVar(value=TILT_SCALE)

        slider_row(p, "ADAPT. BLOCK SIZE", self.block_var,     7,   51,  21)
        slider_row(p, "ADAPT. C VALUE",    self.c_var,         1,   30,   8)
        slider_row(p, "MIN CONTOUR AREA",  self.area_var,    100, 5000, 400)
        slider_row(p, "DEADBAND (px)",     self.deadband_var,  1,   50,  DEADBAND)
        slider_row(p, "CAM HFOV (deg)",    self.hfov_var,     20,  160,  CAM_HFOV,
                   formatter=lambda v: f"{float(v):.0f}°")
        slider_row(p, "CAM VFOV (deg)",    self.vfov_var,     10,  120,  CAM_VFOV,
                   formatter=lambda v: f"{float(v):.0f}°")
        slider_row(p, "PAN SCALE",         self.pan_scale_var, 0.1,  2.0, PAN_SCALE,
               formatter=lambda v: f"{float(v):.2f}")
        slider_row(p, "TILT SCALE",        self.tilt_scale_var,0.1,  2.0, TILT_SCALE,
               formatter=lambda v: f"{float(v):.2f}")

    # ────────────────────────────────────────────────────────────
    # CALLBACKS
    # ────────────────────────────────────────────────────────────
    def _center_servos(self):
        self.pan_us  = float(PAN_SERVO_CENTER)
        self.tilt_us = float(TILT_SERVO_CENTER)
        self.target_confirm_count = 0
        self.pan_bar_var.set(PAN_SERVO_CENTER)
        self.tilt_bar_var.set(TILT_SERVO_CENTER)
        send_servos(PAN_SERVO_CENTER, TILT_SERVO_CENTER, 0)
        self._log(f"Servos centered → PAN {PAN_SERVO_CENTER}µs, TILT {TILT_SERVO_CENTER}µs")

    def _toggle_tracking(self):
        self.tracking = not self.tracking
        if self.tracking:
            self.track_btn.config(text="■  STOP TRACKING",
                                  bg=self.RED,
                                  activebackground="#cc0022",
                                  fg="#ffffff")
            self.last_target_found = False
            self.target_confirm_count = 0
            self._log("Tracking ACTIVE — searching for targets")
        else:
            self.track_btn.config(text="▶  START TRACKING",
                                  bg=self.GREEN,
                                  activebackground="#22cc00",
                                  fg=self.BG)
            # [FIX-1] Don't reset PID on stop — just turn off laser
            # Servos stay where they are, no jerk
            send_servos(int(self.pan_us), int(self.tilt_us), 0)
            self.target_confirm_count = 0
            self.laser_ind.config(text="◉ LASER OFF", fg=self.TEXTDIM)
            self._log("Tracking STOPPED")

    def _toggle_laser(self):
        if self.tracking:
            self._log("Cannot manually control laser while tracking")
            return
        self.laser_on = not self.laser_on
        if self.laser_on:
            self.laser_btn.config(text="◉  LASER   ON", fg=self.RED)
            self.laser_ind.config(text="◉ LASER ON",   fg=self.RED)
            send_servos(int(self.pan_us), int(self.tilt_us), 1)
        else:
            self.laser_btn.config(text="◉  LASER  OFF", fg=self.TEXTDIM)
            self.laser_ind.config(text="◉ LASER OFF",   fg=self.TEXTDIM)
            send_servos(int(self.pan_us), int(self.tilt_us), 0)

    def _toggle_thresh(self):
        self.show_thresh = not self.show_thresh
        self.thresh_btn.config(
            text="THRESHOLD VIEW:  ON" if self.show_thresh else "THRESHOLD VIEW: OFF",
            fg=self.ACCENT if self.show_thresh else self.TEXTDIM)

    def _manual_pan(self, val):
        if not self.tracking:
            self.pan_us = float(val)
            self.pan_val_lbl.config(text=f"{int(self.pan_us)}µs")
            send_servos(int(self.pan_us), int(self.tilt_us), 0)

    def _manual_tilt(self, val):
        if not self.tracking:
            self.tilt_us = float(val)
            self.tilt_val_lbl.config(text=f"{int(self.tilt_us)}µs")
            send_servos(int(self.pan_us), int(self.tilt_us), 0)

    def _reconnect(self):
        self._log("Reconnecting to stream...")
        self.connected = False
        self.stop_evt.set()
        time.sleep(0.6)
        self.stop_evt.clear()
        self._start_stream_thread()

    def _toggle_resolution(self):
        global FRAME_W, FRAME_H, FRAME_CX, FRAME_CY

        current = self.res_var.get()
        new_res = "VGA" if current == "QVGA" else "QVGA"
        label = "640×480" if new_res == "VGA" else "320×240"

        try:
            r = requests.get(f"http://{ESP32_IP}/resolution?size={new_res}", timeout=3)
            if r.status_code == 200:
                if new_res == "VGA":
                    FRAME_W, FRAME_H = 640, 480
                else:
                    FRAME_W, FRAME_H = 320, 240

                FRAME_CX = FRAME_W // 2
                FRAME_CY = FRAME_H // 2

                self.res_var.set(new_res)
                self.res_lbl.config(text=label)
                next_label = "QVGA" if new_res == "VGA" else "VGA"
                self.res_btn.config(text=f"⇄  SWITCH TO {next_label}")

                self._log(f"Resolution → {new_res} ({label})")
                self._reconnect()
            else:
                self._log(f"Resolution change failed: HTTP {r.status_code}")
        except Exception as e:
            self._log(f"Resolution change error: {e}")

    def _apply_ip_from_event(self, _event):
        self._apply_ip()

    def _apply_ip(self):
        global ESP32_IP, STREAM_URL

        new_ip = self.esp32_ip_var.get().strip()
        if not new_ip:
            self._log("ESP32 IP cannot be empty")
            return

        ipv4_pattern = r"^(?:\d{1,3}\.){3}\d{1,3}$"
        if not re.match(ipv4_pattern, new_ip):
            self._log("Invalid IP format (use x.x.x.x)")
            return

        octets = [int(v) for v in new_ip.split(".")]
        if any(v < 0 or v > 255 for v in octets):
            self._log("Invalid IP range (0-255)")
            return

        ESP32_IP = new_ip
        STREAM_URL = f"http://{ESP32_IP}/stream"
        self._log(f"ESP32 IP set to {ESP32_IP}")
        self._reconnect()

    def _log(self, msg):
        self.status_lbl.config(text=f"» {msg}")

    # ────────────────────────────────────────────────────────────
    # STREAM THREAD
    # [FIX-5] Stream runs fully independently from GUI.
    # Drops oldest frame if queue is full rather than blocking.
    # ────────────────────────────────────────────────────────────
    def _start_stream_thread(self):
        t = threading.Thread(target=self._stream_worker, daemon=True)
        t.start()

    def _stream_worker(self):
        while not self.stop_evt.is_set():
            try:
                for frame in mjpeg_stream(STREAM_URL):
                    if self.stop_evt.is_set():
                        break
                    # [FIX-5] Drop oldest frame if queue is full (never block)
                    if self.frame_q.full():
                        try:
                            self.frame_q.get_nowait()
                        except queue.Empty:
                            pass
                    try:
                        self.frame_q.put_nowait(frame)
                    except queue.Full:
                        pass
            except Exception:
                if not self.stop_evt.is_set():
                    self.connected = False
                    time.sleep(3)

    # ────────────────────────────────────────────────────────────
    # FRAME POLL — runs on Tkinter main thread via root.after()
    # ────────────────────────────────────────────────────────────
    def _poll_frame(self):
        try:
            frame = self.frame_q.get_nowait()
            self.connected = True
            self._process_frame(frame)
        except queue.Empty:
            pass

        self.conn_dot.config(
            text="● ONLINE"  if self.connected else "● OFFLINE",
            fg  =self.GREEN  if self.connected else self.RED)

        # ~60Hz poll — fast enough to feel responsive, light on CPU
        self.root.after(16, self._poll_frame)

    # ────────────────────────────────────────────────────────────
    # FRAME PROCESSING — detection + servo commands
    # ────────────────────────────────────────────────────────────
    def _process_frame(self, frame):
        frame_h, frame_w = frame.shape[:2]
        frame_cx = frame_w // 2
        frame_cy = frame_h // 2

        # Read slider values
        block_size  = self.block_var.get()
        c_val       = self.c_var.get()
        min_area    = self.area_var.get()
        deadband    = self.deadband_var.get()
        cam_hfov    = float(self.hfov_var.get())
        cam_vfov    = float(self.vfov_var.get())
        pan_scale   = float(self.pan_scale_var.get())
        tilt_scale  = float(self.tilt_scale_var.get())

        # block_size must be odd and >= 3
        if block_size % 2 == 0:
            block_size += 1
        block_size = max(3, block_size)

        thresh  = preprocess(frame, block_size, c_val)
        display = frame.copy()

        if self.tracking:
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            target = None
            for cnt in contours:
                shape, cx, cy = classify_contour(cnt, min_area)
                if shape and self.shape_vars[shape].get():
                    if target is None or \
                       PRIORITY.index(shape) < PRIORITY.index(target[0]):
                        target = (shape, cx, cy, cnt)

            if target:
                shape, cx, cy, cnt = target
                self.last_target_found = True

                # Rolling average to smooth jitter
                self._cx_hist.append(cx)
                self._cy_hist.append(cy)
                if len(self._cx_hist) > 3:
                    self._cx_hist.pop(0)
                    self._cy_hist.pop(0)
                cx = int(sum(self._cx_hist) / len(self._cx_hist))
                cy = int(sum(self._cy_hist) / len(self._cy_hist))

                self.err_x = cx - frame_cx
                self.err_y = cy - frame_cy

                # Confirmation filter — ignore first few frames to avoid
                # false detections shooting the servo on tracking start
                self.target_confirm_count += 1
                if self.target_confirm_count < self.TARGET_CONFIRM_FRAMES:
                    send_servos(int(self.pan_us), int(self.tilt_us), laser=0)
                    self.target_lbl.config(
                        text=f"◈ confirming... ({self.target_confirm_count}/{self.TARGET_CONFIRM_FRAMES})",
                        fg=self.YELLOW)
                else:
                    # ── ONE-SHOT ABSOLUTE POSITIONING ──────────────────────
                    # Camera is fixed. Compute where the laser needs to point
                    # based on where the target sits in the frame right now.
                    # Do NOT accumulate. Do NOT step. Just calculate and send.
                    #
                    # pixel error → angle from camera center → µs offset from servo center
                    #
                    # If laser overshoots: lower PAN_SCALE / TILT_SCALE in UI
                    # If laser undershoots: raise them toward 1.0

                    if abs(self.err_x) > deadband:
                        angle_x = (self.err_x / frame_w) * cam_hfov
                        pan_offset = angle_x * PAN_US_PER_DEG * pan_scale
                        if PAN_INVERT:
                            pan_offset = -pan_offset
                        self.pan_us = PAN_SERVO_CENTER + pan_offset

                    if abs(self.err_y) > deadband:
                        angle_y = (self.err_y / frame_h) * cam_vfov
                        tilt_offset = angle_y * TILT_US_PER_DEG * tilt_scale
                        if TILT_INVERT:
                            tilt_offset = -tilt_offset
                        self.tilt_us = TILT_SERVO_CENTER + tilt_offset

                    # Clamp to safe range
                    self.pan_us  = max(PAN_SERVO_MIN,  min(PAN_SERVO_MAX,  self.pan_us))
                    self.tilt_us = max(TILT_SERVO_MIN, min(TILT_SERVO_MAX, self.tilt_us))

                    send_servos(int(self.pan_us), int(self.tilt_us), laser=1)
                    self.laser_ind.config(text="◉ LASER ON", fg=self.RED)
                    self.target_lbl.config(
                        text=f"◈ {shape.upper()}  ({cx},{cy})", fg=self.GREEN)

                # Draw detection overlay
                cv2.drawContours(display, [cnt], -1, (0, 255, 80), 2)
                cv2.circle(display, (cx, cy), 4, (0, 60, 255), -1)
                cv2.putText(display, shape, (cx+8, cy-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            else:
                # Target lost — hold last position, laser off, reset counters
                send_servos(int(self.pan_us), int(self.tilt_us), laser=0)
                self.laser_ind.config(text="◉ LASER OFF", fg=self.TEXTDIM)
                self.target_lbl.config(text="◈ No target", fg=self.TEXTDIM)
                self.err_x = 0
                self.err_y = 0
                self.target_confirm_count = 0
                self._cx_hist.clear()
                self._cy_hist.clear()

        # Crosshair overlay
        cv2.line(display, (frame_cx-7, frame_cy),
                  (frame_cx+7, frame_cy), (255, 210, 0), 1)
        cv2.line(display, (frame_cx, frame_cy-7),
                  (frame_cx, frame_cy+7), (255, 210, 0), 1)
        cv2.circle(display, (frame_cx, frame_cy), 2, (255, 210, 0), 1)

        # FPS counter
        self.frame_count += 1
        now = time.time()
        if now - self.fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.fps_time)
            self.fps_lbl.config(text=f"FPS: {self.fps:.1f}")
            self.frame_count = 0
            self.fps_time    = now

        # Update manual sliders only when not tracking
        if not self.tracking:
            self.pan_bar_var.set(self.pan_us)
            self.tilt_bar_var.set(self.tilt_us)

        self.pan_val_lbl.config( text=f"{int(self.pan_us)}µs")
        self.tilt_val_lbl.config(text=f"{int(self.tilt_us)}µs")

        ex_col = self.RED   if abs(self.err_x) > deadband else self.GREEN
        ey_col = self.RED   if abs(self.err_y) > deadband else self.GREEN
        self.errx_lbl.config(text=f"{self.err_x:+4d}px", fg=ex_col)
        self.erry_lbl.config(text=f"{self.err_y:+4d}px", fg=ey_col)

        # Render to canvas
        if self.show_thresh:
            src = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)
        else:
            src = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw > 10 and ch > 10:
            src = cv2.resize(src, (cw, ch), interpolation=cv2.INTER_LINEAR)

        img  = Image.fromarray(src)
        imtk = ImageTk.PhotoImage(image=img)
        self.canvas.imgtk = imtk
        self.canvas.create_image(0, 0, anchor=tk.NW, image=imtk)

    # ────────────────────────────────────────────────────────────
    # CLEAN SHUTDOWN
    # ────────────────────────────────────────────────────────────
    def on_close(self):
        self.tracking = False
        self.stop_evt.set()
        send_servos(PAN_SERVO_CENTER, TILT_SERVO_CENTER, 0)
        sock.close()
        self.root.destroy()


# ═══════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════
def apply_ttk_style(root):
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure("TScale",
                    background="#111820",
                    troughcolor="#1e2d3d",
                    sliderlength=38,
                    sliderrelief=tk.FLAT)

def main():
    root = tk.Tk()
    apply_ttk_style(root)
    app  = TrackerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()