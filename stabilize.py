"""Marble stabilizer: SCAMP-5d ball tracker + OpenRB-150 PID tilt control.

Combines:
- the visual tracker from scamp5-sol/interface/scamp_python_minimal/minimal.py
  (SCAMP packet capture on TX channel 100 + connected-components ball blob),
- the OpenRB-150 motor driver via labyrinth.Labyrinth from labyrinth.py,
- a 2-axis PID feedback loop that pushes (left_right, front_back) tilts to the
  board to stabilise the ball at a clicked target position.

Live tuning: a "stabilizer controls" window has trackbars for Kp/Ki/Kd per
axis (read every frame, so changes take effect immediately), a mode selector
(PID / LEVEL / MANUAL), and manual-tilt sliders that override the PID when
mode = MANUAL. Use them to retune without restarting the script.

Engage button: the script starts in LEVEL (board flat). Click the big button
under the SCAMP image to ENGAGE the PID loop; click again to disengage. This
is meant to be the safe default — nothing tilts until you ask for it.

PID-term plots: under the engage button, two scrolling plots (one per axis)
show the live P / I / D contributions to the tilt output. Useful for tuning:
look at which term dominates and whether anything is winding up. Auto-scaled.

Kill switches (all redundant):

    spacebar          HARD KILL: disables motor torque immediately and exits.
    q / Esc           Graceful: centre board, disable torque, exit.
    Ctrl+C (terminal) Same as spacebar.
    Window closed     Graceful quit.

Other hotkeys / clicks in the main window:

    left-click on image    set target at clicked pixel
    left-click on button   toggle PID engaged / disengaged
    p                      cycle mode (PID -> LEVEL -> MANUAL -> PID)
    l                      force LEVEL mode
    m                      force MANUAL mode
    r                      reset PID integrators (and clears the plot trails)
    t                      DIRECTION TEST: walks tilt through +X, -X, +Y, -Y
    x / y                  invert x / y axis sign

Run from the project root:

    python stabilize.py
    python stabilize.py --port COM5
    python stabilize.py --kp 0.005 --kd 0.003

Requires:
    pip install pyserial opencv-python numpy
"""

import argparse
import os
import signal
import sys
import threading
import time
import typing
from dataclasses import dataclass, field

import cv2
import numpy as np
import serial
import serial.tools.list_ports

# Make the SCAMP .pyd discoverable regardless of cwd.
SCAMP_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "scamp5-sol", "interface", "scamp_python_minimal",
)
if SCAMP_DIR not in sys.path:
    sys.path.insert(0, SCAMP_DIR)
import scamp  # noqa: E402

import labyrinth as labyrinth_module  # noqa: E402
from labyrinth_demo import autodetect_port, list_ports_verbose  # noqa: E402


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

USB_SERIAL = "0"
TX_CHANNEL = 100               # the SCAMP channel carrying the ball-candidate map
WINDOW_NAME = "marble stabilizer"
CONTROLS_WINDOW = "stabilizer controls"
DISPLAY_SCALE = 2

# Calibration constants for this rig.

MAX_ROT = 500
LEFT_LIMIT = 2244 - MAX_ROT
RIGHT_LIMIT = 2244 + MAX_ROT
FRONT_LIMIT = 2091 - MAX_ROT
BACK_LIMIT = 2091 + MAX_ROT

# PID defaults. Tilt output is in [-0.5, 0.5] (added to 0.5 -> [0, 1]).
# Image coords are 0..255; full-range error of ~128 px should not produce more
# than ~0.4 tilt fraction with default Kp.
DEFAULT_KP = 0.003
DEFAULT_KI = 0.0005
DEFAULT_KD = 0.002

INTEGRAL_LIMIT = 200.0     # px*s — caps integrator wind-up
TILT_OUTPUT_LIMIT = 0.45   # keep some headroom from the calibrated limits

# When the ball has been lost for this many consecutive frames, level the
# board (PID hold-off). Prevents the integrator from pushing the board hard
# while we have no measurement.
BALL_LOST_LEVEL_FRAMES = 5

# Direction-test: each phase holds tilt for this long, then advances.
DIRTEST_PHASE_S = 1.5
DIRTEST_TILT_DELTA = 0.30   # how far from 0.5 to push during the test

# Trackbar scales (cv2 trackbars are integer-valued).
KP_SCALE = 10000     # trackbar 0..500 -> 0..0.050  (step 1e-4)
KI_SCALE = 100000    # trackbar 0..500 -> 0..0.005  (step 1e-5)
KD_SCALE = 10000     # trackbar 0..500 -> 0..0.050  (step 1e-4)
KP_TB_MAX = 500
KI_TB_MAX = 500
KD_TB_MAX = 500
TILT_TB_MAX = 100    # trackbar 0..100 -> 0.0..1.0

# Centre trim: shifts where "0.5" actually points without changing PID gains.
# Lets the operator (or the auto-trim hotkey) absorb persistent motor backlash
# / drift into a static bias so the integrator stays unloaded.
TRIM_RANGE = 0.25                # max additive offset, ±TRIM_RANGE
TRIM_TB_MAX = 200                # trackbar 0..200, centred at 100 = 0.0
TRIM_TB_CENTRE = TRIM_TB_MAX // 2

# Output low-pass filter applied in the labyrinth thread, after trim, before
# tilt(). 100 = pass-through (no smoothing). Lower = smoother.
SMOOTHING_TB_MAX = 100

# Below this absolute change in commanded tilt, the labyrinth thread skips the
# motor write — keeps smoothing from spamming the serial bus once it settles.
SEND_THRESHOLD = 0.001

# Engage button (rendered under the SCAMP image inside the main window).
BUTTON_HEIGHT = 30


# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------

class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = TILT_OUTPUT_LIMIT,
                 integral_limit: float = INTEGRAL_LIMIT):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.prev_error: typing.Optional[float] = None
        self.integral = 0.0
        # Last contributions of each term, exposed for plotting.
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def reset(self) -> None:
        self.prev_error = None
        self.integral = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        self.last_p = self.kp * error
        self.last_i = self.ki * self.integral
        self.last_d = self.kd * derivative
        out = self.last_p + self.last_i + self.last_d
        return max(-self.output_limit, min(self.output_limit, out))


# ---------------------------------------------------------------------------
# Ball tracker (copied verbatim from minimal.py — same behaviour the user
# already validated)
# ---------------------------------------------------------------------------

class BallTracker:
    def __init__(self):
        self.prev_xy: typing.Optional[tuple[int, int]] = None
        self.lost_count = 0
        self.min_area = 5
        self.max_area = 400
        self.min_w = 2
        self.min_h = 2
        self.max_w = 40
        self.max_h = 40
        self.min_aspect = 0.3
        self.max_aspect = 3.0
        self.distance_penalty = 0.5
        self.use_morphology = True
        self.kernel = np.ones((3, 3), np.uint8)

    def update(self, img_u8: np.ndarray) -> typing.Optional[dict]:
        mask = (img_u8 > 0).astype(np.uint8) * 255
        if self.use_morphology:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        n, _labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8,
        )
        best = None
        best_score = -1e18
        for label in range(1, n):
            x, y, w, h, area = stats[label]
            if area < self.min_area or area > self.max_area:
                continue
            if w < self.min_w or h < self.min_h:
                continue
            if w > self.max_w or h > self.max_h:
                continue
            aspect = w / max(h, 1)
            if aspect < self.min_aspect or aspect > self.max_aspect:
                continue
            cx, cy = centroids[label]
            score = float(area)
            if self.prev_xy is not None:
                px, py = self.prev_xy
                score -= self.distance_penalty * (abs(cx - px) + abs(cy - py))
            if score > best_score:
                best_score = score
                best = {
                    "xy": (int(round(cx)), int(round(cy))),
                    "bbox": (int(x), int(y), int(w), int(h)),
                    "area": int(area),
                }
        if best is None:
            self.lost_count += 1
            return None
        self.prev_xy = best["xy"]
        self.lost_count = 0
        return best


def packet_to_numpy_fast(packet: dict) -> np.ndarray:
    w, h = packet["width"], packet["height"]
    img = np.frombuffer(packet["buffer"], dtype=np.uint8).reshape((h, w))
    img = np.flipud(img)
    return np.ascontiguousarray(img)


# ---------------------------------------------------------------------------
# PID-term plotter: scrolling P/I/D contributions per axis, drawn into an
# OpenCV image so it composes naturally under the SCAMP view.
# ---------------------------------------------------------------------------

class PIDPlotter:
    """Ring-buffer scope for the 6 PID terms (P/I/D × X/Y)."""

    LENGTH = 256              # samples kept in history (= image width in px)
    AXIS_HEIGHT = 80          # px per sub-plot
    BG_COLOR = (24, 24, 24)
    AXIS_COLOR = (70, 70, 70)
    LABEL_COLOR = (210, 210, 210)
    P_COLOR = (60, 60, 255)   # red
    I_COLOR = (60, 255, 60)   # green
    D_COLOR = (255, 200, 60)  # cyan

    def __init__(self):
        # rows: 0=Px 1=Ix 2=Dx 3=Py 4=Iy 5=Dy
        self.buf = np.zeros((6, self.LENGTH), dtype=np.float32)
        self.pos = 0

    def push(self, px: float, ix: float, dx: float,
             py: float, iy: float, dy: float) -> None:
        i = self.pos
        self.buf[0, i] = px
        self.buf[1, i] = ix
        self.buf[2, i] = dx
        self.buf[3, i] = py
        self.buf[4, i] = iy
        self.buf[5, i] = dy
        self.pos = (i + 1) % self.LENGTH

    def clear(self) -> None:
        self.buf.fill(0.0)
        self.pos = 0

    def render(self, width: int) -> np.ndarray:
        """X-axis plot stacked above Y-axis plot. Shape (2*AXIS_HEIGHT, width, 3)."""
        return np.vstack([
            self._render_axis(width, axis=0, name="X"),
            self._render_axis(width, axis=1, name="Y"),
        ])

    def _render_axis(self, width: int, axis: int, name: str) -> np.ndarray:
        h = self.AXIS_HEIGHT
        img = np.full((h, width, 3), self.BG_COLOR, dtype=np.uint8)

        # Roll so the most recent sample sits at the right edge.
        rows = (axis * 3 + 0, axis * 3 + 1, axis * 3 + 2)
        traces = [np.roll(self.buf[r], -self.pos) for r in rows]

        vmax = max(0.05, float(max(np.abs(t).max() for t in traces)))

        cy = h // 2
        cv2.line(img, (0, cy), (width, cy), self.AXIS_COLOR, 1)

        xs = np.arange(width, dtype=np.int32)
        half = h // 2 - 4
        if width != self.LENGTH:
            sample_idx = np.linspace(0, self.LENGTH - 1, width)
            ref_idx = np.arange(self.LENGTH)

            def resample(t):
                return np.interp(sample_idx, ref_idx, t)
        else:
            def resample(t):
                return t

        colors = (self.P_COLOR, self.I_COLOR, self.D_COLOR)
        for trace, color in zip(traces, colors):
            v = resample(trace)
            ys = (cy - (v / vmax) * half).astype(np.int32)
            pts = np.stack([xs, ys], axis=1).reshape((-1, 1, 2))
            cv2.polylines(img, [pts], False, color, 1, cv2.LINE_AA)

        latest_p, latest_i, latest_d = traces[0][-1], traces[1][-1], traces[2][-1]
        label = (f"{name}:  P={latest_p:+.4f}  I={latest_i:+.4f}  "
                 f"D={latest_d:+.4f}   range=+/-{vmax:.3f}")
        cv2.putText(img, label, (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    self.LABEL_COLOR, 1, cv2.LINE_AA)

        return img


# ---------------------------------------------------------------------------
# Engage button
# ---------------------------------------------------------------------------

def render_engage_button(width: int, engaged: bool) -> np.ndarray:
    img = np.zeros((BUTTON_HEIGHT, width, 3), dtype=np.uint8)
    bg = (40, 200, 40) if engaged else (40, 40, 200)
    cv2.rectangle(img, (0, 0), (width - 1, BUTTON_HEIGHT - 1), bg, -1)
    cv2.rectangle(img, (1, 1), (width - 2, BUTTON_HEIGHT - 2), (255, 255, 255), 1)
    label = "DISENGAGE PID  (click)" if engaged else "ENGAGE PID  (click)"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    tx = max(4, (width - tw) // 2)
    ty = BUTTON_HEIGHT // 2 + th // 2
    cv2.putText(img, label, (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ---------------------------------------------------------------------------
# Direction-test macro
# ---------------------------------------------------------------------------

DIRTEST_PHASES = [
    ("+X (lr+)", 0.5 + DIRTEST_TILT_DELTA, 0.5),
    ("-X (lr-)", 0.5 - DIRTEST_TILT_DELTA, 0.5),
    ("+Y (fb+)", 0.5, 0.5 + DIRTEST_TILT_DELTA),
    ("-Y (fb-)", 0.5, 0.5 - DIRTEST_TILT_DELTA),
    ("centre",   0.5, 0.5),
]


@dataclass
class DirectionTest:
    active: bool = False
    phase: int = 0
    phase_start: float = 0.0


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

MODES = ["PID", "LEVEL", "MANUAL"]


@dataclass
class State:
    target_x: float = 128.0
    target_y: float = 128.0
    measured_x: float = 128.0
    measured_y: float = 128.0
    ball_present: bool = False

    desired_lr: float = 0.5
    desired_fb: float = 0.5

    # Default LEVEL: nothing tilts until the user clicks ENGAGE PID.
    mode: str = "LEVEL"

    invert_x: bool = False
    invert_y: bool = False

    # Centre trim and output smoothing (read from trackbars each frame).
    trim_lr: float = 0.0
    trim_fb: float = 0.0
    smoothing_alpha: float = 1.0   # 1.0 = pass-through

    kill: bool = False           # hard stop: drop torque, exit ASAP
    quit: bool = False           # graceful: centre, then stop

    lock: threading.Lock = field(default_factory=threading.Lock)


# ---------------------------------------------------------------------------
# Trackbars
# ---------------------------------------------------------------------------

TB_KP_X = f"Kp_x x{KP_SCALE}"
TB_KI_X = f"Ki_x x{KI_SCALE}"
TB_KD_X = f"Kd_x x{KD_SCALE}"
TB_KP_Y = f"Kp_y x{KP_SCALE}"
TB_KI_Y = f"Ki_y x{KI_SCALE}"
TB_KD_Y = f"Kd_y x{KD_SCALE}"
TB_MODE = "mode 0PID 1LEVEL 2MANUAL"
TB_LR = "manual_lr x100"
TB_FB = "manual_fb x100"
TB_TRIM_LR = "trim_lr (100=0)"
TB_TRIM_FB = "trim_fb (100=0)"
TB_SMOOTHING = "smoothing 0..100 (100=off)"


def setup_controls(initial_kp: float, initial_ki: float, initial_kd: float,
                   initial_mode_idx: int = 1) -> None:
    cv2.namedWindow(CONTROLS_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS_WINDOW, 480, 360)
    def nop(_):
        pass
    cv2.createTrackbar(TB_KP_X, CONTROLS_WINDOW, min(int(round(initial_kp * KP_SCALE)), KP_TB_MAX), KP_TB_MAX, nop)
    cv2.createTrackbar(TB_KI_X, CONTROLS_WINDOW, min(int(round(initial_ki * KI_SCALE)), KI_TB_MAX), KI_TB_MAX, nop)
    cv2.createTrackbar(TB_KD_X, CONTROLS_WINDOW, min(int(round(initial_kd * KD_SCALE)), KD_TB_MAX), KD_TB_MAX, nop)
    cv2.createTrackbar(TB_KP_Y, CONTROLS_WINDOW, min(int(round(initial_kp * KP_SCALE)), KP_TB_MAX), KP_TB_MAX, nop)
    cv2.createTrackbar(TB_KI_Y, CONTROLS_WINDOW, min(int(round(initial_ki * KI_SCALE)), KI_TB_MAX), KI_TB_MAX, nop)
    cv2.createTrackbar(TB_KD_Y, CONTROLS_WINDOW, min(int(round(initial_kd * KD_SCALE)), KD_TB_MAX), KD_TB_MAX, nop)
    cv2.createTrackbar(TB_MODE, CONTROLS_WINDOW, initial_mode_idx, 2, nop)
    cv2.createTrackbar(TB_LR, CONTROLS_WINDOW, 50, TILT_TB_MAX, nop)
    cv2.createTrackbar(TB_FB, CONTROLS_WINDOW, 50, TILT_TB_MAX, nop)
    cv2.createTrackbar(TB_TRIM_LR, CONTROLS_WINDOW, TRIM_TB_CENTRE, TRIM_TB_MAX, nop)
    cv2.createTrackbar(TB_TRIM_FB, CONTROLS_WINDOW, TRIM_TB_CENTRE, TRIM_TB_MAX, nop)
    cv2.createTrackbar(TB_SMOOTHING, CONTROLS_WINDOW, SMOOTHING_TB_MAX, SMOOTHING_TB_MAX, nop)


def _trim_from_tb(tb_value: int) -> float:
    return (tb_value - TRIM_TB_CENTRE) / TRIM_TB_CENTRE * TRIM_RANGE


def _trim_to_tb(trim: float) -> int:
    pos = int(round(trim / TRIM_RANGE * TRIM_TB_CENTRE + TRIM_TB_CENTRE))
    return max(0, min(TRIM_TB_MAX, pos))


def read_controls(pid_x: PID, pid_y: PID
                  ) -> tuple[str, float, float, float, float, float]:
    pid_x.kp = cv2.getTrackbarPos(TB_KP_X, CONTROLS_WINDOW) / KP_SCALE
    pid_x.ki = cv2.getTrackbarPos(TB_KI_X, CONTROLS_WINDOW) / KI_SCALE
    pid_x.kd = cv2.getTrackbarPos(TB_KD_X, CONTROLS_WINDOW) / KD_SCALE
    pid_y.kp = cv2.getTrackbarPos(TB_KP_Y, CONTROLS_WINDOW) / KP_SCALE
    pid_y.ki = cv2.getTrackbarPos(TB_KI_Y, CONTROLS_WINDOW) / KI_SCALE
    pid_y.kd = cv2.getTrackbarPos(TB_KD_Y, CONTROLS_WINDOW) / KD_SCALE
    mode = MODES[cv2.getTrackbarPos(TB_MODE, CONTROLS_WINDOW)]
    manual_lr = cv2.getTrackbarPos(TB_LR, CONTROLS_WINDOW) / TILT_TB_MAX
    manual_fb = cv2.getTrackbarPos(TB_FB, CONTROLS_WINDOW) / TILT_TB_MAX
    trim_lr = _trim_from_tb(cv2.getTrackbarPos(TB_TRIM_LR, CONTROLS_WINDOW))
    trim_fb = _trim_from_tb(cv2.getTrackbarPos(TB_TRIM_FB, CONTROLS_WINDOW))
    smooth_int = cv2.getTrackbarPos(TB_SMOOTHING, CONTROLS_WINDOW)
    alpha = max(0.01, smooth_int / SMOOTHING_TB_MAX)
    return mode, manual_lr, manual_fb, trim_lr, trim_fb, alpha


def set_mode_trackbar(mode: str) -> None:
    cv2.setTrackbarPos(TB_MODE, CONTROLS_WINDOW, MODES.index(mode))


def set_trim_trackbars(trim_lr: float, trim_fb: float) -> None:
    cv2.setTrackbarPos(TB_TRIM_LR, CONTROLS_WINDOW, _trim_to_tb(trim_lr))
    cv2.setTrackbarPos(TB_TRIM_FB, CONTROLS_WINDOW, _trim_to_tb(trim_fb))


# ---------------------------------------------------------------------------
# Labyrinth thread
# ---------------------------------------------------------------------------

def labyrinth_thread_target(labyrinth: labyrinth_module.Labyrinth,
                            state: State) -> None:
    last_lr: typing.Optional[float] = None
    last_fb: typing.Optional[float] = None
    filtered_lr = 0.5
    filtered_fb = 0.5
    while True:
        with state.lock:
            kill = state.kill
            quit_ = state.quit
            target_lr = state.desired_lr
            target_fb = state.desired_fb
            trim_lr = state.trim_lr
            trim_fb = state.trim_fb
            alpha = state.smoothing_alpha

        if kill:
            print("[labyrinth] kill received -> disabling torque")
            break
        if quit_:
            try:
                labyrinth.tilt(0.5, 0.5)
            except Exception as exc:
                print(f"[labyrinth] centre tilt on quit failed: {exc}")
            break

        # 1. Apply trim (additive offset around 0.5) and clip to [0, 1].
        trimmed_lr = max(0.0, min(1.0, target_lr + trim_lr))
        trimmed_fb = max(0.0, min(1.0, target_fb + trim_fb))

        # 2. Output low-pass smoothing. alpha=1 is pass-through.
        if alpha >= 0.999:
            filtered_lr = trimmed_lr
            filtered_fb = trimmed_fb
        else:
            filtered_lr = (1.0 - alpha) * filtered_lr + alpha * trimmed_lr
            filtered_fb = (1.0 - alpha) * filtered_fb + alpha * trimmed_fb

        # 3. Send only if the filtered command moved enough — keeps the
        #    smoothing tail from spamming the bus once it has settled.
        if last_lr is None or last_fb is None:
            send = True
        else:
            send = (abs(filtered_lr - last_lr) > SEND_THRESHOLD
                    or abs(filtered_fb - last_fb) > SEND_THRESHOLD)
        if send:
            try:
                labyrinth.tilt(left_right=filtered_lr, front_back=filtered_fb)
            except Exception as exc:
                print(f"[labyrinth] tilt failed: {exc} -> engaging kill")
                with state.lock:
                    state.kill = True
                break
            last_lr, last_fb = filtered_lr, filtered_fb
        else:
            time.sleep(0.002)

    try:
        labyrinth.stop()
        print("[labyrinth] motors stopped")
    except Exception as exc:
        print(f"[labyrinth] stop failed: {exc}")


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------

def render(img_u8: np.ndarray, ball: typing.Optional[dict],
           state: State, pid_x: PID, pid_y: PID,
           dirtest: DirectionTest, fps: float,
           plotter: PIDPlotter) -> np.ndarray:
    scamp_vis = cv2.cvtColor(img_u8, cv2.COLOR_GRAY2BGR)

    with state.lock:
        tx, ty = int(state.target_x), int(state.target_y)
        mode = state.mode
        invert_x = state.invert_x
        invert_y = state.invert_y
        desired_lr = state.desired_lr
        desired_fb = state.desired_fb
        trim_lr = state.trim_lr
        trim_fb = state.trim_fb
        smoothing_alpha = state.smoothing_alpha
        kill = state.kill

    cv2.drawMarker(scamp_vis, (tx, ty), (0, 255, 0),
                   markerType=cv2.MARKER_CROSS, markerSize=14, thickness=1)
    cv2.circle(scamp_vis, (tx, ty), 6, (0, 255, 0), 1, lineType=cv2.LINE_AA)

    if ball is not None:
        bx, by = ball["xy"]
        cv2.line(scamp_vis, (bx, by), (tx, ty), (0, 255, 255), 1, lineType=cv2.LINE_AA)
        cv2.circle(scamp_vis, (bx, by), 3, (0, 0, 255), -1, lineType=cv2.LINE_AA)

        ux = (desired_lr - 0.5) * (-1.0 if invert_x else 1.0)
        uy = (desired_fb - 0.5) * (-1.0 if invert_y else 1.0)
        if float(np.hypot(ux, uy)) > 1e-3:
            scale = 60.0
            ax = int(round(bx + ux * scale))
            ay = int(round(by + uy * scale))
            cv2.arrowedLine(scamp_vis, (bx, by), (ax, ay),
                            (255, 0, 255), 2, line_type=cv2.LINE_AA, tipLength=0.25)

    width = scamp_vis.shape[1]
    button = render_engage_button(width, mode == "PID" and not dirtest.active)
    plots = plotter.render(width)
    composite = np.vstack([scamp_vis, button, plots])

    if DISPLAY_SCALE != 1:
        composite = cv2.resize(
            composite,
            (composite.shape[1] * DISPLAY_SCALE, composite.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )

    line1 = (
        f"fps={fps:5.1f}  mode={mode}  "
        f"kill={'YES' if kill else 'no'}  "
        f"invert={'X' if invert_x else '-'}{'Y' if invert_y else '-'}"
        + (f"  DIRTEST {dirtest.phase + 1}/{len(DIRTEST_PHASES)}: "
           f"{DIRTEST_PHASES[dirtest.phase][0]}" if dirtest.active else "")
    )
    line2 = (
        f"X: Kp={pid_x.kp:.4f} Ki={pid_x.ki:.5f} Kd={pid_x.kd:.4f}    "
        f"Y: Kp={pid_y.kp:.4f} Ki={pid_y.ki:.5f} Kd={pid_y.kd:.4f}"
    )
    line3 = (
        f"target=({tx},{ty})  tilt=({desired_lr:.2f},{desired_fb:.2f})  "
        + (f"ball=({ball['xy'][0]},{ball['xy'][1]}) area={ball['area']}"
           if ball is not None else "ball=LOST")
    )
    line4 = (
        f"trim=({trim_lr:+.3f},{trim_fb:+.3f})  "
        f"smooth_a={smoothing_alpha:.2f}"
        + ("  (z=auto-trim)" if mode == "PID" else "")
    )
    for i, text in enumerate((line1, line2, line3, line4)):
        cv2.putText(composite, text, (6, 16 + 16 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    if kill:
        cv2.rectangle(composite, (0, 0), (composite.shape[1] - 1, composite.shape[0] - 1),
                      (0, 0, 255), 4)
        cv2.putText(composite, "KILL",
                    (composite.shape[1] // 2 - 60, composite.shape[0] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 3, cv2.LINE_AA)

    return composite


# ---------------------------------------------------------------------------
# Direction-test stepper
# ---------------------------------------------------------------------------

def step_direction_test(dirtest: DirectionTest, state: State) -> None:
    if not dirtest.active:
        return
    now = time.perf_counter()
    if now - dirtest.phase_start >= DIRTEST_PHASE_S:
        dirtest.phase += 1
        dirtest.phase_start = now
        if dirtest.phase >= len(DIRTEST_PHASES):
            dirtest.active = False
            dirtest.phase = 0
            print("[dirtest] done. if the ball moved opposite to the announced "
                  "direction in any phase, press x and/or y to invert.")
            with state.lock:
                state.desired_lr = 0.5
                state.desired_fb = 0.5
                state.mode = "LEVEL"
            set_mode_trackbar("LEVEL")
            return
        name, lr, fb = DIRTEST_PHASES[dirtest.phase]
        print(f"[dirtest] phase {dirtest.phase + 1}/{len(DIRTEST_PHASES)} "
              f"-> {name}  tilt=({lr:.2f},{fb:.2f}) — watch the ball")
    name, lr, fb = DIRTEST_PHASES[dirtest.phase]
    with state.lock:
        state.desired_lr = lr
        state.desired_fb = fb


def begin_direction_test(dirtest: DirectionTest, state: State) -> None:
    dirtest.active = True
    dirtest.phase = 0
    dirtest.phase_start = time.perf_counter()
    name, lr, fb = DIRTEST_PHASES[0]
    print(f"[dirtest] starting. {len(DIRTEST_PHASES)} phases of "
          f"{DIRTEST_PHASE_S}s each.")
    print(f"[dirtest] phase 1/{len(DIRTEST_PHASES)} -> {name}  "
          f"tilt=({lr:.2f},{fb:.2f}) — watch the ball")
    with state.lock:
        state.mode = "MANUAL"
        state.desired_lr = lr
        state.desired_fb = fb
    set_mode_trackbar("MANUAL")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", help="OpenRB-150 serial port, e.g. COM5")
    p.add_argument("--list-ports", action="store_true")
    p.add_argument("--kp", type=float, default=DEFAULT_KP)
    p.add_argument("--ki", type=float, default=DEFAULT_KI)
    p.add_argument("--kd", type=float, default=DEFAULT_KD)
    p.add_argument("--invert-x", action="store_true")
    p.add_argument("--invert-y", action="store_true")
    return p.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.list_ports:
        list_ports_verbose()
        return 0

    port = args.port or autodetect_port()
    if port is None:
        print("could not auto-detect OpenRB-150 port; pass --port COMx")
        list_ports_verbose()
        return 1

    state = State()
    # Always start in LEVEL — the rig stays flat until the user clicks
    # ENGAGE PID. Both the state and the mode trackbar default to LEVEL.
    state.mode = "LEVEL"
    state.invert_x = args.invert_x
    state.invert_y = args.invert_y

    pid_x = PID(args.kp, args.ki, args.kd)
    pid_y = PID(args.kp, args.ki, args.kd)
    tracker = BallTracker()
    dirtest = DirectionTest()
    plotter = PIDPlotter()

    def handle_sigint(_sig, _frame):
        print("\n[signal] SIGINT -> kill")
        with state.lock:
            state.kill = True
    signal.signal(signal.SIGINT, handle_sigint)

    print(f"opening SCAMP-5d (USB serial {USB_SERIAL!r})...")
    rc = scamp.open_usb(USB_SERIAL)
    if rc != 0:
        print(f"open_usb failed rc={rc}")
        return 1
    scamp.send_message(scamp.VS_MSG_HOST_ON, 0, 0)
    print("scamp connected.")

    print(f"opening labyrinth on {port}...")
    try:
        labyrinth = labyrinth_module.Labyrinth(
            port=port,
            left_limit=LEFT_LIMIT, right_limit=RIGHT_LIMIT,
            front_limit=FRONT_LIMIT, back_limit=BACK_LIMIT,
        )
    except (serial.SerialException, OSError, AssertionError) as exc:
        print(f"failed to open labyrinth: {exc}")
        scamp.send_message(scamp.VS_MSG_HOST_DC, 0, 0)
        scamp.close()
        return 1
    print("labyrinth ready.")

    lab_thread = threading.Thread(
        target=labyrinth_thread_target, args=(labyrinth, state), daemon=True,
    )
    lab_thread.start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    setup_controls(args.kp, args.ki, args.kd,
                   initial_mode_idx=MODES.index("LEVEL"))

    # Click regions in native (un-scaled) image y-coords:
    #   0..255       SCAMP image      -> set target
    #   256..285     ENGAGE PID button -> toggle PID
    #   286..        plots             -> ignored
    SCAMP_H = 256
    BUTTON_Y1 = SCAMP_H + BUTTON_HEIGHT

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        img_x = int(x / DISPLAY_SCALE)
        img_y = int(y / DISPLAY_SCALE)
        if img_y < SCAMP_H:
            with state.lock:
                state.target_x = float(img_x)
                state.target_y = float(img_y)
            pid_x.reset()
            pid_y.reset()
            plotter.clear()
            print(f"[target] -> ({img_x}, {img_y})")
        elif img_y < BUTTON_Y1:
            with state.lock:
                if state.mode == "PID":
                    state.mode = "LEVEL"
                    new_mode = "LEVEL"
                else:
                    state.mode = "PID"
                    new_mode = "PID"
            pid_x.reset()
            pid_y.reset()
            plotter.clear()
            set_mode_trackbar(new_mode)
            print(f"[button] -> {new_mode}")

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    print("controls: click image=target  click button=engage PID  "
          "SPACE=KILL  q/Esc=quit  l=level  m=manual  r=reset  t=dirtest  x/y=invert")

    last_t = time.perf_counter()
    fps_window: list[float] = []
    last_image: typing.Optional[np.ndarray] = None
    last_ball: typing.Optional[dict] = None

    try:
        while True:
            with state.lock:
                if state.kill or state.quit:
                    break

            # Read live trackbar values into PID gains + mode + manual tilt
            # + trim + smoothing.
            (tb_mode, manual_lr, manual_fb,
             trim_lr_val, trim_fb_val, alpha) = read_controls(pid_x, pid_y)
            with state.lock:
                state.trim_lr = trim_lr_val
                state.trim_fb = trim_fb_val
                state.smoothing_alpha = alpha
                if not dirtest.active:
                    state.mode = tb_mode

            scamp.routine()
            n = 0
            while n < 1000:
                packet = scamp.get_packet()
                if packet is None:
                    break
                n += 1
                if packet.get("type") != "data":
                    continue
                if packet.get("datatype") not in ("SCAMP5_AOUT", "SCAMP5_DOUT"):
                    continue
                if packet.get("channel") != TX_CHANNEL:
                    continue

                img = packet_to_numpy_fast(packet)
                last_image = img

                ball = tracker.update(img)
                last_ball = ball

                now = time.perf_counter()
                dt = now - last_t
                last_t = now

                fps_window.append(now)
                while fps_window and now - fps_window[0] > 1.0:
                    fps_window.pop(0)

                pid_active_this_frame = False
                if dirtest.active:
                    pass  # handled by step_direction_test below
                elif state.mode == "PID" and ball is not None:
                    bx, by = ball["xy"]
                    with state.lock:
                        state.measured_x = float(bx)
                        state.measured_y = float(by)
                        state.ball_present = True
                        ex = state.target_x - bx
                        ey = state.target_y - by
                        if state.invert_x:
                            ex = -ex
                        if state.invert_y:
                            ey = -ey
                    out_x = pid_x.update(ex, dt)
                    out_y = pid_y.update(ey, dt)
                    with state.lock:
                        state.desired_lr = 0.5 + out_x
                        state.desired_fb = 0.5 + out_y
                    pid_active_this_frame = True
                elif state.mode == "PID" and ball is None:
                    with state.lock:
                        state.ball_present = False
                        if tracker.lost_count >= BALL_LOST_LEVEL_FRAMES:
                            state.desired_lr = 0.5
                            state.desired_fb = 0.5
                elif state.mode == "MANUAL":
                    with state.lock:
                        state.desired_lr = manual_lr
                        state.desired_fb = manual_fb
                else:  # LEVEL
                    with state.lock:
                        state.desired_lr = 0.5
                        state.desired_fb = 0.5

                # Push PID-term sample (zeros when PID didn't fire this frame).
                if pid_active_this_frame:
                    plotter.push(pid_x.last_p, pid_x.last_i, pid_x.last_d,
                                 pid_y.last_p, pid_y.last_i, pid_y.last_d)
                else:
                    plotter.push(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

            step_direction_test(dirtest, state)

            if last_image is not None:
                fps = (len(fps_window) / max(1e-3, fps_window[-1] - fps_window[0])
                       if len(fps_window) >= 2 else 0.0)
                vis = render(last_image, last_ball, state, pid_x, pid_y,
                             dirtest, fps, plotter)
                cv2.imshow(WINDOW_NAME, vis)

                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    print("[window] closed -> quit")
                    with state.lock:
                        state.quit = True
                    break

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                if key == ord(" "):
                    print("[hotkey] SPACE -> KILL")
                    with state.lock:
                        state.kill = True
                    break
                elif key in (ord("q"), 27):
                    print("[hotkey] q/Esc -> quit")
                    with state.lock:
                        state.quit = True
                    break
                elif key == ord("p"):
                    with state.lock:
                        idx = (MODES.index(state.mode) + 1) % len(MODES)
                        state.mode = MODES[idx]
                        m = state.mode
                    set_mode_trackbar(m)
                    pid_x.reset()
                    pid_y.reset()
                    plotter.clear()
                    print(f"[hotkey] mode -> {m}")
                elif key == ord("l"):
                    with state.lock:
                        state.mode = "LEVEL"
                    set_mode_trackbar("LEVEL")
                    pid_x.reset()
                    pid_y.reset()
                    plotter.clear()
                    print("[hotkey] LEVEL")
                elif key == ord("m"):
                    with state.lock:
                        state.mode = "MANUAL"
                    set_mode_trackbar("MANUAL")
                    print("[hotkey] MANUAL")
                elif key == ord("r"):
                    pid_x.reset()
                    pid_y.reset()
                    plotter.clear()
                    print("[hotkey] PID integrators reset")
                elif key == ord("t"):
                    if not dirtest.active:
                        begin_direction_test(dirtest, state)
                    else:
                        dirtest.active = False
                        with state.lock:
                            state.mode = "LEVEL"
                            state.desired_lr = 0.5
                            state.desired_fb = 0.5
                        set_mode_trackbar("LEVEL")
                        print("[hotkey] dirtest aborted")
                elif key == ord("x"):
                    with state.lock:
                        state.invert_x = not state.invert_x
                        s = state.invert_x
                    pid_x.reset()
                    print(f"[hotkey] invert_x -> {s}")
                elif key == ord("y"):
                    with state.lock:
                        state.invert_y = not state.invert_y
                        s = state.invert_y
                    pid_y.reset()
                    print(f"[hotkey] invert_y -> {s}")
                elif key == ord("z"):
                    # Auto-trim: snap the trim to absorb the current PID
                    # offset, then reset the integrator. The board commands
                    # don't change instantly (because trim absorbs what was
                    # in (desired - 0.5)), but from now on the PID's "0
                    # output" maps to where the board actually balanced.
                    with state.lock:
                        new_trim_lr = max(-TRIM_RANGE, min(
                            TRIM_RANGE,
                            state.trim_lr + (state.desired_lr - 0.5)))
                        new_trim_fb = max(-TRIM_RANGE, min(
                            TRIM_RANGE,
                            state.trim_fb + (state.desired_fb - 0.5)))
                        state.trim_lr = new_trim_lr
                        state.trim_fb = new_trim_fb
                        state.desired_lr = 0.5
                        state.desired_fb = 0.5
                    set_trim_trackbars(new_trim_lr, new_trim_fb)
                    pid_x.reset()
                    pid_y.reset()
                    plotter.clear()
                    print(f"[hotkey] auto-trim z -> "
                          f"({new_trim_lr:+.3f}, {new_trim_fb:+.3f})")

            if n == 0:
                time.sleep(0.0001)

    except KeyboardInterrupt:
        print("\n[main] KeyboardInterrupt -> kill")
        with state.lock:
            state.kill = True
    finally:
        with state.lock:
            if not state.kill and not state.quit:
                state.quit = True
        lab_thread.join(timeout=2.0)
        if lab_thread.is_alive():
            print("[main] labyrinth thread did not exit in time")

        try:
            scamp.send_message(scamp.VS_MSG_HOST_DC, 0, 0)
            time.sleep(0.05)
            scamp.close()
        except Exception as exc:
            print(f"[main] scamp shutdown error: {exc}")
        cv2.destroyAllWindows()
        print("done.")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
