"""Low-latency SCAMP-5d packet capture + OpenCV ball tracker.

Expected SCAMP output channels:

    TX / selected map  -> channel 100
    RAW_IMAGE          -> channel 101
    BOARD              -> channel 102
    MAP                -> channel 103
    BALL_CAND          -> channel 104

Keyboard:
    b   trigger SCAMP "build map"
    d   toggle Python debug windows
    s   toggle SCAMP-side debug sending, if DEBUG_GUI_ITEM_ID is valid
    p   toggle printing
    q   quit
    ESC quit

Requires:
    pip install pillow opencv-python numpy
"""

import sys
import time
from pathlib import Path

import cv2
import numpy as np
import scamp


# ----------------------------------------------------------------------
# SCAMP channel IDs
# ----------------------------------------------------------------------

TX = 100
RAW_IMAGE = 101
BOARD = 102
MAP = 103
BALL_CAND = 104

CHANNEL_NAMES = {
    TX: "TX / selected map",
    RAW_IMAGE: "RAW_IMAGE",
    BOARD: "BOARD",
    MAP: "MAP",
    BALL_CAND: "BALL_CAND",
}


# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

USB_SERIAL = "0"

POLL_INTERVAL_S = 0.0001

DISPLAY = True

BUILD_MAP_ITEM_ID = 0

# Set this to the GUI item id of the SCAMP-side debug slider:
#
#     int show_debug_displays = 0;
#     vs_gui_add_slider("show debug displays", 0, 1, show_debug_displays, &show_debug_displays);
#
# If unknown, leave as None.
DEBUG_GUI_ITEM_ID = None
# DEBUG_GUI_ITEM_ID = 12

AUTO_BUILD_MAP_ON_STARTUP = True

# Use TX if your SCAMP tx_mode sends BALL_CAND on selected map.
TRACK_CHANNEL = TX
# TRACK_CHANNEL = BALL_CAND

# Low-latency settings
DEBUG_DISPLAY_ENABLED = False
SCAMP_DEBUG_ENABLED = False
PRINT_TRACKING = False

# Debug windows update rate
DEBUG_DISPLAY_PERIOD_S = 0.2   # 5 Hz

# Main tracking window update rate
TRACK_DISPLAY_PERIOD_S = 0.0   # 0 means display every newest tracking frame


# ----------------------------------------------------------------------
# Ball tracker
# ----------------------------------------------------------------------

class BallTracker:
    def __init__(self):
        self.prev_xy = None
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

        # Morphology is useful, but it costs time.
        # If your SCAMP binary map is clean, set this False.
        self.use_morphology = True

        self.kernel = np.ones((3, 3), np.uint8)

    def update(self, img_u8):
        mask = (img_u8 > 0).astype(np.uint8) * 255

        if self.use_morphology:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        n, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask,
            connectivity=8,
        )

        best = None
        best_score = -1e18

        for label in range(1, n):
            x, y, w, h, area = stats[label]
            cx, cy = centroids[label]

            if area < self.min_area or area > self.max_area:
                continue

            if w < self.min_w or h < self.min_h:
                continue

            if w > self.max_w or h > self.max_h:
                continue

            aspect = w / max(h, 1)

            if aspect < self.min_aspect or aspect > self.max_aspect:
                continue

            score = float(area)

            if self.prev_xy is not None:
                px, py = self.prev_xy
                dist = abs(cx - px) + abs(cy - py)
                score -= self.distance_penalty * dist

            if score > best_score:
                best_score = score
                best = {
                    "xy": (int(round(cx)), int(round(cy))),
                    "bbox": (int(x), int(y), int(w), int(h)),
                    "area": int(area),
                    "score": float(score),
                    "mask": mask,
                    "num_components": int(n - 1),
                }

        if best is None:
            self.lost_count += 1
            return None

        self.prev_xy = best["xy"]
        self.lost_count = 0
        return best


tracker = BallTracker()


# ----------------------------------------------------------------------
# State
# ----------------------------------------------------------------------

latest_frames = {}
latest_tracking_result = None

running = True

last_debug_display_time = 0.0
last_track_display_time = 0.0

processed_track_frames = 0
dropped_nontrack_packets = 0


# ----------------------------------------------------------------------
# Fast packet decoding
# ----------------------------------------------------------------------

def packet_to_numpy_fast(packet):
    """
    Fast replacement for PIL Image.frombytes + transpose.

    Original code did:
        FLIP_LEFT_RIGHT
        ROTATE_180

    That combination is equivalent to vertical flip.
    If the orientation is wrong, replace with one of:
        np.flipud(img)
        np.fliplr(img)
        np.rot90(img, 2)
    """

    w, h = packet["width"], packet["height"]

    img = np.frombuffer(packet["buffer"], dtype=np.uint8).reshape((h, w))

    # Equivalent to FLIP_LEFT_RIGHT then ROTATE_180:
    img = np.flipud(img)

    # Make contiguous for OpenCV.
    return np.ascontiguousarray(img)


# ----------------------------------------------------------------------
# Host controls
# ----------------------------------------------------------------------

def trigger_build_map():
    print(f"[host] triggering build map: gui id={BUILD_MAP_ITEM_ID}")
    scamp.send_gui_value(BUILD_MAP_ITEM_ID, 1)


def set_scamp_debug(enabled):
    global SCAMP_DEBUG_ENABLED

    SCAMP_DEBUG_ENABLED = bool(enabled)

    if DEBUG_GUI_ITEM_ID is None:
        print("[host] DEBUG_GUI_ITEM_ID is None; only local debug display toggled")
        return

    value = 1 if SCAMP_DEBUG_ENABLED else 0
    print(f"[host] setting SCAMP debug displays: id={DEBUG_GUI_ITEM_ID}, value={value}")
    scamp.send_gui_value(DEBUG_GUI_ITEM_ID, value)


def toggle_scamp_debug():
    set_scamp_debug(not SCAMP_DEBUG_ENABLED)


# ----------------------------------------------------------------------
# Display helpers
# ----------------------------------------------------------------------

def draw_tracking_debug(img_u8, result):
    """
    Display only the tracked ball hypothesis, not the candidate pixels.
    """

    h, w = img_u8.shape[:2]

    # Blank black display
    vis = np.zeros((h, w, 3), dtype=np.uint8)

    if result is not None:
        x, y = result["xy"]
        bx, by, bw, bh = result["bbox"]

        # Draw only the selected hypothesis
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 1)
        cv2.circle(vis, (x, y), 3, (0, 0, 255), -1)

        text = f"x={x} y={y} area={result['area']}"

        cv2.putText(
            vis,
            text,
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            vis,
            f"ball lost: {tracker.lost_count}",
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    return vis

def make_display_image(ch, img_u8):
    if ch in (TX, BOARD, MAP, BALL_CAND):
        vis_gray = (img_u8 > 0).astype(np.uint8) * 255
    else:
        vis_gray = img_u8

    vis = cv2.cvtColor(vis_gray, cv2.COLOR_GRAY2BGR)

    name = CHANNEL_NAMES.get(ch, f"channel {ch}")

    cv2.putText(
        vis,
        name,
        (5, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    return vis


def display_tracking_window():
    global last_track_display_time

    if not DISPLAY:
        return

    now = time.perf_counter()

    if TRACK_DISPLAY_PERIOD_S > 0:
        if now - last_track_display_time < TRACK_DISPLAY_PERIOD_S:
            return

    if TRACK_CHANNEL not in latest_frames:
        return

    img_u8 = latest_frames[TRACK_CHANNEL]
    vis = draw_tracking_debug(img_u8, latest_tracking_result)
    cv2.imshow("LOW LATENCY BALL TRACKING", vis)

    last_track_display_time = now


def display_debug_windows():
    global last_debug_display_time

    if not DISPLAY or not DEBUG_DISPLAY_ENABLED:
        return

    now = time.perf_counter()

    if now - last_debug_display_time < DEBUG_DISPLAY_PERIOD_S:
        return

    for ch in (RAW_IMAGE, BOARD, MAP, BALL_CAND):
        if ch not in latest_frames:
            continue

        img_u8 = latest_frames[ch]
        window_name = CHANNEL_NAMES.get(ch, f"channel {ch}")

        vis = make_display_image(ch, img_u8)
        cv2.imshow(window_name, vis)

    last_debug_display_time = now


def handle_keyboard():
    global running
    global DEBUG_DISPLAY_ENABLED
    global PRINT_TRACKING

    key = cv2.waitKey(1) & 0xFF

    if key == 255:
        return

    if key == ord("b"):
        trigger_build_map()

    elif key == ord("d"):
        DEBUG_DISPLAY_ENABLED = not DEBUG_DISPLAY_ENABLED
        print(f"[host] local debug display: {DEBUG_DISPLAY_ENABLED}")

        if not DEBUG_DISPLAY_ENABLED:
            for ch in (RAW_IMAGE, BOARD, MAP, BALL_CAND):
                name = CHANNEL_NAMES.get(ch, f"channel {ch}")
                try:
                    cv2.destroyWindow(name)
                except cv2.error:
                    pass

    elif key == ord("s"):
        toggle_scamp_debug()

    elif key == ord("p"):
        PRINT_TRACKING = not PRINT_TRACKING
        print(f"[host] print tracking: {PRINT_TRACKING}")

    elif key == ord("q") or key == 27:
        running = False


# ----------------------------------------------------------------------
# Packet handling
# ----------------------------------------------------------------------

def handle_image_packet(packet):
    global latest_tracking_result
    global processed_track_frames
    global dropped_nontrack_packets

    ch = packet["channel"]

    # Low-latency rule:
    # - Always decode tracking channel.
    # - Decode debug channels only if debug display is enabled.
    # - Otherwise ignore them quickly.
    if ch != TRACK_CHANNEL and not DEBUG_DISPLAY_ENABLED:
        dropped_nontrack_packets += 1
        return

    img_u8 = packet_to_numpy_fast(packet)
    latest_frames[ch] = img_u8

    if ch == TRACK_CHANNEL:
        latest_tracking_result = tracker.update(img_u8)
        processed_track_frames += 1

        if PRINT_TRACKING:
            lc = packet["loopcounter"]
            if latest_tracking_result is None:
                print(f"[{lc}] ball lost lost_count={tracker.lost_count}")
            else:
                x, y = latest_tracking_result["xy"]
                bbox = latest_tracking_result["bbox"]
                area = latest_tracking_result["area"]
                print(f"[{lc}] ball x={x} y={y} bbox={bbox} area={area}")


def handle(packet):
    if packet.get("type") != "data":
        return

    dt = packet["datatype"]

    if dt == "TEXT":
        # Text printing can create latency if SCAMP posts every frame.
        text = packet.get("text", "")
        if text and ("BUILD_MAP" in text or "ERROR" in text):
            lc = packet.get("loopcounter", -1)
            print(f"[{lc}] TEXT: {text!r}")
        return

    if dt in ("SCAMP5_AOUT", "SCAMP5_DOUT"):
        handle_image_packet(packet)
        return

    # Ignore other packet types in low-latency mode.
    return


def drain_packets(max_packets=1000):
    """
    Drain all currently queued packets.

    This is important. If you process one packet per loop while SCAMP produces
    multiple packets per frame, you build latency forever.
    """

    n = 0

    while n < max_packets:
        packet = scamp.get_packet()

        if packet is None:
            break

        handle(packet)
        n += 1

    return n


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    global running

    print(f"opening USB connection to SCAMP-5d serial={USB_SERIAL!r}...")
    rc = scamp.open_usb(USB_SERIAL)

    if rc != 0:
        print(f"open_usb failed rc={rc}. Is the board plugged in?", file=sys.stderr)
        return 1

    print("connected.")

    # Tell chip host is connected, so vs_gui_is_on() returns true.
    scamp.send_message(scamp.VS_MSG_HOST_ON, 0, 0)

    # Start with SCAMP-side debug disabled, if available.
    if DEBUG_GUI_ITEM_ID is not None:
        set_scamp_debug(False)

    print(f"tracking channel: {TRACK_CHANNEL}")
    print(f"build-map GUI item id: {BUILD_MAP_ITEM_ID}")
    print("keyboard: b=build map, d=local debug, s=scamp debug, p=print, q/ESC=quit")

    if AUTO_BUILD_MAP_ON_STARTUP:
        time.sleep(0.05)
        trigger_build_map()

    last_stats_time = time.perf_counter()
    last_processed_count = 0

    try:
        while running:
            scamp.routine()

            n = drain_packets()

            if n == 0:
                time.sleep(POLL_INTERVAL_S)

            display_tracking_window()
            display_debug_windows()
            handle_keyboard()

            # Occasional stats, not every frame.
            now = time.perf_counter()
            if now - last_stats_time > 2.0:
                fps = (processed_track_frames - last_processed_count) / (now - last_stats_time)
                last_processed_count = processed_track_frames
                last_stats_time = now

                print(
                    f"[stats] tracking_fps={fps:.1f} "
                    f"lost={tracker.lost_count} "
                    f"debug_local={DEBUG_DISPLAY_ENABLED} "
                    f"debug_scamp={SCAMP_DEBUG_ENABLED} "
                    f"dropped_nontrack={dropped_nontrack_packets}"
                )

    except KeyboardInterrupt:
        print("\nstopping.")

    finally:
        scamp.send_message(scamp.VS_MSG_HOST_DC, 0, 0)
        time.sleep(0.05)
        scamp.close()
        cv2.destroyAllWindows()
        print("closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())