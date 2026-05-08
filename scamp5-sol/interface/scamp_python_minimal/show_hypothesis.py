"""SCAMP-5d hypothesis-bank visualizer (bitstack decoder + colored overlay).

The matching SCAMP-5 kernel packs a per-pixel motion-hypothesis id into the
bitplanes S0..S3 (S0=bit0 ... S3=bit3) and ships them via
scamp5_output_bitstack_begin/bit/end. The Python wrapper does not register a
case_bitstack callback, so the packet arrives here as type='unknown'; we parse
vs_data_header_t plus the four packed bitplanes by hand, recombine them into a
uint8 hypothesis id (0..8), and render with a Middlebury-style color wheel
where H_C (no motion) is black and the eight motion directions ride a hue ring
matching the motion vector.

If the displayed hypothesis map does not align with image edges, the wire
format we are guessing is wrong somewhere. Use the runtime hotkeys to find
the right combination:

    b  toggle bit order (big <-> little) inside each byte
    o  cycle 8-way symmetry (D4 group: flipud, none, fliplr, rot180,
        transpose, T+flipud, T+fliplr, T+rot180)
    p  toggle view: colored hypothesis  <->  S0..S3 raw planes (2x2)
    s  toggle plane order: S0=bit0..S3=bit3  <->  S0=bit3..S3=bit0
    q / ESC  quit

Hypothesis IDs (matches the C++ enum):

    H_C  = 0   no motion (DVS reference)   -> black
    H_R  = 1   right                       -> red
    H_L  = 2   left                        -> cyan
    H_D  = 3   down                        -> violet
    H_U  = 4   up                          -> yellow-green
    H_DR = 5   down-right                  -> magenta-pink
    H_DL = 6   down-left                   -> blue
    H_UR = 7   up-right                    -> orange
    H_UL = 8   up-left                     -> green

Requires:
    pip install opencv-python numpy
"""

import struct
import sys
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

import scamp


# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

USB_SERIAL = "0"
POLL_INTERVAL_S = 0.0001

WINDOW_NAME = "SCAMP hypothesis"
DISPLAY_SCALE = 2
LEGEND_HEIGHT = 36
OVERLAY_HEIGHT = 22

VS_DATA_SCAMP5_BITSTACK = 7  # vs_protocol.hpp


# ----------------------------------------------------------------------
# Orientation symmetries (D4 group, cycle with 'o')
# ----------------------------------------------------------------------

ORIENTATIONS = [
    ("flipud",   lambda im: np.flipud(im)),
    ("none",     lambda im: im),
    ("fliplr",   lambda im: np.fliplr(im)),
    ("rot180",   lambda im: np.rot90(im, 2)),
    ("T",        lambda im: im.T),
    ("T+flipud", lambda im: np.flipud(im.T)),
    ("T+fliplr", lambda im: np.fliplr(im.T)),
    ("T+rot180", lambda im: np.rot90(im.T, 2)),
]


@dataclass
class ViewState:
    bitorder: str = "big"          # 'big' or 'little' inside each byte
    orient_idx: int = 0            # index into ORIENTATIONS
    view_mode: str = "color"       # 'color' or 'planes'
    plane_order_reversed: bool = False  # if True, S0 -> bit3 (instead of bit0)

    @property
    def orient_name(self) -> str:
        return ORIENTATIONS[self.orient_idx][0]

    @property
    def orient_fn(self):
        return ORIENTATIONS[self.orient_idx][1]


STATE = ViewState()


# ----------------------------------------------------------------------
# Bitstack decoding
# ----------------------------------------------------------------------

def decode_bitstack(payload: bytes, bitorder: str = "big") -> dict:
    """Parse a SCAMP-5 BITSTACK payload (after the 8-byte vs_packet_header_t)."""

    # vs_data_header_t: u32 loop_counter, u8 type, u8 version,
    #                   u16 dim_size[3]  (width, height, num_planes),
    #                   u32 channel.   16 bytes, packed little-endian.
    loop_counter, dtype, version, w, h, nplanes, channel = struct.unpack_from(
        "<IBBHHHI", payload, 0
    )
    if dtype != VS_DATA_SCAMP5_BITSTACK:
        raise ValueError(f"not a bitstack packet (data type {dtype})")

    bytes_per_plane = (w * h) // 8
    expected = 16 + nplanes * bytes_per_plane
    if len(payload) < expected:
        raise ValueError(f"payload too small: {len(payload)} < {expected}")

    planes = []
    off = 16
    for _ in range(nplanes):
        chunk = np.frombuffer(
            payload, dtype=np.uint8, count=bytes_per_plane, offset=off
        )
        off += bytes_per_plane
        bits = np.unpackbits(chunk, bitorder=bitorder).reshape((h, w))
        planes.append(bits)

    return {
        "loop_counter": loop_counter,
        "channel": channel,
        "version": version,
        "width": w,
        "height": h,
        "num_planes": nplanes,
        "planes": planes,
    }


def planes_to_hypothesis_id(planes, reversed_order: bool = False) -> np.ndarray:
    """Pack S0..S3 binary planes into a uint8 hypothesis-id image (0..15)."""
    out = np.zeros_like(planes[0], dtype=np.uint8)
    n = len(planes)
    for i, p in enumerate(planes):
        bit = (n - 1 - i) if reversed_order else i
        out |= (p.astype(np.uint8) << bit)
    return out


# ----------------------------------------------------------------------
# Color palette: Middlebury-style optical-flow wheel
# ----------------------------------------------------------------------

HYP_NAMES = ["C", "R", "L", "D", "U", "DR", "DL", "UR", "UL"]

HYP_DIRECTIONS = {
    0: None,
    1: ( 1,  0), 2: (-1,  0),
    3: ( 0,  1), 4: ( 0, -1),
    5: ( 1,  1), 6: (-1,  1),
    7: ( 1, -1), 8: (-1, -1),
}


def make_hypothesis_palette() -> np.ndarray:
    palette = np.zeros((9, 3), dtype=np.uint8)
    for hid, vec in HYP_DIRECTIONS.items():
        if vec is None:
            continue
        dx, dy = vec
        angle_deg = (np.degrees(np.arctan2(-dy, dx)) + 360.0) % 360.0
        hue = angle_deg / 2.0  # OpenCV hue is 0..180
        hsv = np.uint8([[[hue, 255, 255]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        palette[hid] = bgr[0, 0]
    return palette


def colorize_hypothesis(hyp_image: np.ndarray, palette: np.ndarray) -> np.ndarray:
    safe = np.where(hyp_image <= 8, hyp_image, 0)
    return palette[safe]


def make_legend_image(palette: np.ndarray, width: int,
                      height: int = LEGEND_HEIGHT) -> np.ndarray:
    n = palette.shape[0]
    legend = np.zeros((height, width, 3), dtype=np.uint8)
    cell_w = width // n
    for hid in range(n):
        x0 = hid * cell_w
        x1 = (hid + 1) * cell_w if hid < n - 1 else width
        legend[:, x0:x1] = palette[hid]
        b, g, r = palette[hid].tolist()
        lum = 0.114 * b + 0.587 * g + 0.299 * r
        fg = (255, 255, 255) if lum < 110 else (0, 0, 0)
        cx = (x0 + x1) // 2
        cy = height // 3
        vec = HYP_DIRECTIONS[hid]
        if vec is not None:
            dx, dy = vec
            mag = float(np.hypot(dx, dy))
            ax = int(round(cx + 9 * dx / mag))
            ay = int(round(cy + 9 * dy / mag))
            cv2.arrowedLine(legend, (cx, cy), (ax, ay), fg, 1,
                            line_type=cv2.LINE_AA, tipLength=0.45)
        else:
            cv2.circle(legend, (cx, cy), 2, fg, -1, lineType=cv2.LINE_AA)
        label = HYP_NAMES[hid]
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.putText(legend, label, (cx - tw // 2, height - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, fg, 1, cv2.LINE_AA)
    return legend


# ----------------------------------------------------------------------
# Plane view (debug): 2x2 grid of S0..S3 with labels
# ----------------------------------------------------------------------

def make_plane_grid(planes) -> np.ndarray:
    """Compose a 2x2 grid of greyscale planes with text labels overlaid."""
    cells = []
    for i, p in enumerate(planes[:4]):
        cell = (p.astype(np.uint8) * 255)
        cell_bgr = cv2.cvtColor(cell, cv2.COLOR_GRAY2BGR)
        # padded label box
        label = f"S{i}  ({np.count_nonzero(p)} set)"
        cv2.rectangle(cell_bgr, (0, 0), (max(140, len(label) * 9), 18), (0, 0, 0), -1)
        cv2.putText(cell_bgr, label, (4, 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cells.append(cell_bgr)
    top = np.hstack([cells[0], cells[1]])
    bot = np.hstack([cells[2], cells[3]])
    return np.vstack([top, bot])


# ----------------------------------------------------------------------
# Status overlay (FPS + current toggles)
# ----------------------------------------------------------------------

def make_overlay(width: int, fps: float, state: ViewState,
                 frame_lc: int, hyp_breakdown: str) -> np.ndarray:
    bar = np.zeros((OVERLAY_HEIGHT, width, 3), dtype=np.uint8)
    text = (
        f"fps={fps:5.1f}  "
        f"bit={state.bitorder}  "
        f"orient={state.orient_name}  "
        f"order={'REV' if state.plane_order_reversed else 'fwd'}  "
        f"view={state.view_mode}  "
        f"lc={frame_lc}"
    )
    cv2.putText(bar, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 255, 255), 1, cv2.LINE_AA)
    if hyp_breakdown:
        # second line with hypothesis breakdown, drawn smaller and right-aligned
        (tw, _), _ = cv2.getTextSize(hyp_breakdown, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
        if tw < width - 12:
            cv2.putText(bar, hyp_breakdown, (width - tw - 6, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)
    return bar


# ----------------------------------------------------------------------
# State
# ----------------------------------------------------------------------

PALETTE = make_hypothesis_palette()
_legend_cache = {}

frame_times = deque(maxlen=120)   # for moving-average FPS
last_breakdown = ""


def show_frame(planes, hyp_image: np.ndarray, frame_lc: int) -> None:
    """Compose either color view or plane grid + overlay, scale, imshow."""
    global last_breakdown

    w = hyp_image.shape[1]

    if STATE.view_mode == "color":
        body = colorize_hypothesis(hyp_image, PALETTE)
        if w not in _legend_cache:
            _legend_cache[w] = make_legend_image(PALETTE, w)
        body = np.vstack([body, _legend_cache[w]])
    else:
        body = make_plane_grid(planes)

    # FPS from the moving window of frame timestamps
    now = time.perf_counter()
    frame_times.append(now)
    if len(frame_times) >= 2:
        fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
    else:
        fps = 0.0

    # Per-second hypothesis breakdown stays sticky across frames
    if int(now) != int(frame_times[-2]) if len(frame_times) >= 2 else True:
        unique, counts = np.unique(hyp_image, return_counts=True)
        last_breakdown = " ".join(
            f"{HYP_NAMES[u] if u < len(HYP_NAMES) else u}={c}"
            for u, c in zip(unique, counts)
        )

    overlay = make_overlay(body.shape[1], fps, STATE, frame_lc, last_breakdown)
    composite = np.vstack([overlay, body])

    if DISPLAY_SCALE != 1:
        composite = cv2.resize(
            composite,
            (composite.shape[1] * DISPLAY_SCALE, composite.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )

    cv2.imshow(WINDOW_NAME, composite)


def handle_bitstack_packet(payload: bytes) -> None:
    info = decode_bitstack(payload, bitorder=STATE.bitorder)

    # apply the symmetry transform to each plane independently so the plane
    # view also reflects the current orientation choice
    planes = [STATE.orient_fn(p) for p in info["planes"]]
    hyp = planes_to_hypothesis_id(planes, reversed_order=STATE.plane_order_reversed)

    show_frame(planes, hyp, info["loop_counter"])


def handle(packet: dict) -> None:
    if packet.get("type") == "unknown":
        payload = packet.get("payload", b"")
        if len(payload) >= 16 and payload[4] == VS_DATA_SCAMP5_BITSTACK:
            handle_bitstack_packet(payload)
        return
    if packet.get("type") == "data" and packet.get("datatype") == "TEXT":
        text = packet.get("text", "")
        if text:
            lc = packet.get("loopcounter", -1)
            print(f"[{lc}] TEXT: {text!r}")


def drain_packets(max_packets: int = 1000) -> int:
    n = 0
    while n < max_packets:
        packet = scamp.get_packet()
        if packet is None:
            break
        handle(packet)
        n += 1
    return n


# ----------------------------------------------------------------------
# Keyboard handling
# ----------------------------------------------------------------------

def handle_key(key: int) -> bool:
    """Returns False if the user wants to quit."""
    if key in (ord("q"), 27):
        return False
    if key == ord("b"):
        STATE.bitorder = "little" if STATE.bitorder == "big" else "big"
        print(f"[toggle] bitorder = {STATE.bitorder}")
    elif key == ord("o"):
        STATE.orient_idx = (STATE.orient_idx + 1) % len(ORIENTATIONS)
        print(f"[toggle] orient = {STATE.orient_name}")
    elif key == ord("p"):
        STATE.view_mode = "planes" if STATE.view_mode == "color" else "color"
        print(f"[toggle] view = {STATE.view_mode}")
    elif key == ord("s"):
        STATE.plane_order_reversed = not STATE.plane_order_reversed
        print(f"[toggle] plane order reversed = {STATE.plane_order_reversed}")
    return True


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main() -> int:
    print(f"opening USB connection to SCAMP-5d serial={USB_SERIAL!r}...")
    rc = scamp.open_usb(USB_SERIAL)
    if rc != 0:
        print(f"open_usb failed rc={rc}. Is the board plugged in?", file=sys.stderr)
        return 1
    print("connected.")

    scamp.send_message(scamp.VS_MSG_HOST_ON, 0, 0)
    print("hotkeys: b=bitorder  o=orientation  s=plane-order  p=view  q/ESC=quit")
    print("hypothesis ids: 0=C 1=R 2=L 3=D 4=U 5=DR 6=DL 7=UR 8=UL")

    try:
        while True:
            scamp.routine()
            n = drain_packets()

            key = cv2.waitKey(1) & 0xFF
            if key != 255 and not handle_key(key):
                break

            if n == 0:
                time.sleep(POLL_INTERVAL_S)
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
