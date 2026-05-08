"""Windows-friendly labyrinth tilt control via the keyboard.

Derivative of demo.py that drops the camera, controller, and Qt UI and talks
only to the OpenRB-150 motor driver over USB CDC serial. Intended as a smoke
test that the labyrinth half of the demo runs on Windows.

Usage:
    python labyrinth_demo.py                  # auto-detect COM port
    python labyrinth_demo.py --port COM5      # explicit port
    python labyrinth_demo.py --list-ports     # show all serial ports + descriptions

Controls (in the running app):
    W / Up         tilt away (front_back -)
    S / Down       tilt toward (front_back +)
    A / Left       tilt left  (left_right -)
    D / Right      tilt right (left_right +)
    C              re-center (0.5, 0.5)
    R              reload marble
    Q / Esc        quit
"""

import argparse
import sys
import time
import typing

import serial
import serial.tools.list_ports

import labyrinth as labyrinth_module


# 1826, 2215, 

MAX_ROT = 600

# Calibration constants copied from demo.py:130-141. If your rig was
# calibrated differently, adjust these.
LEFT_LIMIT = 2244 - MAX_ROT
RIGHT_LIMIT = 2244 + MAX_ROT
FRONT_LIMIT = 2091 - MAX_ROT
BACK_LIMIT = 2091 + MAX_ROT

TILT_STEP = 0.05            # fraction of full range per key press
LOOP_PERIOD_S = 0.01

# Heuristic strings that suggest a port is the OpenRB-150. The board enumerates
# as a generic USB CDC device on Windows, so the description is often something
# like "USB Serial Device (COM5)" or "OpenRB-150".
PORT_HINTS = ("openrb", "usb serial", "samd", "robotis", "arduino")


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

def list_ports_verbose() -> list:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("(no serial ports found)")
        return ports
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:<8}  desc={p.description!r:<40}  hwid={p.hwid}")
    return ports


def autodetect_port() -> typing.Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    if len(ports) == 1:
        return ports[0].device
    scored = []
    for p in ports:
        desc = (p.description or "").lower()
        score = sum(1 for hint in PORT_HINTS if hint in desc)
        scored.append((score, p))
    scored.sort(key=lambda t: -t[0])
    if scored[0][0] > 0 and scored[0][0] > scored[1][0]:
        return scored[0][1].device
    return None


# ---------------------------------------------------------------------------
# Keyboard input (Windows uses msvcrt; POSIX fallback uses termios+select)
# ---------------------------------------------------------------------------

if sys.platform == "win32":
    import msvcrt

    def setup_terminal() -> None:
        pass  # nothing to do on Windows

    def restore_terminal() -> None:
        pass

    def read_key() -> typing.Optional[str]:
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
        # Arrow keys arrive as 0xE0 (or 0x00) prefix + scancode.
        if ch in (b"\xe0", b"\x00"):
            if msvcrt.kbhit():
                code = msvcrt.getch()
                return {
                    b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT",
                }.get(code)
            return None
        if ch == b"\x1b":
            return "ESC"
        try:
            return ch.decode("ascii").lower()
        except UnicodeDecodeError:
            return None

else:
    import select
    import termios
    import tty

    _saved_termios: typing.Optional[list] = None

    def setup_terminal() -> None:
        global _saved_termios
        _saved_termios = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal() -> None:
        if _saved_termios is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _saved_termios)

    def read_key() -> typing.Optional[str]:
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            if select.select([sys.stdin], [], [], 0.001)[0]:
                seq = sys.stdin.read(2)
                return {"[A": "UP", "[B": "DOWN", "[D": "LEFT", "[C": "RIGHT"}.get(seq, "ESC")
            return "ESC"
        return ch.lower()


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Keyboard-driven labyrinth tilt control (Windows-friendly).")
    p.add_argument("--port", help="Serial port, e.g. COM5. If omitted, auto-detect.")
    p.add_argument("--list-ports", action="store_true", help="List serial ports and exit.")
    p.add_argument("--step", type=float, default=TILT_STEP, help=f"Tilt fraction per key press (default {TILT_STEP}).")
    return p.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.list_ports:
        list_ports_verbose()
        return 0

    port = args.port or autodetect_port()
    if port is None:
        print("Could not auto-detect a single matching serial port.")
        list_ports_verbose()
        print("Re-run with --port COMx.")
        return 1

    print(f"opening labyrinth on {port}...")
    try:
        labyrinth = labyrinth_module.Labyrinth(
            port=port,
            left_limit=LEFT_LIMIT,
            right_limit=RIGHT_LIMIT,
            front_limit=FRONT_LIMIT,
            back_limit=BACK_LIMIT,
        )
    except (serial.SerialException, OSError) as exc:
        print(f"failed to open {port}: {exc}")
        return 1
    except AssertionError as exc:
        print(f"motor handshake failed on {port}: {exc}")
        print("(check that the board is powered, motors are wired, and IDs are 1/2/3)")
        return 1
    print("labyrinth ready.")

    print()
    print("controls: W/A/S/D or arrows = tilt, C = center, R = reload, Q/Esc = quit")
    print()

    left_right = 0.5
    front_back = 0.5
    last_lr: typing.Optional[float] = None
    last_fb: typing.Optional[float] = None

    setup_terminal()
    try:
        while True:
            key = read_key()
            if key is not None:
                if key in ("q", "ESC"):
                    break
                elif key in ("w", "UP"):
                    front_back = clamp(front_back - args.step, 0.0, 1.0)
                elif key in ("s", "DOWN"):
                    front_back = clamp(front_back + args.step, 0.0, 1.0)
                elif key in ("a", "LEFT"):
                    left_right = clamp(left_right - args.step, 0.0, 1.0)
                elif key in ("d", "RIGHT"):
                    left_right = clamp(left_right + args.step, 0.0, 1.0)
                elif key == "c":
                    left_right, front_back = 0.5, 0.5
                elif key == "r":
                    print("\r[reload] centering, then reloading...                ", flush=True)
                    try:
                        labyrinth.tilt(0.5, 0.5)
                        labyrinth.reload()
                    except Exception as exc:
                        print(f"\nreload failed: {exc}")
                    last_lr, last_fb = 0.5, 0.5

            if left_right != last_lr or front_back != last_fb:
                try:
                    labyrinth.tilt(left_right=left_right, front_back=front_back)
                except Exception as exc:
                    print(f"\ntilt failed: {exc}")
                    break
                last_lr, last_fb = left_right, front_back
                print(f"\rtilt: lr={left_right:.2f}  fb={front_back:.2f}    ", end="", flush=True)

            time.sleep(LOOP_PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        restore_terminal()
        print("\nstopping motors...")
        try:
            labyrinth.stop()
        except Exception as exc:
            print(f"  stop failed: {exc}")
        print("done.")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
