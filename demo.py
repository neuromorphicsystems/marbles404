import dataclasses
import datetime
import pathlib
import threading
import time
import typing

import faery
import neuromorphic_drivers as nd
import numpy as np

import controller as controller_module
import labyrinth as labyrinth_module
import ui

recordings = pathlib.Path(__file__).resolve().parent / "recordings"
recordings.mkdir(exist_ok=True)

nd.print_device_list()
device = nd.open(
    configuration=nd.prophesee_evk4.Configuration(
        biases=nd.prophesee_evk4.Biases(
            diff_on=120,  # default 102
            diff_off=80,  # default 73
        )
    ),
    iterator_timeout=1.0 / 60.0,
)
app = ui.App(
    f"""
    import QtQuick
    import NeuromorphicDrivers

    Window {{
        id: window
        width: {device.properties().width}
        height: {device.properties().height}

        EventDisplay {{
            width: window.width
            height: window.height
            sensor_size: "{device.properties().width}x{device.properties().height}"
            style: "exponential"
            tau: 60000
        }}
    }}
    """
)


@dataclasses.dataclass
class Target:
    left_right: float
    front_back: float
    reload: bool
    stop: bool
    recording: typing.Optional[str]
    recording_update: int
    lock: threading.Lock

    def update_from_report(self, report: controller_module.Report):
        with self.lock:
            self.front_back = report.left_stick_y / 4095
            self.left_right = report.left_stick_x / 4095
            if report.button_rb:
                self.reload = True
            if report.button_lb:
                now = time.monotonic_ns()
                if now - self.recording_update > 500000000:  # 500 ms
                    self.recording_update = now
                    if self.recording is None:
                        self.recording = (
                            datetime.datetime.now(tz=datetime.timezone.utc)
                            .isoformat()
                            .replace("+00:00", "Z")
                            .replace(":", "-")
                        )
                    else:
                        self.recording = None
            if report.button_a or report.button_b or report.button_x or report.button_y:
                self.stop = True
                app.quit()


target = Target(
    left_right=0.5,
    front_back=0.5,
    reload=False,
    stop=False,
    recording=None,
    recording_update=0,
    lock=threading.Lock(),
)
controller = controller_module.Controller(
    vid=0x057E,
    pid=0x2009,
    on_report=lambda report: target.update_from_report(report),
)


def labyrinth_target():
    labyrinth = labyrinth_module.Labyrinth(
        port="/dev/tty.usbmodem1101",
        left_limit=2543 - 900,
        right_limit=2543 + 900,
        front_limit=2058 - 700,
        back_limit=2058 + 700,
    )
    while True:
        with target.lock:
            left_right = target.left_right
            front_back = target.front_back
            reload = target.reload
            stop = target.stop
        if stop:
            break
        if reload:
            labyrinth.tilt(left_right=0.5, front_back=0.5)
            labyrinth.reload()
            with target.lock:
                target.reload = False
        labyrinth.tilt(left_right=left_right, front_back=front_back)
    labyrinth.stop()


labyrinth_thread = threading.Thread(target=labyrinth_target, daemon=True)


def camera_thread_target(
    device: nd.GenericDeviceOptional,
    event_display: ui.EventDisplay,
):
    recording: typing.Optional[tuple[str, faery.evt.Encoder]] = None
    for status, packet in device:
        with target.lock:
            if target.stop:
                break
            if recording is None and target.recording is not None:
                print(f"start recording to {target.recording}")
                recording = (
                    str(target.recording),
                    faery.evt.Encoder(
                        recordings / f"{target.recording}.raw",
                        "evt3",
                        True,
                        (1280, 720),
                        True,
                    ),
                )
            elif recording is not None and target.recording is None:
                print(f"stop recording to {recording[0]}")
                recording[1].__exit__(None, None, None)
                recording = None
            elif (
                recording is not None
                and target.recording is not None
                and recording[0] != target.recording
            ):
                print(f"stop recording to {recording[0]}")
                recording[1].__exit__(None, None, None)
                print(f"start recording to {target.recording}")
                recording = (
                    str(target.recording),
                    faery.evt.Encoder(
                        recordings / f"{target.recording}.raw",
                        "evt3",
                        True,
                        (1280, 720),
                        True,
                    ),
                )
        if packet is not None:
            if packet.polarity_events is not None:
                assert status.ring is not None and status.ring.current_t is not None
                if recording is not None:
                    recording[1].write({"events": packet.polarity_events})
                event_display.push(
                    events=packet.polarity_events, current_t=status.ring.current_t
                )
            elif status.ring is not None and status.ring.current_t is not None:
                event_display.push(events=np.array([]), current_t=status.ring.current_t)


event_display = app.event_display()
camera_thread = threading.Thread(
    target=camera_thread_target,
    args=(device, event_display),
)

labyrinth_thread.start()
camera_thread.start()
app.run()
labyrinth_thread.join()
camera_thread.join()
