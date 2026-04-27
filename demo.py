import dataclasses
import threading
import time

import neuromorphic_drivers as nd
import numpy as np

import controller as controller_module
import labyrinth as labyrinth_module
import ui

nd.print_device_list()
device = nd.open(iterator_timeout=1.0 / 60.0)
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
            tau: 100000
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
    lock: threading.Lock

    def update_from_report(self, report: controller_module.Report):
        with self.lock:
            self.front_back = report.left_stick_y / 4095
            self.left_right = report.left_stick_x / 4095
            if report.button_rb:
                self.reload = True
            if report.button_a or report.button_b or report.button_x or report.button_y:
                self.stop = True
                app.quit()


target = Target(
    left_right=0.5,
    front_back=0.5,
    reload=False,
    stop=False,
    lock=threading.Lock(),
)
controller = controller_module.Controller(
    vid=0x057E,
    pid=0x2009,
    on_report=lambda report: target.update_from_report(report),
)


def labyrinth_target():
    labyrinth = labyrinth_module.Labyrinth(
        port="/dev/tty.usbmodem101",
        left_limit=2048 - 800,
        right_limit=2048 + 800,
        front_limit=2048 + 500 - 800,
        back_limit=2048 + 500 + 800,
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
    for status, packet in device:
        with target.lock:
            if target.stop:
                break
        if packet is not None:
            if packet.polarity_events is not None:
                assert status.ring is not None and status.ring.current_t is not None
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
