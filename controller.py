import collections.abc
import dataclasses
import threading
import struct

import hid


@dataclasses.dataclass
class ImuSample:
    accelerometer_y: int
    accelerometer_x: int
    accelerometer_z: int
    gyroscope_y: int
    gyroscope_x: int
    gyroscope_z: int


@dataclasses.dataclass
class Report:
    report_identifier: int
    timer: int
    battery_level: int
    connection_status: int
    button_y: bool
    button_x: bool
    button_b: bool
    button_a: bool
    button_right_sr: bool
    button_right_sl: bool
    button_rb: bool
    button_rt: bool
    button_minus: bool
    button_plus: bool
    button_right_stick_click: bool
    button_left_stick_click: bool
    button_home: bool
    button_capture: bool
    charging_grip_attached: bool
    button_down: bool
    button_up: bool
    button_right: bool
    button_left: bool
    button_left_sr: bool
    button_left_sl: bool
    button_lb: bool
    button_lt: bool
    left_stick_x: int
    left_stick_y: int
    right_stick_x: int
    right_stick_y: int
    vibrator_state: int
    imu_samples: list[ImuSample]

    def to_string(self) -> str:
        return " | ".join(
            (
                f"A {1 if self.button_a else 0}",
                f"B {1 if self.button_b else 0}",
                f"X {1 if self.button_x else 0}",
                f"Y {1 if self.button_y else 0}",
                f"↓ {1 if self.button_down else 0}",
                f"↑ {1 if self.button_up else 0}",
                f"→ {1 if self.button_right else 0}",
                f"← {1 if self.button_left else 0}",
                f"RB {1 if self.button_rb else 0}",
                f"RT {1 if self.button_rt else 0}",
                f"LB {1 if self.button_lb else 0}",
                f"LT {1 if self.button_lt else 0}",
                f"RS {1 if self.button_right_stick_click else 0}",
                f"LS {1 if self.button_left_stick_click else 0}",
            )
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "Report":
        imu_samples = []
        for index in range(0, 3):
            sample_data = struct.unpack(
                "<hhhhhh",
                data[13 + (index * 12) : 13 + ((index + 1) * 12)],
            )
            imu_samples.append(
                ImuSample(
                    accelerometer_y=sample_data[0],
                    accelerometer_x=sample_data[1],
                    accelerometer_z=sample_data[2],
                    gyroscope_y=sample_data[3],
                    gyroscope_x=sample_data[4],
                    gyroscope_z=sample_data[5],
                )
            )
        return cls(
            report_identifier=data[0],
            timer=data[1],
            battery_level=data[2] >> 4,
            connection_status=data[2] & 0x0F,
            button_y=((data[3] >> 0) & 1) == 1,
            button_x=((data[3] >> 1) & 1) == 1,
            button_b=((data[3] >> 2) & 1) == 1,
            button_a=((data[3] >> 3) & 1) == 1,
            button_right_sr=((data[3] >> 4) & 1) == 1,
            button_right_sl=((data[3] >> 5) & 1) == 1,
            button_rb=((data[3] >> 6) & 1) == 1,
            button_rt=((data[3] >> 7) & 1) == 1,
            button_minus=((data[4] >> 0) & 1) == 1,
            button_plus=((data[4] >> 1) & 1) == 1,
            button_right_stick_click=((data[4] >> 2) & 1) == 1,
            button_left_stick_click=((data[4] >> 3) & 1) == 1,
            button_home=((data[4] >> 4) & 1) == 1,
            button_capture=((data[4] >> 5) & 1) == 1,
            charging_grip_attached=((data[4] >> 7) & 1) == 1,
            button_down=((data[5] >> 0) & 1) == 1,
            button_up=((data[5] >> 1) & 1) == 1,
            button_right=((data[5] >> 2) & 1) == 1,
            button_left=((data[5] >> 3) & 1) == 1,
            button_left_sr=((data[5] >> 4) & 1) == 1,
            button_left_sl=((data[5] >> 5) & 1) == 1,
            button_lb=((data[5] >> 6) & 1) == 1,
            button_lt=((data[5] >> 7) & 1) == 1,
            left_stick_x=data[6] | ((data[7] & 0x0F) << 8),
            left_stick_y=(data[7] >> 4) | (data[8] << 4),
            right_stick_x=data[9] | ((data[10] & 0x0F) << 8),
            right_stick_y=(data[10] >> 4) | (data[11] << 4),
            vibrator_state=data[12],
            imu_samples=imu_samples,
        )


class Controller:
    def __init__(
        self,
        vid: int,
        pid: int,
        on_report: collections.abc.Callable[[Report], None],
    ):
        self.on_report = on_report
        self.device = hid.Device(vid=vid, pid=pid)
        self.thread = threading.Thread(target=self.target, daemon=True)
        self.thread.start()

    def target(self):
        while True:
            self.on_report(Report.from_bytes(self.device.read(64)))


if __name__ == "__main__":
    import time

    controller = Controller(
        vid=0x057E,
        pid=0x2009,
        on_report=lambda report: print(report.to_string()),
    )
    while True:
        time.sleep(1.0)
