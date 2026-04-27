import dataclasses
import time
import typing

import serial
import struct


CRC_TABLE: list[int] = [
    0x0000,
    0x8005,
    0x800F,
    0x000A,
    0x801B,
    0x001E,
    0x0014,
    0x8011,
    0x8033,
    0x0036,
    0x003C,
    0x8039,
    0x0028,
    0x802D,
    0x8027,
    0x0022,
    0x8063,
    0x0066,
    0x006C,
    0x8069,
    0x0078,
    0x807D,
    0x8077,
    0x0072,
    0x0050,
    0x8055,
    0x805F,
    0x005A,
    0x804B,
    0x004E,
    0x0044,
    0x8041,
    0x80C3,
    0x00C6,
    0x00CC,
    0x80C9,
    0x00D8,
    0x80DD,
    0x80D7,
    0x00D2,
    0x00F0,
    0x80F5,
    0x80FF,
    0x00FA,
    0x80EB,
    0x00EE,
    0x00E4,
    0x80E1,
    0x00A0,
    0x80A5,
    0x80AF,
    0x00AA,
    0x80BB,
    0x00BE,
    0x00B4,
    0x80B1,
    0x8093,
    0x0096,
    0x009C,
    0x8099,
    0x0088,
    0x808D,
    0x8087,
    0x0082,
    0x8183,
    0x0186,
    0x018C,
    0x8189,
    0x0198,
    0x819D,
    0x8197,
    0x0192,
    0x01B0,
    0x81B5,
    0x81BF,
    0x01BA,
    0x81AB,
    0x01AE,
    0x01A4,
    0x81A1,
    0x01E0,
    0x81E5,
    0x81EF,
    0x01EA,
    0x81FB,
    0x01FE,
    0x01F4,
    0x81F1,
    0x81D3,
    0x01D6,
    0x01DC,
    0x81D9,
    0x01C8,
    0x81CD,
    0x81C7,
    0x01C2,
    0x0140,
    0x8145,
    0x814F,
    0x014A,
    0x815B,
    0x015E,
    0x0154,
    0x8151,
    0x8173,
    0x0176,
    0x017C,
    0x8179,
    0x0168,
    0x816D,
    0x8167,
    0x0162,
    0x8123,
    0x0126,
    0x012C,
    0x8129,
    0x0138,
    0x813D,
    0x8137,
    0x0132,
    0x0110,
    0x8115,
    0x811F,
    0x011A,
    0x810B,
    0x010E,
    0x0104,
    0x8101,
    0x8303,
    0x0306,
    0x030C,
    0x8309,
    0x0318,
    0x831D,
    0x8317,
    0x0312,
    0x0330,
    0x8335,
    0x833F,
    0x033A,
    0x832B,
    0x032E,
    0x0324,
    0x8321,
    0x0360,
    0x8365,
    0x836F,
    0x036A,
    0x837B,
    0x037E,
    0x0374,
    0x8371,
    0x8353,
    0x0356,
    0x035C,
    0x8359,
    0x0348,
    0x834D,
    0x8347,
    0x0342,
    0x03C0,
    0x83C5,
    0x83CF,
    0x03CA,
    0x83DB,
    0x03DE,
    0x03D4,
    0x83D1,
    0x83F3,
    0x03F6,
    0x03FC,
    0x83F9,
    0x03E8,
    0x83ED,
    0x83E7,
    0x03E2,
    0x83A3,
    0x03A6,
    0x03AC,
    0x83A9,
    0x03B8,
    0x83BD,
    0x83B7,
    0x03B2,
    0x0390,
    0x8395,
    0x839F,
    0x039A,
    0x838B,
    0x038E,
    0x0384,
    0x8381,
    0x0280,
    0x8285,
    0x828F,
    0x028A,
    0x829B,
    0x029E,
    0x0294,
    0x8291,
    0x82B3,
    0x02B6,
    0x02BC,
    0x82B9,
    0x02A8,
    0x82AD,
    0x82A7,
    0x02A2,
    0x82E3,
    0x02E6,
    0x02EC,
    0x82E9,
    0x02F8,
    0x82FD,
    0x82F7,
    0x02F2,
    0x02D0,
    0x82D5,
    0x82DF,
    0x02DA,
    0x82CB,
    0x02CE,
    0x02C4,
    0x82C1,
    0x8243,
    0x0246,
    0x024C,
    0x8249,
    0x0258,
    0x825D,
    0x8257,
    0x0252,
    0x0270,
    0x8275,
    0x827F,
    0x027A,
    0x826B,
    0x026E,
    0x0264,
    0x8261,
    0x0220,
    0x8225,
    0x822F,
    0x022A,
    0x823B,
    0x023E,
    0x0234,
    0x8231,
    0x8213,
    0x0216,
    0x021C,
    0x8219,
    0x0208,
    0x820D,
    0x8207,
    0x0202,
]


@dataclasses.dataclass
class Status:
    motor_id: int
    alert: bool
    error: int | None
    data: bytes

    @classmethod
    def from_bytes(cls, data: bytes) -> "Status":
        assert len(data) >= 11, f"{list(data)=}"
        assert data[0:4] == bytes([0xFF, 0xFF, 0xFD, 0x00]), f"{list(data)=}"
        length = data[5] | (data[6] << 8)
        assert len(data) == length + 7, f"{list(data)=}"
        crc = Motor.crc(data[0:-2])
        assert crc == data[-2:], f"{list(data)=}, {crc=}"
        assert data[7] == 85
        return cls(
            motor_id=data[4],
            alert=(data[8] & 0x80) > 0,
            error=(data[8] & 0x64) if (data[8] & 0x40) > 0 else None,
            data=data[9:-2],
        )

    def check(self, expected_motor_id: int):
        assert not self.alert, f"{self}"
        assert self.error is None, f"{self}"
        assert self.motor_id == expected_motor_id, f"{self}, {expected_motor_id=}"


class Motor:
    def __init__(self, motor_id: int):
        self.motor_id = motor_id

    @staticmethod
    def crc(data: bytes) -> bytes:
        result = 0
        for byte in data:
            result = ((result << 8) ^ CRC_TABLE[((result >> 8) ^ byte) & 0xFF]) & 0xFFFF
        return struct.pack("<H", result)

    def compose_write_message(self, address: int, data: bytes) -> bytearray:
        length = len(data) + 5
        message = bytearray(length + 7)
        message[0] = 0xFF
        message[1] = 0xFF
        message[2] = 0xFD
        message[3] = 0x00
        message[4] = self.motor_id
        message[5:7] = struct.pack("<H", length)
        message[7] = 0x03
        message[8:10] = struct.pack("<H", address)
        message[10 : 10 + len(data)] = data
        message[10 + len(data) : 12 + len(data)] = Motor.crc(
            message[0 : 10 + len(data)]
        )
        return message

    def compose_read_message(self, address: int, read_length: int) -> bytearray:
        length = 7
        message = bytearray(length + 7)
        message[0] = 0xFF
        message[1] = 0xFF
        message[2] = 0xFD
        message[3] = 0x00
        message[4] = self.motor_id
        message[5:7] = struct.pack("<H", length)
        message[7] = 0x02
        message[8:10] = struct.pack("<H", address)
        message[10:12] = struct.pack("<H", read_length)
        message[12:14] = Motor.crc(message[0:12])
        return message

    def set_led(
        self,
        openrb150: serial.Serial,
        enable: bool,
        check_status: bool = True,
    ):
        openrb150.write(
            self.compose_write_message(address=65, data=bytes([1 if enable else 0]))
        )
        openrb150.flush()
        status = Status.from_bytes(openrb150.read(11))
        if check_status:
            status.check(expected_motor_id=self.motor_id)

    def set_torque(
        self,
        openrb150: serial.Serial,
        enable: bool,
        check_status: bool = True,
    ):
        openrb150.write(
            self.compose_write_message(address=64, data=bytes([1 if enable else 0]))
        )
        openrb150.flush()
        status = Status.from_bytes(openrb150.read(11))
        if check_status:
            status.check(expected_motor_id=self.motor_id)

    def set_operating_mode(
        self,
        openrb150: serial.Serial,
        mode: typing.Literal["velocity", "position", "extended_position", "pwm"],
        check_status: bool = True,
    ):
        if mode == "velocity":
            mode_value = 1
        elif mode == "position":
            mode_value = 3
        elif mode == "extended_position":
            mode_value = 4
        elif mode == "pwm":
            mode_value = 16
        else:
            raise Exception(f"unknow mode {mode}")
        openrb150.write(
            self.compose_write_message(
                address=11,
                data=bytes([mode_value]),
            )
        )
        openrb150.flush()
        status = Status.from_bytes(openrb150.read(11))
        if check_status:
            status.check(expected_motor_id=self.motor_id)

    def set_position(
        self,
        openrb150: serial.Serial,
        position: int,
        threshold: typing.Optional[int] = None,
        check_status: bool = True,
    ):
        openrb150.write(
            self.compose_write_message(
                address=116,
                data=struct.pack("<I", position),
            )
        )
        openrb150.flush()
        status = Status.from_bytes(openrb150.read(11))
        if check_status:
            status.check(expected_motor_id=self.motor_id)
        if threshold is not None:
            while abs(self.get_position(openrb150=openrb150) - position) > threshold:
                continue

    def get_position(self, openrb150: serial.Serial) -> int:
        openrb150.write(
            self.compose_read_message(
                address=132,
                read_length=4,
            )
        )
        openrb150.flush()
        status = Status.from_bytes(openrb150.read(15))
        return struct.unpack("<I", status.data)[0]

    def set_velocity(
        self,
        openrb150: serial.Serial,
        velocity: int,
        check_status: bool = True,
    ):
        openrb150.write(
            self.compose_write_message(
                address=104,
                data=struct.pack("<i", velocity),
            )
        )
        openrb150.flush()
        status = Status.from_bytes(openrb150.read(11))
        if check_status:
            status.check(expected_motor_id=self.motor_id)

    def stop(self, openrb150: serial.Serial):
        self.set_torque(openrb150=openrb150, enable=False)
        self.set_led(openrb150=openrb150, enable=False)


class Labyrinth:
    def __init__(
        self,
        port: str,
        left_limit: int,
        right_limit: int,
        front_limit: int,
        back_limit: int,
    ) -> None:
        self.left_limit = left_limit
        self.right_limit = right_limit
        self.front_limit = front_limit
        self.back_limit = back_limit
        self.front_motor = Motor(motor_id=1)
        self.right_motor = Motor(motor_id=2)
        self.elevator_motor = Motor(motor_id=3)
        self.openrb150 = serial.Serial(port=port, baudrate=57600, timeout=1.0)
        self.front_motor.set_torque(openrb150=self.openrb150, enable=False)
        self.right_motor.set_torque(openrb150=self.openrb150, enable=False)
        self.elevator_motor.set_torque(openrb150=self.openrb150, enable=False)
        self.front_motor.set_operating_mode(openrb150=self.openrb150, mode="position")
        self.right_motor.set_operating_mode(openrb150=self.openrb150, mode="position")
        self.elevator_motor.set_operating_mode(
            openrb150=self.openrb150, mode="position"
        )
        self.front_motor.set_led(openrb150=self.openrb150, enable=True)
        self.front_motor.set_torque(openrb150=self.openrb150, enable=True)
        self.right_motor.set_led(openrb150=self.openrb150, enable=True)
        self.right_motor.set_torque(openrb150=self.openrb150, enable=True)
        self.elevator_motor.set_led(openrb150=self.openrb150, enable=True)
        self.elevator_motor.set_torque(openrb150=self.openrb150, enable=True)
        elevator_position = self.elevator_motor.get_position(openrb150=self.openrb150)
        self.reload_left = abs(elevator_position - 1024) < abs(elevator_position - 3072)
        if self.reload_left:
            self.elevator_motor.set_position(
                openrb150=self.openrb150, position=1024, threshold=10
            )
        else:
            self.elevator_motor.set_position(
                openrb150=self.openrb150, position=3072, threshold=10
            )

    def stop(self):
        self.front_motor.stop(openrb150=self.openrb150)
        self.right_motor.stop(openrb150=self.openrb150)
        self.elevator_motor.stop(openrb150=self.openrb150)

    def tilt(self, left_right: float, front_back: float, check_status: bool = True):
        assert left_right >= 0.0 and left_right <= 1.0
        assert front_back >= 0.0 and front_back <= 1.0
        self.front_motor.set_position(
            openrb150=self.openrb150,
            position=int(
                round(
                    self.left_limit * (1.0 - left_right) + self.right_limit * left_right
                )
            ),
            check_status=True,
        )
        self.right_motor.set_position(
            openrb150=self.openrb150,
            position=int(
                round(
                    self.front_limit * (1.0 - front_back) + self.back_limit * front_back
                )
            ),
            check_status=True,
        )

    def reload(self, check_status: bool = True):
        if self.reload_left:
            self.elevator_motor.set_position(
                openrb150=self.openrb150,
                position=3072,
                threshold=10,
                check_status=check_status,
            )
            self.reload_left = False
        else:
            self.elevator_motor.set_position(
                openrb150=self.openrb150,
                position=1024,
                threshold=10,
                check_status=check_status,
            )
            self.reload_left = True
