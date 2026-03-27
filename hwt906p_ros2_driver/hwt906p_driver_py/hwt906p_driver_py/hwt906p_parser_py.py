import math
from dataclasses import dataclass
from typing import List, Optional


kGravity = 9.80665
kDegToRad = math.pi / 180.0
kQuatScale = 32768.0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class ImuSample:
    acceleration: Optional[Vector3] = None
    angular_velocity: Optional[Vector3] = None
    euler_deg: Optional[Vector3] = None
    quaternion_xyzw: Optional[Quaternion] = None
    magnetic_field_raw: Optional[Vector3] = None
    temperature_c: Optional[float] = None


@dataclass
class ParseStats:
    valid_frames: int = 0
    emitted_samples: int = 0
    unknown_frames: int = 0
    last_frame_type: Optional[int] = None


@dataclass
class FeedResult:
    samples: List[ImuSample]
    stats: ParseStats


class PendingState:
    def __init__(self) -> None:
        self.acceleration: Optional[Vector3] = None
        self.angular_velocity: Optional[Vector3] = None
        self.euler_deg: Optional[Vector3] = None
        self.quaternion_xyzw: Optional[Quaternion] = None
        self.magnetic_field_raw: Optional[Vector3] = None
        self.temperature_c: Optional[float] = None


class Hwt906pParserPy:
    FRAME_HEADER = 0x55
    FRAME_LENGTH = 11

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.pending_state = PendingState()
        self.fresh_acceleration = False
        self.fresh_angular_velocity = False
        self.fresh_euler = False
        self.fresh_quaternion = False

    def reset(self) -> None:
        self.buffer.clear()
        self.pending_state = PendingState()
        self.fresh_acceleration = False
        self.fresh_angular_velocity = False
        self.fresh_euler = False
        self.fresh_quaternion = False

    def feed(self, chunk: bytes) -> FeedResult:
        self.buffer.extend(chunk)
        result = FeedResult(samples=[], stats=ParseStats())

        while len(self.buffer) >= self.FRAME_LENGTH:
            if self.buffer[0] != self.FRAME_HEADER:
                del self.buffer[0]
                continue

            checksum = sum(self.buffer[: self.FRAME_LENGTH - 1]) & 0xFF
            if checksum != self.buffer[self.FRAME_LENGTH - 1]:
                del self.buffer[0]
                continue

            frame = bytes(self.buffer[: self.FRAME_LENGTH])
            del self.buffer[: self.FRAME_LENGTH]

            result.stats.valid_frames += 1
            result.stats.last_frame_type = frame[1]

            sample = self.parse_frame(frame)
            if sample is not None:
                result.samples.append(sample)
                result.stats.emitted_samples += 1
            else:
                result.stats.unknown_frames += 1

        return result

    def parse_frame(self, frame: bytes) -> Optional[ImuSample]:
        frame_type = frame[1]
        data = frame[2:]

        if frame_type == 0x51:
            sample = self.maybe_emit_sample(frame_type)
            self.pending_state.acceleration = Vector3(
                self._to_int16(data[0], data[1]) / 32768.0 * 16.0 * kGravity,
                self._to_int16(data[2], data[3]) / 32768.0 * 16.0 * kGravity,
                self._to_int16(data[4], data[5]) / 32768.0 * 16.0 * kGravity,
            )
            self.pending_state.temperature_c = self._to_int16(data[6], data[7]) / 100.0
            self.fresh_acceleration = True
            return sample

        if frame_type == 0x52:
            self.pending_state.angular_velocity = Vector3(
                self._to_int16(data[0], data[1]) / 32768.0 * 2000.0 * kDegToRad,
                self._to_int16(data[2], data[3]) / 32768.0 * 2000.0 * kDegToRad,
                self._to_int16(data[4], data[5]) / 32768.0 * 2000.0 * kDegToRad,
            )
            self.pending_state.temperature_c = self._to_int16(data[6], data[7]) / 100.0
            self.fresh_angular_velocity = True
            return None

        if frame_type == 0x53:
            self.pending_state.euler_deg = Vector3(
                self._to_int16(data[0], data[1]) / 32768.0 * 180.0,
                self._to_int16(data[2], data[3]) / 32768.0 * 180.0,
                self._to_int16(data[4], data[5]) / 32768.0 * 180.0,
            )
            self.fresh_euler = True
            return None

        if frame_type == 0x54:
            self.pending_state.magnetic_field_raw = Vector3(
                float(self._to_int16(data[0], data[1])),
                float(self._to_int16(data[2], data[3])),
                float(self._to_int16(data[4], data[5])),
            )
            return None

        if frame_type == 0x59:
            q0 = self._to_int16(data[0], data[1]) / kQuatScale
            q1 = self._to_int16(data[2], data[3]) / kQuatScale
            q2 = self._to_int16(data[4], data[5]) / kQuatScale
            q3 = self._to_int16(data[6], data[7]) / kQuatScale
            self.pending_state.quaternion_xyzw = Quaternion(q1, q2, q3, q0)
            self.fresh_quaternion = True
            return self.maybe_emit_sample(frame_type)

        return None

    def maybe_emit_sample(self, frame_type: int) -> Optional[ImuSample]:
        if not self.fresh_acceleration or not self.fresh_angular_velocity:
            return None

        should_emit_from_quaternion = frame_type == 0x59 and self.fresh_quaternion
        should_emit_from_euler = frame_type == 0x51 and self.fresh_euler and not self.fresh_quaternion
        if not should_emit_from_quaternion and not should_emit_from_euler:
            return None

        sample = ImuSample(
            acceleration=self.pending_state.acceleration,
            angular_velocity=self.pending_state.angular_velocity,
            temperature_c=self.pending_state.temperature_c,
            magnetic_field_raw=self.pending_state.magnetic_field_raw,
        )
        if should_emit_from_quaternion:
            sample.quaternion_xyzw = self.pending_state.quaternion_xyzw
        else:
            sample.euler_deg = self.pending_state.euler_deg

        self.fresh_acceleration = False
        self.fresh_angular_velocity = False
        self.fresh_euler = False
        self.fresh_quaternion = False
        return sample

    @staticmethod
    def _to_int16(lo: int, hi: int) -> int:
        value = ((hi << 8) | lo) & 0xFFFF
        return value - 0x10000 if value >= 0x8000 else value
