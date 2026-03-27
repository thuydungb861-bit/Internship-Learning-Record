import math
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
from serial import SerialException

from hwt906p_driver_py.hwt906p_parser_py import Hwt906pParserPy, ImuSample, Quaternion, Vector3, kDegToRad


DEFAULT_ORIENTATION_COVARIANCE = [
    5.250769e-09, 1.790648e-09, 0.0,
    1.790648e-09, 6.129790e-09, 0.0,
    0.0, 0.0, 1.0e-06,
]

DEFAULT_ANGULAR_VELOCITY_COVARIANCE = [
    1.137772e-08, 1.599640e-08, 0.0,
    1.599640e-08, 2.523375e-08, 0.0,
    0.0, 0.0, 1.0e-08,
]

DEFAULT_LINEAR_ACCELERATION_COVARIANCE = [
    1.181765e-05, -1.705412e-07, 4.447220e-06,
    -1.705412e-07, 1.245834e-05, -2.611583e-06,
    4.447220e-06, -2.611583e-06, 3.182327e-05,
]

MAGNETIC_FIELD_COVARIANCE = [
    1.0e-6, 0.0, 0.0,
    0.0, 1.0e-6, 0.0,
    0.0, 0.0, 1.0e-6,
]


def euler_to_quaternion(euler_deg: Vector3) -> Quaternion:
    roll = euler_deg.x * kDegToRad
    pitch = euler_deg.y * kDegToRad
    yaw = euler_deg.z * kDegToRad

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return Quaternion(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

def preview_bytes(chunk: bytes, max_bytes: int) -> str:
    return ' '.join(f'{byte:02x}' for byte in chunk[:max_bytes])


class Hwt906pNodePy(Node):
    def __init__(self) -> None:
        super().__init__('hwt906p_node_py')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('poll_hz', 200.0)
        self.declare_parameter('magnetic_field_lsb_to_tesla', 1.0e-6)
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('frame_timeout_sec', 2.0)
        self.declare_parameter('status_log_interval_sec', 2.0)
        self.declare_parameter('baudrate_candidates', [230400, 115200, 9600, 57600, 38400, 19200])

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.reconnect_interval_sec = float(self.get_parameter('reconnect_interval_sec').value)
        self.frame_timeout_sec = float(self.get_parameter('frame_timeout_sec').value)
        self.status_log_interval_sec = float(self.get_parameter('status_log_interval_sec').value)
        self.mag_scale = float(self.get_parameter('magnetic_field_lsb_to_tesla').value)

        self.baudrate_candidates = self.normalize_baud_candidates(
            [int(value) for value in self.get_parameter('baudrate_candidates').value]
        )
        self.current_baud_index = 0
        self.current_baudrate = self.baudrate_candidates[0]

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 20)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 20)

        self.parser = Hwt906pParserPy()
        self.serial_port: Optional[serial.Serial] = None
        self.connected = False
        self.last_open_error = ''
        self.last_reconnect_attempt = self.get_clock().now()
        self.received_bytes = 0
        self.valid_frames = 0
        self.valid_samples = 0
        self.unknown_frames = 0
        self.last_valid_frame = None
        self.last_status_log = None
        self.last_frame_type = None
        self.last_chunk_preview = ''

        timer_period = max(1.0 / self.poll_hz, 0.001)
        self.timer = self.create_timer(timer_period, self.poll_serial)

        self.get_logger().info(
            f'HWT906P Python driver started for {self.port}, '
            f'baud candidates={self.baudrate_candidates}, frame_id={self.frame_id}'
        )

        self.ensure_serial_connection(force=True)

    def destroy_node(self) -> bool:
        self.close_serial()
        return super().destroy_node()

    def normalize_baud_candidates(self, values: List[int]) -> List[int]:
        combined = [self.baudrate] + values
        candidates: List[int] = []
        for baud in combined:
            if baud in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600) and baud not in candidates:
                candidates.append(baud)
        if not candidates:
            raise RuntimeError('No supported baudrate candidates were configured')
        return candidates

    def close_serial(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None

        if self.connected:
            self.get_logger().warn(f'Lost connection to {self.port}, waiting to reconnect')

        self.connected = False
        self.parser.reset()
        self.received_bytes = 0
        self.valid_frames = 0
        self.valid_samples = 0
        self.unknown_frames = 0
        self.last_frame_type = None
        self.last_chunk_preview = ''

    def ensure_serial_connection(self, force: bool = False) -> bool:
        if self.serial_port is not None:
            return True

        now = self.get_clock().now()
        if not force and self.last_reconnect_attempt is not None:
            elapsed = (now - self.last_reconnect_attempt).nanoseconds / 1e9
            if elapsed < max(self.reconnect_interval_sec, 0.1):
                return False
        self.last_reconnect_attempt = now

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.current_baudrate,
                timeout=0.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
        except (SerialException, ValueError) as exc:
            error_text = str(exc)
            if error_text != self.last_open_error:
                self.get_logger().warn(
                    f'Unable to open {self.port} @ {self.current_baudrate}: {error_text}. '
                    f'Retrying every {self.reconnect_interval_sec:.1f}s'
                )
                self.last_open_error = error_text
            return False

        self.connected = True
        self.last_open_error = ''
        self.last_valid_frame = now
        self.get_logger().info(
            f'Connected to HWT906P on {self.port} using baudrate {self.current_baudrate}'
        )
        return True

    def advance_baudrate(self) -> None:
        previous = self.current_baudrate
        self.current_baud_index = (self.current_baud_index + 1) % len(self.baudrate_candidates)
        self.current_baudrate = self.baudrate_candidates[self.current_baud_index]
        self.get_logger().warn(
            f'No valid HWT906P frames at {previous} baud, switching to {self.current_baudrate}'
        )
        self.close_serial()

    def timed_out_since_last_valid_frame(self, now) -> bool:
        if self.last_valid_frame is None:
            return False
        elapsed = (now - self.last_valid_frame).nanoseconds / 1e9
        return elapsed > max(self.frame_timeout_sec, 0.5)

    def maybe_log_status(self, now) -> None:
        if self.last_status_log is not None:
            elapsed = (now - self.last_status_log).nanoseconds / 1e9
            if elapsed < max(self.status_log_interval_sec, 0.5):
                return
        self.last_status_log = now
        frame_type_text = f'0x{self.last_frame_type:02x}' if self.last_frame_type is not None else 'none'
        self.get_logger().info(
            'Serial status: '
            f'baud={self.current_baudrate}, bytes={self.received_bytes}, '
            f'valid_frames={self.valid_frames}, emitted_samples={self.valid_samples}, '
            f'unknown_frames={self.unknown_frames}, last_frame_type={frame_type_text}'
        )

    def poll_serial(self) -> None:
        if not self.ensure_serial_connection():
            return

        now = self.get_clock().now()
        try:
            chunk = self.serial_port.read(4096) if self.serial_port is not None else b''
        except SerialException as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            self.close_serial()
            return

        if not chunk:
            if self.timed_out_since_last_valid_frame(now):
                self.advance_baudrate()
            return

        self.received_bytes += len(chunk)
        self.last_chunk_preview = preview_bytes(chunk, 16)

        result = self.parser.feed(chunk)
        self.valid_frames += result.stats.valid_frames
        self.unknown_frames += result.stats.unknown_frames
        if result.stats.last_frame_type is not None:
            self.last_frame_type = result.stats.last_frame_type

        if result.samples:
            self.valid_samples += len(result.samples)
            self.last_valid_frame = now
        elif self.timed_out_since_last_valid_frame(now):
            preview = self.last_chunk_preview if self.last_chunk_preview else 'empty'
            self.get_logger().warn(
                f'No emitted samples at baud {self.current_baudrate}. Last chunk preview: {preview}'
            )
            self.advance_baudrate()
            return

        self.maybe_log_status(now)

        for sample in result.samples:
            self.publish_sample(sample)
#发布 sensor_msgs/Imu
#如果有磁力计数据，再发布 sensor_msgs/MagneticField
    def publish_sample(self, sample: ImuSample) -> None:
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation_covariance = list(DEFAULT_ORIENTATION_COVARIANCE)
        imu_msg.angular_velocity_covariance = list(DEFAULT_ANGULAR_VELOCITY_COVARIANCE)
        imu_msg.linear_acceleration_covariance = list(DEFAULT_LINEAR_ACCELERATION_COVARIANCE)

        if sample.quaternion_xyzw is not None:
            imu_msg.orientation.x = sample.quaternion_xyzw.x
            imu_msg.orientation.y = sample.quaternion_xyzw.y
            imu_msg.orientation.z = sample.quaternion_xyzw.z
            imu_msg.orientation.w = sample.quaternion_xyzw.w
        elif sample.euler_deg is not None:
            q = euler_to_quaternion(sample.euler_deg)
            imu_msg.orientation.x = q.x
            imu_msg.orientation.y = q.y
            imu_msg.orientation.z = q.z
            imu_msg.orientation.w = q.w
        else:
            imu_msg.orientation_covariance[0] = -1.0

        if sample.angular_velocity is not None:
            imu_msg.angular_velocity.x = sample.angular_velocity.x
            imu_msg.angular_velocity.y = sample.angular_velocity.y
            imu_msg.angular_velocity.z = sample.angular_velocity.z
        else:
            imu_msg.angular_velocity_covariance[0] = -1.0

        if sample.acceleration is not None:
            imu_msg.linear_acceleration.x = sample.acceleration.x
            imu_msg.linear_acceleration.y = sample.acceleration.y
            imu_msg.linear_acceleration.z = sample.acceleration.z
        else:
            imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        if sample.magnetic_field_raw is not None:
            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            mag_msg.magnetic_field.x = sample.magnetic_field_raw.x * self.mag_scale
            mag_msg.magnetic_field.y = sample.magnetic_field_raw.y * self.mag_scale
            mag_msg.magnetic_field.z = sample.magnetic_field_raw.z * self.mag_scale
            mag_msg.magnetic_field_covariance = list(MAGNETIC_FIELD_COVARIANCE)
            self.mag_pub.publish(mag_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Hwt906pNodePy()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if rclpy.ok():
            rclpy.logging.get_logger('hwt906p_node_py').fatal(f'HWT906P python node stopped: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
