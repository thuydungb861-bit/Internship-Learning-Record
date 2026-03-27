from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='hwt906p_driver',
                executable='hwt906p_node',
                name='hwt906p_node',
                output='screen',
                parameters=[
                    {
                        'port': '/dev/ttyUSB1',
                        'baudrate': 230400,
                        'frame_id': 'imu_link',
                        'poll_hz': 200.0,
                        'magnetic_field_lsb_to_tesla': 1.0e-6,
                        'reconnect_interval_sec': 1.0,
                        'frame_timeout_sec': 2.0,
                        'status_log_interval_sec': 2.0,
                        'baudrate_candidates': [230400, 115200, 9600, 57600, 38400, 19200],
                    }
                ],
            )
        ]
    )
