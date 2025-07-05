from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Python node'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for the serial connection'
    )

    config_dir = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share/walle_bot',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,

        Node(
            package='walle_bot',
            executable='arduino_node.py',
            name='arduino_node',
            output='screen',
            parameters=[
                config_dir,
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'baud_rate': LaunchConfiguration('baud_rate')
                }
            ]
        )
    ])
