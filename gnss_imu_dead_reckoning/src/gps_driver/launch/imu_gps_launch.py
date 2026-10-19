from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB1', description='IMU device port'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyUSB0', description='GPS device port'),

        Node(
            package='imu_driver',
            executable='IMU_publisher',
            name='IMU_publisher',
            output='screen',
            parameters=[{'port': LaunchConfiguration('imu_port')}],
        ),
        Node(
            package='gps_driver',
            executable='driver',
            name='Gps_publisher',
            output='screen',
            parameters=[{'port': LaunchConfiguration('gps_port')}],
        ),
    ])
