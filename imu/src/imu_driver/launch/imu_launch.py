from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='IMU device port'),
        
        Node(
            package='imu_driver',
            executable='IMU_publisher',
            name='IMU_publisher',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}] ,
        
        )
    ])



