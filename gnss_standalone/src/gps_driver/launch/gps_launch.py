from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='GPS device port'),
        
        Node(
            package='gps_driver',
            executable='driver',
            name='gps_node',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}] ,
        
        )
    ])



