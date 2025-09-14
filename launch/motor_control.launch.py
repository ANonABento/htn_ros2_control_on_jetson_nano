from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments for easy tuning
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='10.0',
        description='Maximum motor speed (0-100) - reduced for 12V motor heat management'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.25',
        description='Distance between wheels in meters'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout_duration',
        default_value='1.5',
        description='Motor safety timeout in seconds'
    )
    
    # Motor controller node
    motor_controller_node = Node(
        package='jetson_motor_control',
        executable='motor_controller',
        name='motor_controller',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'timeout_duration': LaunchConfiguration('timeout_duration'),
            'publish_rate': 10.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        max_speed_arg,
        wheel_base_arg,
        timeout_arg,
        motor_controller_node
    ])
