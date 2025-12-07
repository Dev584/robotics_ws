import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_model = LaunchConfiguration('model', default='burger')
    
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='TurtleBot3 model (burger, waffle, waffle_pi)'
        ),
        
        # Set TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', robot_model),
        
        # Launch Gazebo with TurtleBot3
        Node(
            package='turtlebot3_gazebo',
            executable='gazebo.launch.py',
            arguments=['world:=' + os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world')],
        ),
        
        # Launch TurtleBot3 state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': open(
                    os.path.join(
                        get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_burger.urdf'
                    )
                ).read()}
            ],
        ),
        
        # Your navigation bridge
        Node(
            package='autonomous_navigation',
            executable='autonomous_navigation_bridge_exe',
            parameters=[
                {'max_velocity': 0.3},
                {'max_acceleration': 0.5},
                {'lookahead_distance': 0.3},
                {'control_frequency': 20.0}
            ],
            output='screen'
        ),
    ])