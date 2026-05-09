import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. TurtleBot3 Empty World launch file
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_share = get_package_share_directory('peppermint_task1')
    sphere_path = os.path.join(pkg_share, 'models', 'sphere.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    # 2. Command to spawn the green sphere

    spawn_sphere = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', 
             '-file', sphere_path, 
             '-name', 'green_sphere', 
             '-x', '1.5', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    # 3.Vision Node
    vision_node = Node(
        package='peppermint_task1',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    # 4. Controller Node
    controller_node = Node(
        package='peppermint_task1',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        
        # A slight delay (5 seconds) before spawning the sphere and starting nodes to give Gazebo time to fully boot up the environment first.
        TimerAction(
            period=5.0,
            actions=[spawn_sphere, vision_node, controller_node]
        )
    ])