import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    pkg_dir = get_package_share_directory('peppermint_task1_cpp')
    sphere_path = os.path.join(pkg_dir, 'models', 'sphere.sdf')
    
    spawn_sphere = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', 
             '-file', sphere_path, 
             '-name', 'green_sphere', 
             '-x', '1.5', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    vision_node = Node(
        package='peppermint_task1_cpp',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    controller_node = Node(
        package='peppermint_task1_cpp',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(
            period=5.0,
            actions=[spawn_sphere, vision_node, controller_node]
        )
    ])
