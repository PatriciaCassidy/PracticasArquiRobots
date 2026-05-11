from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_waiter = FindPackageShare('waiter_robot')
    pkg_nav2 = FindPackageShare('nav2_bringup')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_waiter, 'maps', 'map.yaml'),
        description='Full path to map file'
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_waiter, 'config', 'nav2_params.yaml'),
        }.items()
    )
    
    waiter_bt_node = Node(
        package='waiter_robot',
        executable='waiter_bt_example',
        name='waiter_bt',
        output='screen',
        parameters=[os.path.join(pkg_waiter, 'config', 'waiter_params.yaml')],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_waiter, 'rviz', 'waiter.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        nav2_launch,
        waiter_bt_node,
        rviz_node,
    ])