from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Encontrar los directorios de los paquetes
    pkg_waiter_share = FindPackageShare('waiter_robot')
    pkg_nav2_share = FindPackageShare('nav2_bringup')
    
    # Argumentos de lanzamiento
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=[pkg_waiter_share, '/maps/map.yaml'],
        description='Full path to map file'
    )
    
    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_nav2_share, '/launch/bringup_launch.py']
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': [pkg_waiter_share, '/config/nav2_params.yaml'],
        }.items()
    )
    
    # Nodo BT waiter
    waiter_bt_node = Node(
        package='waiter_robot',
        executable='waiter_bt_example',
        name='waiter_bt',
        output='screen',
        parameters=[LaunchConfiguration('use_sim_time')],
    )
    
    # RViz (opcional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [pkg_waiter_share, '/rviz/waiter.rviz']],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        nav2_launch,
        waiter_bt_node,
        rviz_node,
    ])