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
    
    # Argumentos
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
    
    # Nav2
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
    
    # Nodo HRI (siguiendo el patrón del profesor)
    hri_node = Node(
        package='simple_hri',
        executable='simple_hri_node',
        name='simple_hri',
        output='screen',
    )
    
    # Nodo BT waiter (similar al estilo bumpandgo del profesor)
    waiter_bt_node = Node(
        package='waiter_robot',
        executable='waiter_bt_example',
        name='waiter_bt',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'kitchen_x': 2.0,
            'kitchen_y': 5.0,
            'kitchen_yaw': 0.0,
            'client_x': 2.0,
            'client_y': -2.0,
            'client_yaw': 0.0,
            'home_x': -2.0,
            'home_y': 0.0,
            'home_yaw': 0.0,
        }],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        nav2_launch,
        hri_node,
        waiter_bt_node,
    ])