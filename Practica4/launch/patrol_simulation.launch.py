from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_patrol_fsm = get_package_share_directory('patrol_fsm')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar tiempo de simulacion'
    )
    
    patrol_fsm_node = Node(
        package='patrol_fsm',
        executable='patrol_fsm',
        name='patrol_fsm_node',
        output='screen',
        parameters=[
            os.path.join(pkg_patrol_fsm, 'config', 'patrol_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([use_sim_time_arg, patrol_fsm_node])