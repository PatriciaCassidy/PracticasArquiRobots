# launch/waiter.launch.py
#
# CORRECCIONES respecto a la version original:
#
# 1. waiter_bt_node: parameters=[LaunchConfiguration('use_sim_time')] era
#    incorrecto (una LaunchConfiguration no es un dict).
#    Corregido a parameters=[{'use_sim_time': ...}, params_file].
#
# 2. Se pasa waiter_params.yaml al nodo para que lea kitchen_x, client_x, etc.
#
# 3. rviz_node: el fichero waiter.rviz puede no existir aun; se protege con
#    condition=IfCondition para no bloquear el lanzamiento.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_waiter = FindPackageShare('waiter_robot')
    pkg_nav2   = FindPackageShare('nav2_bringup')

    # ── Argumentos ─────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar tiempo de simulacion')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_waiter, 'maps', 'map.yaml']),
        description='Ruta completa al fichero de mapa')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Lanzar RViz2')

    # ── Nav2 ────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map':         LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([pkg_waiter, 'config', 'nav2_params.yaml']),
        }.items()
    )

    # ── Nodo principal del robot camarero ──────────────────────────────────
    waiter_node = Node(
        package='waiter_robot',
        executable='waiter_bt_example',
        name='waiter_bt',
        output='screen',
        # FIX: parameters debe ser una lista de dicts, no un LaunchConfiguration suelto
        parameters=[
            PathJoinSubstitution([pkg_waiter, 'config', 'waiter_params.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ── RViz (opcional) ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_waiter, 'rviz', 'waiter.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        use_rviz_arg,
        nav2_launch,
        waiter_node,
        rviz_node,
    ])