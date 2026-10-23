"""Localización (AMCL) y navegación (Nav2). Lo incluye competencia.launch.py.

Sin nav2_bringup: su launch de localización levanta map_server, que está prohibido.
El mapa llega solo del juez por /map. Parámetros: params/navegacion.yaml.
Los topics quedan con los nombres de Nav2 (/initialpose, /amcl_pose, ...), así
funcionan las herramientas del visor del equipo. Solo se mueve /diagnostics: el
driver del robot real también lo publica.

`localizacion:=false` deja AMCL afuera, para pruebas.launch.py (map -> odom fijo).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('equipo_jar'), 'params', 'navegacion.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    comunes = [params, {'use_sim_time': use_sim_time}]
    # El driver del robot real publica /diagnostics; el lifecycle manager de Nav2 también.
    sin_diagnostics = [('/diagnostics', '/jar/diagnostics')]
    # Un solo publicador de /cmd_vel: el smoother. Controller y behaviors le pasan
    # sus velocidades por cmd_vel_nav. Después de /done alcanza con que paren ellos.
    al_smoother = [('cmd_vel', 'cmd_vel_nav')]

    def nodo(paquete, ejecutable, nombre, remapeos=()):
        return Node(package=paquete, executable=ejecutable, name=nombre, output='screen',
                    parameters=comunes, remappings=list(remapeos))

    localizacion = GroupAction(
        condition=IfCondition(LaunchConfiguration('localizacion')),
        actions=[
            nodo('nav2_amcl', 'amcl', 'amcl'),
            nodo('equipo_jar', 'localizacion', 'localizacion'),
            nodo('nav2_lifecycle_manager', 'lifecycle_manager',
                 'lifecycle_manager_localizacion', sin_diagnostics),
        ])

    # Siempre arranca: en pruebas.launch.py usa la TF fija en vez de AMCL.
    navegacion = [
        nodo('nav2_controller', 'controller_server', 'controller_server', al_smoother),
        nodo('nav2_planner', 'planner_server', 'planner_server'),
        nodo('nav2_behaviors', 'behavior_server', 'behavior_server', al_smoother),
        nodo('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nodo('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel')]),
        nodo('nav2_lifecycle_manager', 'lifecycle_manager',
             'lifecycle_manager_navegacion', sin_diagnostics),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localizacion', default_value='true',
                              description='false: sin AMCL (otro publica map -> odom)'),
        localizacion,
        *navegacion,
    ])
