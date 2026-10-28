"""Localización (AMCL) y navegación (Nav2). Lo incluye competencia.launch.py.

Sin nav2_bringup: su launch de localización levanta map_server, que está prohibido.
El mapa llega solo del juez por /map. Parámetros: params/navegacion.yaml.
Los topics quedan con los nombres de Nav2 (/initialpose, /amcl_pose, ...), así
funcionan las herramientas del visor del equipo.

Sin lifecycle_manager de Nav2: el nodo `localizacion` lleva los nodos a activos con
reintentos (el de Nav2 puede quedar colgado para siempre, ver localizacion.py).

`localizacion:=false` deja AMCL afuera, para pruebas.launch.py (map -> odom fijo).

Forma del robot: un rectángulo, no un círculo. El chasis está corrido hacia atrás,
así que el círculo que lo cubre (0.21 m) cierra huecos de 0.5 m entre cajas y
paredes, y uno más chico deja afuera las esquinas de atrás. Con el rectángulo el
planner ve el medio ancho real (0.10 m) y MPPI y los behaviors chequean la forma
real cerca de los obstáculos.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Chasis del X3 medido desde base_footprint (URDF del simulador,
# rosmaster_x3_base.urdf.xacro): 0.30 m de largo corrido 3.1 cm hacia atrás y
# 0.20 m de ancho con las ruedas. El margen extra es footprint_padding (navegacion.yaml).
ADELANTE = 0.119
ATRAS = 0.181
MEDIO_ANCHO = 0.100


def footprint():
    """Rectángulo del chasis en el formato de Nav2: "[[x, y], ...]"."""
    esquinas = [(ADELANTE, MEDIO_ANCHO), (ADELANTE, -MEDIO_ANCHO),
                (-ATRAS, -MEDIO_ANCHO), (-ATRAS, MEDIO_ANCHO)]
    return str([list(e) for e in esquinas])


def generate_launch_description():
    compartido = get_package_share_directory('equipo_jar')
    params = os.path.join(compartido, 'params', 'navegacion.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Un dict de parámetros le llega a todos los nodos del proceso (launch_ros lo
    # escribe para /**), también a los costmaps que corren dentro del controller y
    # del planner. Los nodos que no usan `footprint` lo ignoran.
    comunes = [params, {'use_sim_time': use_sim_time, 'footprint': footprint()}]
    # Un solo publicador de /cmd_vel: el smoother. Controller y behaviors le pasan
    # sus velocidades por cmd_vel_nav. Después de /done alcanza con que paren ellos.
    al_smoother = [('cmd_vel', 'cmd_vel_nav')]

    def nodo(paquete, ejecutable, nombre, remapeos=(), extra=None):
        return Node(package=paquete, executable=ejecutable, name=nombre, output='screen',
                    parameters=comunes + ([extra] if extra else []), remappings=list(remapeos))

    # Siempre arrancan: en pruebas.launch.py usan la TF fija en vez de AMCL.
    navegacion = [
        nodo('nav2_controller', 'controller_server', 'controller_server', al_smoother),
        nodo('nav2_planner', 'planner_server', 'planner_server'),
        nodo('nav2_behaviors', 'behavior_server', 'behavior_server', al_smoother),
        nodo('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel')]),
        # navegar.xml: Humble's default tree, with the goal checker named (there are two).
        nodo('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', extra={
            'default_nav_to_pose_bt_xml': os.path.join(
                compartido, 'behavior_trees', 'navegar.xml')}),
    ]
    con_amcl = LaunchConfiguration('localizacion')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localizacion', default_value='true',
                              description='false: sin AMCL (otro publica map -> odom)'),
        GroupAction(condition=IfCondition(con_amcl), actions=[nodo('nav2_amcl', 'amcl', 'amcl')]),
        *navegacion,
        # Activa los nodos en el orden de la lista (AMCL primero si está). bt_navigator
        # último: cuando aparece navigate_to_pose, todo lo demás ya está activo.
        nodo('equipo_jar', 'localizacion', 'localizacion', extra={
            'usar_amcl': ParameterValue(con_amcl, value_type=bool),
            'nodos_nav2': ['controller_server', 'planner_server', 'behavior_server',
                           'velocity_smoother', 'bt_navigator']}),
    ])
