"""Localización: le da a AMCL la pose de inicio cuando llega /map (docs/team/INTERFACES.md).

AMCL publica map -> odom. Con `set_initial_pose: true` arranca todas las partículas
en un solo punto (covarianza cero), así que nunca corrige el error de colocar el
robot a mano. Este nodo manda la pose de inicio con un margen de error, y después
le pide a AMCL unas actualizaciones sin movimiento para que el filtro se acomode
antes de que el robot se mueva.
"""
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_srvs.srv import Empty

from equipo_jar.comun import QOS_MAPA, correr, parametros_en_vivo


class Localizacion(Node):
    def __init__(self):
        super().__init__('localizacion')
        self.p = parametros_en_vivo(self, {
            'x_inicial': 0.0,             # pose de inicio en map: cerca del origen (§5)
            'y_inicial': 0.0,
            'yaw_inicial': 0.0,
            'desvio_xy_inicial': 0.20,    # m, error de colocar el robot a mano
            'desvio_yaw_inicial': 0.30,   # rad
            'actualizaciones_quieto': 10,  # actualizaciones de AMCL sin movimiento al inicio
        })
        self.mapa_recibido = False
        self.pose_enviada = False
        # > 0: actualizaciones que faltan pedir; 0: todavía no empezó; -1: terminado.
        self.faltan = 0
        self.create_subscription(OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        # AMCL publica /amcl_pose reliable + transient local, igual que /map.
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.al_recibir_pose_amcl, QOS_MAPA)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # AMCL crea este servicio con nombre relativo: queda en el namespace raíz.
        self.sin_movimiento = self.create_client(Empty, '/request_nomotion_update')
        self.create_timer(0.5, self.tick)

    def al_recibir_mapa(self, _msg):
        self.mapa_recibido = True

    def al_recibir_pose_amcl(self, _msg):
        # AMCL tomó la pose de inicio: dejar de mandarla y empezar a acomodar.
        if self.pose_enviada and self.faltan == 0:
            self.faltan = self.p['actualizaciones_quieto']

    def tick(self):
        if not self.mapa_recibido or self.faltan < 0:
            return
        if self.faltan == 0:
            # Se reenvía en cada tick hasta que AMCL conteste: puede no estar activo todavía.
            self.enviar_pose_inicial()
            return
        if self.sin_movimiento.service_is_ready():
            self.sin_movimiento.call_async(Empty.Request())
            self.faltan -= 1
            if self.faltan == 0:
                self.get_logger().info('AMCL acomodado')
                self.faltan = -1

    def enviar_pose_inicial(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.p['x_inicial']
        msg.pose.pose.position.y = self.p['y_inicial']
        yaw = self.p['yaw_inicial']
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.p['desvio_xy_inicial'] ** 2   # x, y
        cov[35] = self.p['desvio_yaw_inicial'] ** 2          # yaw
        msg.pose.covariance = cov
        self.pub_pose.publish(msg)
        if not self.pose_enviada:
            self.get_logger().info('pose de inicio enviada a AMCL')
        self.pose_enviada = True


def main(args=None):
    correr(Localizacion, args)
