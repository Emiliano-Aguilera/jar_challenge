"""Localización: arranca Nav2 y le da a AMCL la pose de inicio (docs/team/INTERFACES.md).

Dos tareas:

1. Arranque: lleva AMCL y los nodos de Nav2 a activos, en orden. Reemplaza al
   lifecycle_manager de Nav2, que en Humble espera la respuesta de cada cambio de
   estado sin límite: si una respuesta se pierde, queda colgado y el robot no se
   mueve nunca (pasó en una corrida: "failed to send response to /amcl/change_state").
   Acá cada pedido tiene un plazo; al vencer se pregunta el estado y se sigue desde ahí.

2. Pose de inicio: AMCL publica map -> odom. Con `set_initial_pose: true` arranca
   todas las partículas en un solo punto (covarianza cero), así que nunca corrige el
   error de colocar el robot a mano. Este nodo manda la pose de inicio con un margen
   de error, y después le pide a AMCL unas actualizaciones sin movimiento para que el
   filtro se acomode antes de que el robot se mueva.
"""
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import Empty

from equipo_jar.comun import QOS_MAPA, correr, parametros_en_vivo


class Arranque:
    """Lleva una lista de nodos lifecycle a activos, uno por uno, sin bloquear nunca.

    En cada paso pregunta el estado del primer nodo que falta y pide la transición
    que corresponde: unconfigured -> configure, inactive -> activate. Cada respuesta
    dispara el paso siguiente; el timer solo cuenta los plazos. Un pedido sin
    respuesta en `plazo_s` se abandona y se vuelve a preguntar el estado.
    """

    def __init__(self, nodo, nombres, plazo_s):
        self.log = nodo.get_logger()
        self.faltan = list(nombres)
        self.plazo_s = plazo_s
        self.clientes = {n: (nodo.create_client(GetState, f'/{n}/get_state'),
                             nodo.create_client(ChangeState, f'/{n}/change_state'))
                         for n in nombres}
        self.pedido = None      # (cliente, future) en curso
        self.vence = 0.0
        self.ultimo_estado = None

    def paso(self):
        if not self.faltan:
            return
        if self.pedido is not None:
            cliente, futuro = self.pedido
            if not futuro.done():
                if time.monotonic() < self.vence:
                    return
                cliente.remove_pending_request(futuro)
                # Normal mientras el planner espera /map para activarse: no contesta.
                self.log.info(f'{self.faltan[0]}: sin respuesta en {self.plazo_s:.0f} s, '
                              'vuelvo a preguntar')
            elif isinstance(futuro.result(), GetState.Response):
                self.pedido = None
                self.decidir(futuro.result().current_state)
                return
            self.pedido = None
        self.pedir(GetState.Request(), self.clientes[self.faltan[0]][0])

    def decidir(self, estado):
        nombre = self.faltan[0]
        if estado.id != self.ultimo_estado:
            self.log.info(f'{nombre}: {estado.label}')
            self.ultimo_estado = estado.id
        if estado.id == State.PRIMARY_STATE_ACTIVE:
            self.faltan.pop(0)
            self.ultimo_estado = None
            if not self.faltan:
                self.log.info('Nav2 activo')
            return
        transicion = {State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
                      State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE}.get(estado.id)
        if transicion is not None:    # en los otros estados hay una transición en curso
            pedido = ChangeState.Request()
            pedido.transition.id = transicion
            self.pedir(pedido, self.clientes[nombre][1])

    def pedir(self, pedido, cliente):
        if cliente.service_is_ready():
            futuro = cliente.call_async(pedido)
            futuro.add_done_callback(lambda _f: self.paso())   # sigue apenas contesta
            self.pedido = (cliente, futuro)
            # Activar el planner espera la TF map -> base, que llega después de /map.
            # Si vence el plazo, el estado dice si la activación sigue en curso.
            self.vence = time.monotonic() + self.plazo_s


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
        # Los pone navegacion.launch.py. Sin AMCL (pruebas.launch.py) otro publica map -> odom.
        self.usar_amcl = self.declare_parameter('usar_amcl', True).value
        nodos_nav2 = self.declare_parameter('nodos_nav2', ['']).value
        plazo = self.declare_parameter('plazo_arranque_s', 5.0).value
        self.arranque = Arranque(
            self, (['amcl'] if self.usar_amcl else []) + [n for n in nodos_nav2 if n], plazo)
        # Reloj de pared: el arranque no depende de /clock.
        self.create_timer(0.5, self.arranque.paso, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.mapa_recibido = False
        self.pose_enviada = False
        # > 0: actualizaciones que faltan pedir; 0: todavía no empezó; -1: terminado.
        self.faltan = 0 if self.usar_amcl else -1
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
