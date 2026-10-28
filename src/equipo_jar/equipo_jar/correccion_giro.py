"""When AMCL's heading needs a fix after a turn, and when it is fixed again (no ROS here).

On the real robot /odom counts turns ~25 % too much. AMCL fixes the heading with the
LiDAR, but only while the robot moves: when it stops after a turn the error stays,
just when the camera looks. So after a turn of at least `giro_minimo_rad` (odometry,
added up since the last stop) the heading is "in doubt" until AMCL has done
`actualizaciones_giro` updates with scans taken after the turn ended (rotation below
`girando_rads`). Meanwhile the node asks AMCL for no-motion updates
(`/request_nomotion_update`): AMCL then uses the next scan even if the robot barely moved.

Turned rotation resets at every stop. Checked on the week-10 bag (notes/hans-die/NOTES.md).
"""
import math


def diferencia_angulo(a, b):
    """a - b in (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class CorreccionGiro:
    """Feed it /odom and AMCL's updates; read `en_duda` and `pedir_actualizacion()`.

    `p` is the node's live parameter dict: actualizaciones_giro (0 = off),
    giro_minimo_rad, girando_rads, quieto_lineal_ms, quieto_angular_rads, max_correccion_s.
    Times are message stamps in seconds (same clock for /odom and /amcl_pose).
    """

    def __init__(self, p):
        self.p = p
        self.yaw_previo = None
        self.girado = 0.0          # rad turned since the last stop or fix
        self.en_duda = False       # heading not trusted: turned and not fixed yet
        self.fin_giro = None       # stamp when the rotation stopped (None: still turning)
        self.actualizaciones = 0   # AMCL updates with scans after fin_giro
        self.quieto = False
        self.t = None
        self.correcciones = 0      # finished fixes, for the log
        self.vencidas = 0          # fixes ended by max_correccion_s
        self.ultima = None         # (updates, seconds) of the last finished fix

    def con_odom(self, t, yaw, vx, vy, wz):
        self.t = t
        if self.yaw_previo is not None:
            self.girado += abs(diferencia_angulo(yaw, self.yaw_previo))
        self.yaw_previo = yaw
        # Turning: an in-place turn, not the small corrections while driving.
        gira = abs(wz) >= self.p['girando_rads']
        self.quieto = (abs(wz) < self.p['quieto_angular_rads']
                       and math.hypot(vx, vy) < self.p['quieto_lineal_ms'])
        if self.p['actualizaciones_giro'] <= 0:
            self.en_duda = False
            self.girado = 0.0
            return
        if not self.en_duda and self.girado >= self.p['giro_minimo_rad']:
            self.en_duda = True
            self.fin_giro = None
            self.actualizaciones = 0
        if self.en_duda:
            if gira:
                self.fin_giro = None
                self.actualizaciones = 0
            elif self.fin_giro is None:
                self.fin_giro = t
            elif t - self.fin_giro > self.p['max_correccion_s']:
                self.vencidas += 1
                self._terminar()
        elif self.quieto:
            self.girado = 0.0       # a small turn followed by a stop does not add up

    def con_actualizacion_amcl(self, t_scan):
        """AMCL published a pose for the scan taken at t_scan."""
        if self.en_duda and self.fin_giro is not None and t_scan > self.fin_giro:
            self.actualizaciones += 1
            if self.actualizaciones >= self.p['actualizaciones_giro']:
                self._terminar()

    def pedir_actualizacion(self):
        """True: ask AMCL for a no-motion update now (the turn ended, not fixed yet)."""
        return self.en_duda and self.fin_giro is not None

    def _terminar(self):
        self.ultima = (self.actualizaciones, (self.t or 0.0) - (self.fin_giro or self.t or 0.0))
        self.correcciones += 1
        self.en_duda = False
        self.girado = 0.0
        self.fin_giro = None
        self.actualizaciones = 0
