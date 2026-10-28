"""Heading fix after turns: when the heading is in doubt and when it is trusted again."""
import math

from equipo_jar.correccion_giro import CorreccionGiro

P = {'actualizaciones_giro': 3, 'giro_minimo_rad': 0.3, 'girando_rads': 0.3,
     'quieto_lineal_ms': 0.02, 'quieto_angular_rads': 0.05, 'max_correccion_s': 3.0}


def girar(c, t, yaw, hasta, wz=1.0, dt=0.1):
    """Odometry of a turn in place from yaw to `hasta`; returns (t, yaw) at the end."""
    paso = math.copysign(abs(wz) * dt, hasta - yaw)
    while abs(hasta - yaw) > abs(paso):
        t += dt
        yaw += paso
        c.con_odom(t, yaw, 0.0, 0.0, math.copysign(abs(wz), paso))
    return t, yaw


def quieto(c, t, yaw, segundos, dt=0.1):
    for _ in range(round(segundos / dt)):
        t += dt
        c.con_odom(t, yaw, 0.0, 0.0, 0.0)
    return t


def test_big_turn_is_in_doubt_until_updates_after_it():
    c = CorreccionGiro(dict(P))
    t, yaw = girar(c, 0.0, 0.0, 1.57)
    assert c.en_duda
    c.con_actualizacion_amcl(t - 0.5)        # scan taken during the turn: does not count
    t = quieto(c, t, yaw, 0.2)
    assert c.pedir_actualizacion()
    for k in range(3):
        assert c.en_duda
        c.con_actualizacion_amcl(t + 0.01 * k)
    assert not c.en_duda and not c.pedir_actualizacion()
    assert c.correcciones == 1 and c.vencidas == 0


def test_small_turns_with_stops_in_between_do_not_add_up():
    c = CorreccionGiro(dict(P))
    t, yaw = 0.0, 0.0
    for _ in range(3):
        t, yaw = girar(c, t, yaw, yaw + 0.2)
        t = quieto(c, t, yaw, 0.3)
        assert not c.en_duda


def test_turning_again_restarts_the_count():
    c = CorreccionGiro(dict(P))
    t, yaw = girar(c, 0.0, 0.0, 1.0)
    t = quieto(c, t, yaw, 0.2)
    c.con_actualizacion_amcl(t)
    c.con_actualizacion_amcl(t)
    t, yaw = girar(c, t, yaw, 2.0)
    assert c.en_duda and c.actualizaciones == 0
    t = quieto(c, t, yaw, 0.2)
    for _ in range(3):
        c.con_actualizacion_amcl(t)
    assert not c.en_duda


def test_driving_on_after_the_turn_still_fixes_it():
    c = CorreccionGiro(dict(P))
    t, yaw = girar(c, 0.0, 0.0, 1.0)
    t += 0.1
    c.con_odom(t, yaw, 0.3, 0.0, 0.1)        # drives on with a small heading correction
    assert c.en_duda and c.pedir_actualizacion()
    for _ in range(3):
        c.con_actualizacion_amcl(t + 0.05)
    assert not c.en_duda


def test_gives_up_after_max_time():
    c = CorreccionGiro(dict(P))
    t, yaw = girar(c, 0.0, 0.0, 1.0)
    t = quieto(c, t, yaw, 3.5)               # no AMCL updates arrive
    assert not c.en_duda and c.vencidas == 1


def test_off_never_doubts():
    c = CorreccionGiro(dict(P, actualizaciones_giro=0))
    girar(c, 0.0, 0.0, 3.0)
    assert not c.en_duda and not c.pedir_actualizacion()


def test_angle_wraps_around_pi():
    c = CorreccionGiro(dict(P))
    c.con_odom(0.0, 3.1, 0.0, 0.0, 1.0)
    c.con_odom(0.1, -3.1, 0.0, 0.0, 1.0)     # 0.08 rad across +-pi, not 6.2
    assert c.girado < 0.1
