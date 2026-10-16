#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 02 (v2, herramienta nueva)
# experiment_runner.py
#
# Recoleccion de datos de seguimiento de rutas (pure_pursuit / stanley) con
# la misma logica minimalista de la Practica 01: un solo nodo rclpy que
# reconfigura los parametros del controlador EN CALIENTE (SetParameters,
# sin relanzar nada), publica la meta directamente en /goal_pose (sin
# `ros2 topic pub` ni clics en RViz) y, en cuanto el controlador avisa por
# /navigation/goal_reached que termino, lee los dos .txt que el propio
# controlador ya guarda y calcula ahi mismo el error transversal (misma
# formula que practica2/scripts/analyze_path_following.py, para que sea
# comparable con lo que ya se reporto). Todo queda en UN SOLO CSV.
#
# No modifica nada de la practica2/ ya entregada: usa su propia carpeta
# (practica2_v2/datos/), nunca escribe en datos/runs.csv ni toca
# run_experiments.sh. El controlador (pure_pursuit.py / stanley.py) tampoco
# se modifica: ya trae todo lo que hace falta (parametros + aviso de
# goal_reached + guardado de los .txt).
#
# USO:
#   Terminal 1 (deja esto corriendo toda la sesion; -p controller:=
#   elige el controlador activo, pure_pursuit o stanley):
#     ros2 launch navig_utils motion_planning_experimentos.launch.py controller:=pure_pursuit
#
#   Terminal 2, cuando el log de la terminal 1 diga "Ready to execute new path":
#     ros2 run path_follower experiment_runner --ros-args \
#         -p controlador:=pure_pursuit -p num_repeticiones:=3
#
#   Para Stanley: relanza la terminal 1 con controller:=stanley y corre
#   experiment_runner con -p controlador:=stanley.
#
import csv
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ---------------------------------------------------------------------------
# CONFIG - edita a tu gusto.
# ---------------------------------------------------------------------------
# Mismas rutas ya validadas en la practica2 entregada (R1 casi recta, R2 en
# forma de "S"): (nombre, x, y, yaw final).
ROUTES = [("R1", 4.4716, -0.7614, 0.0),
          ("R2", 0.0, 5.0, 0.0)]

# Pure pursuit: barre alpha (beta fija) y despues beta (alpha fija), sin
# repetir el punto base (mismos valores que ya se usaron antes a mano).
PP_ALPHA_BASE, PP_BETA_BASE = 1.0, 0.4
PP_ALPHAS = [0.1, 0.5, 1.0, 4.0]
PP_BETAS = [0.1, 1.0, 4.0]  # sin 0.4: ya sale en el barrido de alpha

# Stanley: igual, Ka y Kd por separado.
ST_KD_BASE, ST_KA_BASE = 1.0, 1.0
ST_KAS = [0.5, 1.0, 2.0, 5.0]
ST_KDS = [0.5, 2.0, 5.0]  # sin 1.0: ya sale en el barrido de Ka

NUM_REPETICIONES_DEFAULT = 3
GOAL_REACHED_TIMEOUT_SEC = 120.0
SAVE_SETTLE_SEC = 0.5  # margen tras goal_reached para que acabe de escribirse el .txt
# ---------------------------------------------------------------------------


def pp_combos():
    combos = [{'alpha': PP_ALPHA_BASE, 'beta': b} for b in PP_BETAS]
    combos += [{'alpha': a, 'beta': PP_BETA_BASE} for a in PP_ALPHAS]
    return combos


def st_combos():
    combos = [{'Kd': ST_KD_BASE, 'Ka': a} for a in ST_KAS]
    combos += [{'Kd': d, 'Ka': ST_KA_BASE} for d in ST_KDS]
    return combos


def cross_track(points, path):
    """Distancia absoluta de cada punto a la polilinea 'path' (misma formula
    que cross_track() en analyze_path_following.py de practica2/)."""
    a, b = path[:-1], path[1:]
    ab = b - a
    l2 = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    d_abs = np.empty(len(points))
    for i, p in enumerate(points):
        t = np.clip(np.sum((p - a) * ab, axis=1) / l2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d_abs[i] = np.min(np.hypot(*(p - proj).T))
    return d_abs


FIELDNAMES = ['received_stamp', 'controlador', 'ruta', 'rep', 'alpha', 'beta', 'Kd', 'Ka',
              'success', 'duration_s', 'cte_rmse_m', 'cte_max_m', 'v_media_ms', 'w_max_abs_rads',
              'num_puntos_ruta', 'num_muestras']


class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner_p2')
        self.declare_parameter('controlador', 'pure_pursuit')  # 'pure_pursuit' o 'stanley'
        self.declare_parameter('num_repeticiones', NUM_REPETICIONES_DEFAULT)
        self.declare_parameter('folder', '')  # debe coincidir con el 'folder' del controlador; vacio = default
        self.declare_parameter('csv_path', '')  # vacio = se calcula relativo a este archivo

        self.controlador = self.get_parameter('controlador').get_parameter_value().string_value
        self.num_repeticiones = self.get_parameter('num_repeticiones').get_parameter_value().integer_value

        this_dir = os.path.dirname(os.path.realpath(__file__))
        practica2_v2_dir = os.path.normpath(os.path.join(this_dir, '..', '..', 'practica2_v2'))

        folder = self.get_parameter('folder').get_parameter_value().string_value
        self.folder = folder if folder else os.path.join(practica2_v2_dir, 'datos')
        os.makedirs(self.folder, exist_ok=True)

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.csv_path = csv_path if csv_path else os.path.join(practica2_v2_dir, 'datos', 'resultados_seguimiento.csv')
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_is_new = not os.path.exists(self.csv_path)

        if self.controlador not in ('pure_pursuit', 'stanley'):
            self.get_logger().error("controlador debe ser 'pure_pursuit' o 'stanley', no: " + self.controlador)
            raise SystemExit(1)

        self.goal_reached = False
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.sub_goal_reached = self.create_subscription(Bool, '/navigation/goal_reached',
                                                           self.callback_goal_reached, 1)
        self.clt_set_params = self.create_client(SetParameters, '/' + self.controlador + '/set_parameters')
        self.get_logger().info("Esperando servicio de parametros de " + self.controlador + " ...")
        while not self.clt_set_params.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("   ... sigo esperando (esta corriendo " + self.controlador +
                                    " en motion_planning_experimentos.launch.py?)")
        self.get_logger().info("Listo. CSV -> " + os.path.abspath(self.csv_path) +
                                "   carpeta de datos crudos -> " + os.path.abspath(self.folder))

    def callback_goal_reached(self, msg):
        if msg.data:
            self.goal_reached = True

    def set_param(self, name, value):
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        req = SetParameters.Request(parameters=[Parameter(name=name, value=pv)])
        future = self.clt_set_params.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result() if future.done() else None
        ok = result is not None and all(r.successful for r in result.results)
        if not ok:
            self.get_logger().error("No se pudo poner " + name + "=" + str(value))
        return ok

    def publish_goal(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(p)

    def newest_file(self, prefix, after_ts):
        candidates = []
        for name in os.listdir(self.folder):
            if name.startswith(prefix) and name.endswith('.txt'):
                full = os.path.join(self.folder, name)
                if os.path.getmtime(full) >= after_ts:
                    candidates.append(full)
        return max(candidates, key=os.path.getmtime) if candidates else None

    def append_row(self, row):
        write_header = self.csv_is_new and not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=FIELDNAMES, extrasaction='ignore', restval='')
            if write_header:
                w.writeheader()
            w.writerow(row)
        self.csv_is_new = False

    def run_one(self, params, route_label, rx, ry, ryaw, rep):
        for name, value in params.items():
            if not self.set_param(name, value):
                return
        self.goal_reached = False
        t_mark = time.time() - 0.5  # margen de seguridad para el filtro "-newer"
        t0 = time.time()
        self.publish_goal(rx, ry, ryaw)
        self.get_logger().info("[%s %s rep %d] meta (%.2f, %.2f) params=%s" % (
            self.controlador, route_label, rep, rx, ry, params))

        waited = 0.0
        while rclpy.ok() and not self.goal_reached and waited < GOAL_REACHED_TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.2)
            waited += 0.2
        duration = time.time() - t0

        row = {'received_stamp': datetime.now().isoformat(timespec='seconds'),
               'controlador': self.controlador, 'ruta': route_label, 'rep': rep,
               'alpha': params.get('alpha', ''), 'beta': params.get('beta', ''),
               'Kd': params.get('Kd', ''), 'Ka': params.get('Ka', ''),
               'success': False, 'duration_s': round(duration, 2),
               'cte_rmse_m': '', 'cte_max_m': '', 'v_media_ms': '', 'w_max_abs_rads': '',
               'num_puntos_ruta': '', 'num_muestras': ''}

        if not self.goal_reached:
            self.get_logger().warn("   -> timeout (%.0fs), se registra como fallo" % GOAL_REACHED_TIMEOUT_SEC)
            self.append_row(row)
            return

        self.get_clock().sleep_for(Duration(seconds=SAVE_SETTLE_SEC))
        gfile = self.newest_file('goal_' + self.controlador, t_mark)
        rfile = self.newest_file('robot_' + self.controlador, t_mark)
        if not gfile or not rfile:
            self.get_logger().warn("   -> goal_reached pero no encontre los .txt en " + self.folder)
            self.append_row(row)
            return

        try:
            path = np.loadtxt(gfile, delimiter=',', ndmin=2)[:, :2]
            robot = np.loadtxt(rfile, delimiter=',', ndmin=2)
            cte = cross_track(robot[:, :2], path)
            row.update({'success': True,
                        'cte_rmse_m': round(float(np.sqrt(np.mean(cte ** 2))), 4),
                        'cte_max_m': round(float(np.max(cte)), 4),
                        'v_media_ms': round(float(np.mean(robot[:, 3])), 3),
                        'w_max_abs_rads': round(float(np.max(np.abs(robot[:, 4]))), 3),
                        'num_puntos_ruta': len(path), 'num_muestras': len(robot)})
        except Exception as e:
            self.get_logger().error("   -> error leyendo/calculando: " + str(e))
        self.append_row(row)

    def run_all(self):
        combos = pp_combos() if self.controlador == 'pure_pursuit' else st_combos()
        total = len(combos) * len(ROUTES) * self.num_repeticiones
        self.get_logger().info("%d combinaciones x %d rutas x %d repeticiones = %d corridas" % (
            len(combos), len(ROUTES), self.num_repeticiones, total))
        n = 0
        for params in combos:
            for route_label, rx, ry, ryaw in ROUTES:
                for rep in range(1, self.num_repeticiones + 1):
                    n += 1
                    self.get_logger().info("--- corrida %d/%d ---" % (n, total))
                    self.run_one(params, route_label, rx, ry, ryaw, rep)
        self.get_logger().info("Listo: %d corridas. CSV -> %s" % (n, os.path.abspath(self.csv_path)))


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    node.run_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
