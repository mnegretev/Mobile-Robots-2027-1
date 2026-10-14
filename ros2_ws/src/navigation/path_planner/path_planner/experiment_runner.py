#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 01
# experiment_runner.py
#
# Reemplaza a experimentos_practica1.sh en la parte de "llamar el servicio
# muchas veces y medir tiempos": en vez de invocar `ros2 service call` por
# CLI y leer los logs del nodo con grep, este nodo es un cliente real de
# /path_planning/plan_path (nav_msgs/srv/GetPlan) hecho con rclpy, y
# reconfigura los parametros del nodo planificador en caliente
# (SetParameters) para no tener que reiniciarlo entre combinaciones.
#
# El tiempo de ejecucion NO se mide aqui: lo publica el propio nodo
# planificador (a_star.py / rrt.py, ya modificados) en el topico
# /path_planning/plan_metrics. Ese topico lo recoge metrics_logger.py.
#
# El nodo de mapas de costo (cost_map) y el nodo planificador (a_star o
# rrt) los sigues lanzando tu a mano, exactamente como dice la practica
# (otra terminal, otro proceso). Este nodo solo automatiza la parte
# repetitiva de probar muchos puntos meta y combinaciones de parametros.
#
# USO (con house_simul, navig_utils, cost_map y a_star/rrt ya corriendo):
#   Terminal aparte, UNA sola vez por sesion completa:
#     ros2 run path_planner metrics_logger
#   Con a_star_node corriendo:
#     ros2 run path_planner experiment_runner --ros-args -p phase:=a_star
#   Despues, con rrt_node corriendo (ya pudiste parar a_star_node):
#     ros2 run path_planner experiment_runner --ros-args -p phase:=rrt
#
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ---------------------------------------------------------------------------
# CONFIG - mismos valores que experimentos_practica1.sh; edita a tu gusto
# ---------------------------------------------------------------------------
START = (0.0, 0.0)
GOALS = [
    (3.0, 0.0),
    (8.0, 0.0),
    (8.0, 3.0),
    (-3.0, 3.0),
    (5.0, -3.0),
]

A_STAR_NODE_NAME = 'a_star_node'
A_STAR_DIAGONALS_VALUES = [False, True]

RRT_NODE_NAME = 'rrt_node'
RRT_EPSILONS = [0.3, 0.5]
RRT_NS = [50, 100, 200, 300]
RRT_REPEATS = 2          # RRT es aleatorio -> repetir cada combinacion

CALL_TIMEOUT_SEC = 90.0
# ---------------------------------------------------------------------------


def make_pose(x, y):
    p = PoseStamped()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    return p


class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        self.declare_parameter('phase', 'a_star')  # 'a_star' o 'rrt'
        self.phase = self.get_parameter('phase').get_parameter_value().string_value
        self.target_node = A_STAR_NODE_NAME if self.phase == 'a_star' else RRT_NODE_NAME

        self.clt_plan = self.create_client(GetPlan, '/path_planning/plan_path')
        self.get_logger().info("Esperando el servicio /path_planning/plan_path ...")
        while not self.clt_plan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("   ... sigo esperando (ya corriste a_star o rrt?)")
        self.get_logger().info("Servicio disponible. Fase: " + self.phase)

        self.clt_set_params = self.create_client(SetParameters, '/' + self.target_node + '/set_parameters')

    def set_param(self, name, value):
        if not self.clt_set_params.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("No encontre el servicio de parametros de " + self.target_node +
                                     " (esta corriendo ese nodo?)")
            return False
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
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

    def call_plan(self, sx, sy, gx, gy):
        req = GetPlan.Request()
        req.start = make_pose(sx, sy)
        req.goal = make_pose(gx, gy)
        req.tolerance = 0.0
        future = self.clt_plan.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_SEC)
        if not future.done() or future.result() is None:
            self.get_logger().warn("Timeout o fallo llamando al servicio de planeacion.")
            return None
        return future.result().plan

    def run_a_star(self):
        sx, sy = START
        total = 0
        for diag in A_STAR_DIAGONALS_VALUES:
            self.set_param('diagonals', diag)
            self.get_logger().info("diagonals=" + str(diag))
            for (gx, gy) in GOALS:
                plan = self.call_plan(sx, sy, gx, gy)
                ok = plan is not None and len(plan.poses) > 0
                self.get_logger().info("   goal=(%.1f,%.1f) -> %s (%d pts)" % (
                    gx, gy, "OK" if ok else "FALLO", len(plan.poses) if plan else 0))
                total += 1
        self.get_logger().info("A* listo: %d llamadas de servicio hechas." % total)

    def run_rrt(self):
        sx, sy = START
        total = 0
        for eps in RRT_EPSILONS:
            self.set_param('epsilon', eps)
            for n in RRT_NS:
                self.set_param('N', n)
                self.get_logger().info("epsilon=%.2f N=%d" % (eps, n))
                for (gx, gy) in GOALS:
                    for rep in range(RRT_REPEATS):
                        plan = self.call_plan(sx, sy, gx, gy)
                        ok = plan is not None and len(plan.poses) > 1
                        self.get_logger().info("   goal=(%.1f,%.1f) rep=%d -> %s" % (
                            gx, gy, rep + 1, "OK" if ok else "FALLO"))
                        total += 1
        self.get_logger().info("RRT listo: %d llamadas de servicio hechas." % total)

    def run(self):
        if self.phase == 'a_star':
            self.run_a_star()
        elif self.phase == 'rrt':
            self.run_rrt()
        else:
            self.get_logger().error("Parametro 'phase' invalido: '" + self.phase +
                                     "' (usa 'a_star' o 'rrt')")


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
