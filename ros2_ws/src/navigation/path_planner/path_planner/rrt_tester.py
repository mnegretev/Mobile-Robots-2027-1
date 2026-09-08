#!/usr/bin/env python3
import csv
import time
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

class RRTTester(Node):
    def __init__(self):
        super().__init__('rrt_automation_tester')
        
        # Cliente para pedir rutas
        self.cli_plan = self.create_client(GetPlan, '/path_planning/plan_path')
        # Cliente para cambiar parámetros del nodo RRT en caliente
        self.cli_param = self.create_client(SetParameters, '/rrt_node/set_parameters')

        self.get_logger().info('Esperando servicios de rrt_node...')
        while not self.cli_plan.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando /path_planning/plan_path...')
        while not self.cli_param.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando /rrt_node/set_parameters...')
        self.get_logger().info('Servicios detectados y listos.')

    def set_rrt_params(self, epsilon: float, n_val: int):
        """Actualiza epsilon y N en rrt_node sin reiniciar el nodo."""
        req = SetParameters.Request()
        
        p_eps = Parameter()
        p_eps.name = 'epsilon'
        p_eps.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(epsilon))
        
        p_n = Parameter()
        p_n.name = 'N'
        p_n.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(n_val))
        
        req.parameters = [p_eps, p_n]
        future = self.cli_param.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def request_path(self, sx, sy, gx, gy):
        """Envía una solicitud GetPlan y mide el tiempo."""
        req = GetPlan.Request()
        req.start.header.frame_id = 'map'
        req.start.pose.position.x = float(sx)
        req.start.pose.position.y = float(sy)
        
        req.goal.header.frame_id = 'map'
        req.goal.pose.position.x = float(gx)
        req.goal.pose.position.y = float(gy)

        t_start = time.perf_counter()
        future = self.cli_plan.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        t_end = time.perf_counter()
        
        res = future.result()
        elapsed_s = t_end - t_start
        success = len(res.plan.poses) > 1
        num_waypoints = len(res.plan.poses)
        
        return success, elapsed_s, num_waypoints


def run_experiments():
    rclpy.init()
    tester = RRTTester()

    # --- BANCO DE EXPERIMENTOS ---
    # Punto de origen fijo
    start_pos = (0.0, 0.0)

    # Distintos puntos meta (P_g)
    goals = [
        (5.0, 0.0),   # Meta estándar de la práctica (pasillo despejado)
        (7.0, -4.0),   # Meta en habitación contigua
        (6.0, -2.5),
        (2.0, 3.0),
        (1.0, 5.0),
        (-4.0, -4.0),
        (-8.0, -3.0),
        (-8.0, 1.0),
        (-3.5, 2.5),
        (8.0, 2.5)

          # Meta lejana con esquinas
    ]

    # Diferentes combinaciones de parámetros de sintonización
    epsilons = [0.2, 0.5, 0.8, 1.0]
    n_values = [100, 300, 600, 900]

    # Repeticiones por combinación (debido a la naturaleza estocástica de RRT)
    repetitions = 10

    output_file = "experimentos_rrt.csv"

    with open(output_file, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Goal_X", "Goal_Y", "Epsilon", "N", "Iteracion", "Exito", "Tiempo_s", "Waypoints"])

        for gx, gy in goals:
            for eps in epsilons:
                for n_val in n_values:
                    tester.set_rrt_params(eps, n_val)
                    print(f"\n==================================================")
                    print(f"Probando Goal=({gx}, {gy}) | Epsilon={eps} | N={n_val}")
                    print(f"==================================================")
                    
                    exitos_lote = 0
                    tiempos_lote = []

                    for rep in range(1, repetitions + 1):
                        success, elapsed, waypoints = tester.request_path(
                            start_pos[0], start_pos[1], gx, gy
                        )
                        writer.writerow([gx, gy, eps, n_val, rep, success, f"{elapsed:.4f}", waypoints])
                        f.flush()

                        if success:
                            exitos_lote += 1
                            tiempos_lote.append(elapsed)

                        status = "ÉXITO" if success else "FALLO"
                        print(f"  Intento {rep}/{repetitions}: {status} | Tiempo: {elapsed*1000:.2f} ms | Puntos: {waypoints}")
                        time.sleep(0.1)  # Pequeño delay entre peticiones

                    tasa = (exitos_lote / repetitions) * 100
                    prom_t = (sum(tiempos_lote)/len(tiempos_lote)) if tiempos_lote else 0.0
                    print(f"-> Resumen: Tasa éxito = {tasa:.1f}% | Tiempo prom (éxitos) = {prom_t*1000:.2f} ms")

    print(f"\nTodos los experimentos guardados en '{output_file}'.")
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_experiments()
