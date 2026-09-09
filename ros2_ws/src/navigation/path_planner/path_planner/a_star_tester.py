#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# MOBILE ROBOTS - FI-UNAM
# AUTOMATED TESTER FOR A* PATH PLANNING
#
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import csv
import time
import math
import os

class AStarTester(Node):
    def __init__(self):
        super().__init__('a_star_tester')
        self.get_logger().info("Iniciando nodo cliente de pruebas para A*...")

        self.clt_plan = self.create_client(GetPlan, '/path_planning/plan_path')
        self.clt_params = self.create_client(SetParameters, '/a_star_node/set_parameters')

        while not self.clt_plan.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Esperando servicio /path_planning/plan_path...")

        while not self.clt_params.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Esperando servicio de parámetros /a_star_node/set_parameters...")

        self.get_logger().info("Servicios conectados correctamente.")

    def set_diagonals(self, use_diagonals: bool):
        req = SetParameters.Request()
        param = Parameter()
        param.name = "diagonals"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=use_diagonals
        )
        req.parameters = [param]
        future = self.clt_params.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def compute_path_length(self, poses):
        if len(poses) < 2:
            return 0.0
        total_dist = 0.0
        for i in range(1, len(poses)):
            p1 = poses[i - 1].pose.position
            p2 = poses[i].pose.position
            total_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
        return total_dist

    def request_plan(self, start, goal):
        req = GetPlan.Request()
        req.start = PoseStamped()
        req.start.header.frame_id = "map"
        req.start.pose.position.x = float(start[0])
        req.start.pose.position.y = float(start[1])

        req.goal = PoseStamped()
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = float(goal[0])
        req.goal.pose.position.y = float(goal[1])

        t0 = time.perf_counter()
        future = self.clt_plan.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        t1 = time.perf_counter()

        elapsed_ms = (t1 - t0) * 1000.0
        return future.result(), elapsed_ms

    def run_tests(self, scenarios, repetitions=1, output_csv="a_star_raw_results.csv"):
        records = []
        test_id = 1

        print(f"\n[INFO] Ejecutando {len(scenarios)} pruebas (1 ejecución por caso)...\n")

        for sc in scenarios:
            name = sc["name"]
            start = sc["start"]
            goal = sc["goal"]
            use_diag = sc["use_diagonals"]

            self.set_diagonals(use_diag)
            time.sleep(0.05)

            direct_distance = math.hypot(goal[0] - start[0], goal[1] - start[1])

            for rep in range(1, repetitions + 1):
                resp, rtt_ms = self.request_plan(start, goal)

                if resp is not None and resp.plan and len(resp.plan.poses) > 0:
                    success = True
                    nodes = len(resp.plan.poses)
                    path_len = self.compute_path_length(resp.plan.poses)
                    detour_ratio = path_len / direct_distance if direct_distance > 0 else 1.0
                else:
                    success = False
                    nodes = 0
                    path_len = 0.0
                    detour_ratio = 0.0

                record = {
                    "test_id": test_id,
                    "scenario": name,
                    "repetition": rep,
                    "use_diagonals": use_diag,
                    "start_x": start[0],
                    "start_y": start[1],
                    "goal_x": goal[0],
                    "goal_y": goal[1],
                    "direct_distance_m": round(direct_distance, 4),
                    "success": success,
                    "time_ms": round(rtt_ms, 3),
                    "path_nodes": nodes,
                    "path_length_m": round(path_len, 4),
                    "detour_ratio": round(detour_ratio, 4)
                }
                records.append(record)
                status = "OK" if success else "FALLÓ"
                diag_tag = "ConDiag" if use_diag else "SinDiag"
                print(f"[{test_id:02d}/20] Meta ({goal[0]:>4.1f}, {goal[1]:>4.1f}) | {diag_tag:<7} -> {status} | Nodos: {nodes:<4} | Tiempo: {rtt_ms:.2f} ms")
                test_id += 1

        self.save_csv(records, output_csv)

    def save_csv(self, records, filename):
        if not records:
            return
        with open(filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)
        print(f"\n[ÉXITO] Archivo CSV generado: {os.path.abspath(filename)}")


def main(args=None):
    rclpy.init(args=args)
    tester = AStarTester()

    start_point = [0.0, 0.0]

    goals = [
        (5.0, 0.0),
        (7.0, -4.0),
        (6.0, -2.5),
        (2.0, 3.0),
        (1.0, 5.0),
        (-4.0, -4.0),
        (-8.0, -3.0),
        (-8.0, 1.0),
        (-3.5, 2.5),
        (8.0, 2.5)
    ]

    test_scenarios = []
    for idx, goal in enumerate(goals, 1):
        gx, gy = goal
        # Sin diagonales
        test_scenarios.append({
            "name": f"Meta{idx}_({gx},{gy})_SinDiag",
            "start": start_point,
            "goal": [gx, gy],
            "use_diagonals": False
        })
        # Con diagonales
        test_scenarios.append({
            "name": f"Meta{idx}_({gx},{gy})_ConDiag",
            "start": start_point,
            "goal": [gx, gy],
            "use_diagonals": True
        })

    try:
        # Se ejecuta exactamente 1 vez por configuración
        tester.run_tests(test_scenarios, repetitions=1, output_csv="a_star_raw_results.csv")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()