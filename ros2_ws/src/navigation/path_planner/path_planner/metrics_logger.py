#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 01
# metrics_logger.py
#
# Escucha /path_planning/plan_metrics (publicado por a_star.py y rrt.py
# justo despues de calcular cada ruta) y va guardando cada resultado (de
# a_star o de rrt, los distingue la columna "algorithm") en UN SOLO CSV.
# Dejalo corriendo durante toda la sesion de experimentos, aunque
# cambies de algoritmo - no pierde nada entre medio porque escribe a
# disco en cada mensaje (append).
#
# Ruta por defecto: practica1/datos/resultados_practica01.csv, calculada
# siempre relativa a la ubicacion real de este archivo (funciona con
# --symlink-install sin importar desde donde lances el nodo). Pasa una
# ruta absoluta propia con -p csv_path:=... si quieres otra ubicacion.
#
import csv
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PRACTICA1_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', '..', 'practica1'))
DEFAULT_CSV = os.path.join(_PRACTICA1_DIR, 'datos', 'resultados_practica01.csv')

FIELDNAMES = ['received_stamp', 'session_label', 'algorithm', 'success',
              'exec_time_ms', 'num_points', 'start_x', 'start_y', 'goal_x',
              'goal_y', 'diagonals', 'epsilon', 'n']


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('csv_path', DEFAULT_CSV)
        self.declare_parameter('session_label', '')

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        csv_dir = os.path.dirname(self.csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        self.is_new_file = not os.path.exists(self.csv_path)
        self.count = 0

        self.sub = self.create_subscription(String, '/path_planning/plan_metrics',
                                             self.callback_metrics, 10)
        self.get_logger().info("Escuchando /path_planning/plan_metrics -> " + os.path.abspath(self.csv_path))

    def callback_metrics(self, msg):
        try:
            record = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error("Mensaje de metricas invalido: " + str(e))
            return
        record['received_stamp'] = datetime.now().isoformat(timespec='seconds')
        record['session_label'] = self.get_parameter('session_label').get_parameter_value().string_value

        write_header = self.is_new_file and self.count == 0
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES, extrasaction='ignore', restval='')
            if write_header:
                writer.writeheader()
            writer.writerow(record)
        self.count += 1
        self.get_logger().info("[%d] %s success=%s t=%.2fms goal=(%.1f,%.1f)" % (
            self.count, record.get('algorithm', '?'), record.get('success', '?'),
            record.get('exec_time_ms', -1.0), record.get('goal_x', 0.0), record.get('goal_y', 0.0)))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
