#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 01
# metrics_logger.py
#
# Escucha /path_planning/plan_metrics (publicado por a_star.py y rrt.py
# justo despues de calcular cada ruta) y va guardando cada resultado en un
# CSV. Dejalo corriendo durante TODA la sesion de experimentos, aunque
# cambies de algoritmo o reinicies cost_map - no pierde nada entre medio
# porque escribe a disco en cada mensaje (append, con flush al cerrar
# el archivo en cada llamada).
#
# Para generar la tabla comparativa en cualquier momento, sin parar nada:
#   ros2 service call /path_planning/generate_table std_srvs/srv/Trigger
#
# Rutas por defecto (relativas a donde lances el nodo): practica1/datos/...
# y practica1/resultados/... Lanza el nodo desde ros2_ws/src/navigation/,
# o pasa rutas absolutas con -p csv_path:=... -p table_path:=...
#
import csv
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEFAULT_CSV = 'practica1/datos/practica01_resultados.csv'
DEFAULT_TABLE_MD = 'practica1/resultados/tabla_comparativa.md'

FIELDNAMES = ['received_stamp', 'session_label', 'algorithm', 'success',
              'exec_time_ms', 'num_points', 'start_x', 'start_y', 'goal_x',
              'goal_y', 'diagonals', 'epsilon', 'n']


def df_to_md(df, float_cols=()):
    import pandas as pd
    cols = list(df.columns)

    def fmt(v, c):
        if c in float_cols:
            try:
                if pd.isna(v):
                    return ''
                return '%.2f' % float(v)
            except (TypeError, ValueError):
                return str(v)
        if pd.isna(v):
            return ''
        return str(v)

    header = '| ' + ' | '.join(cols) + ' |'
    sep = '| ' + ' | '.join(['---'] * len(cols)) + ' |'
    rows = []
    for _, row in df.iterrows():
        rows.append('| ' + ' | '.join(fmt(row[c], c) for c in cols) + ' |')
    return '\n'.join([header, sep] + rows)


def generate_table(csv_path, table_path):
    import pandas as pd
    df = pd.read_csv(csv_path)
    df['success'] = df['success'].astype(str).str.lower() == 'true'

    out_dir = os.path.dirname(table_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    lines = ['# Tabla comparativa A* vs RRT - Practica 01', '']
    lines.append('Generado: ' + datetime.now().isoformat(timespec='seconds'))
    lines.append('Filas totales: %d' % len(df))
    lines.append('')

    for algo in sorted(df['algorithm'].dropna().unique()):
        sub = df[df['algorithm'] == algo].copy()
        lines.append('## ' + str(algo))
        lines.append('')
        if algo == 'a_star':
            group_cols = ['diagonals']
        elif algo == 'rrt':
            group_cols = ['epsilon', 'n']
        else:
            group_cols = []
        if group_cols:
            grouped = sub.groupby(group_cols).agg(
                llamadas=('success', 'count'),
                exitos=('success', 'sum'),
                tiempo_prom_ms=('exec_time_ms', 'mean'),
                tiempo_std_ms=('exec_time_ms', 'std'),
            ).reset_index()
            lines.append(df_to_md(grouped, float_cols=('tiempo_prom_ms', 'tiempo_std_ms')))
            lines.append('')
        lines.append('### Detalle por llamada')
        lines.append('')
        detail_cols = group_cols + ['goal_x', 'goal_y', 'success', 'exec_time_ms', 'num_points']
        lines.append(df_to_md(sub[detail_cols], float_cols=('goal_x', 'goal_y', 'exec_time_ms')))
        lines.append('')

    lines.append('## Comparacion global A* vs RRT')
    lines.append('')
    overall = df.groupby('algorithm').agg(
        llamadas=('success', 'count'),
        exitos=('success', 'sum'),
        tasa_exito=('success', 'mean'),
        tiempo_prom_ms=('exec_time_ms', 'mean'),
        tiempo_min_ms=('exec_time_ms', 'min'),
        tiempo_max_ms=('exec_time_ms', 'max'),
    ).reset_index()
    lines.append(df_to_md(overall, float_cols=('tasa_exito', 'tiempo_prom_ms', 'tiempo_min_ms', 'tiempo_max_ms')))
    lines.append('')

    with open(table_path, 'w') as f:
        f.write('\n'.join(lines))
    return len(df)


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('csv_path', DEFAULT_CSV)
        self.declare_parameter('table_path', DEFAULT_TABLE_MD)
        self.declare_parameter('session_label', '')

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        csv_dir = os.path.dirname(self.csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        self.is_new_file = not os.path.exists(self.csv_path)
        self.count = 0

        self.sub = self.create_subscription(String, '/path_planning/plan_metrics',
                                             self.callback_metrics, 10)
        self.srv_table = self.create_service(Trigger, '/path_planning/generate_table',
                                              self.callback_generate_table)
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

    def callback_generate_table(self, request, response):
        try:
            table_path = self.get_parameter('table_path').get_parameter_value().string_value
            n = generate_table(self.csv_path, table_path)
            response.success = True
            response.message = "Tabla generada con %d filas -> %s" % (n, os.path.abspath(table_path))
        except Exception as e:
            response.success = False
            response.message = "Error generando tabla: " + str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
