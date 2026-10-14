#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 01
# generar_tabla.py
#
# Version standalone (sin ROS2) de la generacion de tabla: lee el CSV que
# fue escribiendo metrics_logger.py y genera la tabla comparativa en
# Markdown. Util para regenerar la tabla mientras escribes el reporte,
# sin tener que tener ROS2 corriendo.
#
# Uso:
#   python3 generar_tabla.py [ruta_csv] [ruta_salida_md]
# Por defecto usa ../datos/practica01_resultados.csv y
# ../resultados/tabla_comparativa.md (relativo a este script).
#
import os
import sys
from datetime import datetime

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.join(HERE, '..', 'datos', 'practica01_resultados.csv')
DEFAULT_TABLE_MD = os.path.join(HERE, '..', 'resultados', 'tabla_comparativa.md')


def df_to_md(df, float_cols=()):
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


if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    table_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TABLE_MD
    n = generate_table(csv_path, table_path)
    print('Tabla generada con %d filas -> %s' % (n, os.path.abspath(table_path)))
