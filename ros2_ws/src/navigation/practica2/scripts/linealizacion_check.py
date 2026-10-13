#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dinámica lineal del error transversal de pure pursuit y Stanley cerca de una ruta recta.

Para errores pequeños sobre una recta, ambas leyes llevan el error transversal e a
        e'' + 2*zeta*wn*e' + wn^2*e = 0
  Stanley (version del curso):  wn^2 = kd*v ,        2*zeta*wn = ka
  Pure pursuit (punto a L m):   wn^2 = K*v/L ,       2*zeta*wn = K ,   K = w_max/(2*beta)
(derivacion en el reporte).  Este script compara el sobreimpulso predicho,
exp(-pi*zeta/sqrt(1-zeta^2)), con el obtenido integrando el robot diferencial ideal
(kinematic_check.step) con las MISMAS leyes de control del codigo entregado, sobre una
recta de 60 m, con desfase inicial de 0.05 m y orientacion alineada.

Uso:  python3 linealizacion_check.py --out RESULTADOS/
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematic_check import step, wrap, V_MAX, W_MAX  # noqa: E402

V, L_EFF, Y0, DT, T = V_MAX, 0.3, 0.05, 0.0175, 40.0
PATH = np.c_[np.linspace(0, 60, 1201), np.zeros(1201)]  # puntos cada 0.05 m


def sim_pp(beta, alpha=1.0):
    x, y, a, idx, out = 0.0, Y0, 0.0, 0, []
    for k in range(int(T / DT)):
        gx, gy = PATH[idx]
        e = wrap(math.atan2(gy - y, gx - x) - a)
        v, w = V_MAX * math.exp(-e * e / alpha), W_MAX * math.tanh(e / (2 * beta))
        x, y, a = step(x, y, a, v, w, DT)
        if math.hypot(gx - x, gy - y) < 0.3:
            idx = min(idx + 1, len(PATH) - 1)
        out.append((k * DT, y))
    return np.array(out)


def sim_st(kd, ka, kv=5.0):
    x, y, a, out = 0.0, Y0, 0.0, []
    for k in range(int(T / DT)):
        d = np.hypot(PATH[:, 0] - x, PATH[:, 1] - y)
        i = int(np.argmin(d))
        xi, yi = PATH[i]
        n, p = min(i + 1, len(PATH) - 1), max(i - 1, 0)
        ti = math.atan2(PATH[n, 1] - PATH[p, 1], PATH[n, 0] - PATH[p, 0])
        te = wrap(ti - math.atan2(y - yi, x - xi))
        et = (1.0 if te > 0 else -1.0 if te < 0 else 0.0) * math.hypot(x - xi, y - yi)
        al = wrap(ti - a)
        v, w = V_MAX * math.exp(-kv * (et * et + al * al)), max(-W_MAX, min(W_MAX, ka * al + kd * et))
        x, y, a = step(x, y, a, v, w, DT)
        out.append((k * DT, y))
    return np.array(out)


def metrics(r):
    t, y = r[:, 0], r[:, 1]
    over = max(0.0, -y.min()) / Y0
    big = np.where(np.abs(y) > 0.05 * Y0)[0]
    return over, (t[big[-1]] if len(big) else 0.0)


def pred_over(z):
    return math.exp(-math.pi * z / math.sqrt(1 - z * z)) if z < 1 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="resultados")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for b in (0.1, 0.4, 1.0, 4.0):
        K = W_MAX / (2 * b)
        wn, z = math.sqrt(K * V / L_EFF), K / (2 * math.sqrt(K * V / L_EFF))
        o, ts = metrics(sim_pp(b))
        rows.append({"algoritmo": "pure_pursuit", "config": f"alpha=1, beta={b:g}", "ka_eq": K, "kd_eq": K / L_EFF,
                     "wn": wn, "zeta": z, "sobreimpulso_pred": pred_over(z), "sobreimpulso_sim": o, "t_asent_5pct_s": ts})
    for kd, ka in [(1, 1), (1, 0.5), (1, 2), (1, 5), (0.5, 1), (2, 1), (5, 1)]:
        wn = math.sqrt(kd * V)
        z = ka / (2 * wn)
        o, ts = metrics(sim_st(kd, ka))
        rows.append({"algoritmo": "stanley", "config": f"kd={kd:g}, ka={ka:g}", "ka_eq": ka, "kd_eq": kd,
                     "wn": wn, "zeta": z, "sobreimpulso_pred": pred_over(z), "sobreimpulso_sim": o, "t_asent_5pct_s": ts})
    with open(os.path.join(args.out, "linealizacion_sobreimpulso.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    print(f"{'algoritmo':<13}{'config':<20}{'ka_eq':>6}{'kd_eq':>7}{'wn':>6}{'zeta':>6}{'S_pred%':>9}{'S_sim%':>8}{'ts(s)':>7}")
    for r in rows:
        print(f"{r['algoritmo']:<13}{r['config']:<20}{r['ka_eq']:>6.2f}{r['kd_eq']:>7.2f}{r['wn']:>6.2f}{r['zeta']:>6.2f}"
              f"{r['sobreimpulso_pred'] * 100:>9.1f}{r['sobreimpulso_sim'] * 100:>8.1f}{r['t_asent_5pct_s']:>7.1f}")


if __name__ == "__main__":
    main()
