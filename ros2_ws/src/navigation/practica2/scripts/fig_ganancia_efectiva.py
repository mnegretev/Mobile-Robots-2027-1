#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Error transversal RMS medido vs ganancia transversal efectiva k_d,eq de cada configuración.

Cerca de una ruta recta, pure pursuit equivale a una ley tipo Stanley con
        k_a,eq = K = w_max/(2*beta)   y   k_d,eq = K/L ,   L ~ 0.3 m
(ver reporte, sección de dinámica del error transversal); para Stanley k_d,eq = k_d.

Uso:  python3 fig_ganancia_efectiva.py resumen_metricas.csv --out RESULTADOS/
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_path_following import SURFACE, INK, INK2, GRID, SERIES  # noqa: E402

L_EFF, W_MAX = 0.3, 1.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("metricas")
    ap.add_argument("--out", default="resultados")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = list(csv.DictReader(open(args.metricas, newline="", encoding="utf-8")))
    plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                         "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "axes.grid": True, "grid.color": GRID, "axes.spines.top": False, "axes.spines.right": False,
                         "legend.frameon": False, "savefig.dpi": 200, "mathtext.default": "regular", "text.color": INK})
    fig, axs = plt.subplots(1, 2, figsize=(10.2, 4.2), sharey=False)
    out = []
    for ax, rt in zip(axs, ("R1", "R2")):
        for k, (alg, name, mk) in enumerate([("pure_pursuit", "Pure pursuit (α y β variados)", "o"),
                                             ("stanley", "Stanley ($k_d$ y $k_a$ variados)", "s")]):
            xs, ys = [], []
            for r in rows:
                if r["algoritmo"] != alg or r["ruta"] != rt:
                    continue
                if alg == "pure_pursuit":
                    kd = W_MAX / (2 * float(r["beta"])) / L_EFF
                else:
                    kd = float(r["Kd"])
                xs.append(kd)
                ys.append(float(r["cte_rmse_m"]))
                out.append({"ruta": rt, "algoritmo": alg, "kd_eq": kd, "rms_m": float(r["cte_rmse_m"])})
            ax.plot(xs, ys, mk, ms=7, color=SERIES[k], mec=SURFACE, mew=1.0, label=name, ls="none")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("Ganancia transversal efectiva $k_{d,eq}$ (rad/s por m)")
        ax.set_ylabel("Error transversal RMS (m)")
        ax.set_title(f"Ruta {rt}", loc="left", fontsize=10.5)
    axs[0].legend(loc="lower left", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "fig_ganancia_efectiva.png"), bbox_inches="tight")
    plt.close(fig)
    with open(os.path.join(args.out, "ganancia_efectiva.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        wr.writeheader()
        wr.writerows(out)
    for rt in ("R1", "R2"):
        a = np.array([[o["kd_eq"], o["rms_m"]] for o in out if o["ruta"] == rt])
        c = np.corrcoef(np.log(a[:, 0]), np.log(a[:, 1]))[0, 1]
        print(f"{rt}: correlación log-log (k_d,eq vs RMS), {len(a)} corridas: {c:.2f}")


if __name__ == "__main__":
    main()
