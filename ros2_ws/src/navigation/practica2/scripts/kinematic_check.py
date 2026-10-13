#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verificación con un modelo cinemático ideal (Práctica 2, seguimiento de rutas).

Reproduce, SIN Gazebo, cada corrida de runs.csv: integra un robot diferencial
ideal (v, w constantes durante cada ciclo, integración exacta del unicíclo) con
las MISMAS leyes de control del código entregado (pure_pursuit.py / stanley.py),
sobre la MISMA ruta (goal_*.txt) y desde la MISMA pose inicial (primera fila de
robot_*.txt), y compara el error transversal RMS simulado con el medido.

Sirve para:
  1. validar la métrica y el montaje experimental (si el modelo ideal reproduce
     lo medido, el resultado depende de la ley de control y no del ruido);
  2. explicar corridas que no terminaron (--extra), p. ej. Stanley Kd=0.5, Ka=1;
  3. probar hipótesis baratas (frecuencia de control: --st-dt).

Uso:
    python3 kinematic_check.py DATOS/ --meta runs.csv --out RESULTADOS/ \
            [--extra st:0.5:1.0:R2 pp:1.0:4.0:R2] [--st-dt 0.175] [--pp-dt 0.0175]

Frecuencias por defecto = las medidas en el robot: pure pursuit ~57 Hz
(dt = 0.0175 s), Stanley ~5.7 Hz (dt = 0.175 s, la plantilla hace 10 spin_once
por ciclo).  El modelo NO incluye aceleración finita, retardos ni fricción.
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
from analyze_path_following import cross_track, SURFACE, INK2, GRID, SERIES  # noqa: E402

V_MAX, W_MAX, TOL_D, KV = 0.5, 1.0, 0.3, 5.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def step(x, y, a, v, w, dt):
    if abs(w) > 1e-9:
        x += v / w * (math.sin(a + w * dt) - math.sin(a))
        y += v / w * (-math.cos(a + w * dt) + math.cos(a))
        a = wrap(a + w * dt)
    else:
        x += v * dt * math.cos(a)
        y += v * dt * math.sin(a)
    return x, y, a


def sim_pp(path, pose, alpha, beta, dt, tmax):
    x, y, a = pose
    idx, t, tr = 0, 0.0, [(x, y)]
    while t < tmax:
        if math.hypot(path[-1, 0] - x, path[-1, 1] - y) <= TOL_D:
            return np.array(tr), t, True
        gx, gy = path[idx]
        e = wrap(math.atan2(gy - y, gx - x) - a)
        v = V_MAX * math.exp(-e * e / alpha)
        w = W_MAX * math.tanh(e / (2 * beta))
        x, y, a = step(x, y, a, v, w, dt)
        if math.hypot(gx - x, gy - y) < 0.3:
            idx = min(idx + 1, len(path) - 1)
        t += dt
        tr.append((x, y))
    return np.array(tr), t, False


def sim_st(path, pose, Kd, Ka, dt, tmax):
    x, y, a = pose
    t, tr = 0.0, [(x, y)]
    last = (0.0, 0.0, 0.0)
    while t < tmax:
        if math.hypot(path[-1, 0] - x, path[-1, 1] - y) <= TOL_D:
            return np.array(tr), t, True, last
        d = np.hypot(path[:, 0] - x, path[:, 1] - y)
        i = int(np.argmin(d))
        xi, yi = path[i]
        n, p = min(i + 1, len(path) - 1), max(i - 1, 0)
        ti = math.atan2(path[n, 1] - path[p, 1], path[n, 0] - path[p, 0])
        te = wrap(ti - math.atan2(y - yi, x - xi))
        et = (1.0 if te > 0 else -1.0 if te < 0 else 0.0) * math.hypot(x - xi, y - yi)
        al = wrap(ti - a)
        v = V_MAX * math.exp(-KV * (et * et + al * al))
        w = max(-W_MAX, min(W_MAX, Ka * al + Kd * et))
        last = (v, et, al)
        x, y, a = step(x, y, a, v, w, dt)
        t += dt
        tr.append((x, y))
    return np.array(tr), t, False, last


def simulate(alg, path, pose, p1, p2, pp_dt, st_dt, tmax):
    if alg == "pure_pursuit":
        tr, t, ok = sim_pp(path, pose, p1, p2, pp_dt, tmax)
        return tr, t, ok, None
    return sim_st(path, pose, p1, p2, st_dt, tmax)


def rmse(tr, path):
    d, _ = cross_track(tr, path)
    return float(np.sqrt(np.mean(d ** 2)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir")
    ap.add_argument("--meta", required=True)
    ap.add_argument("--out", default="resultados")
    ap.add_argument("--extra", nargs="*", default=[], help="corridas sin datos: pp:alpha:beta:R2  st:Kd:Ka:R2")
    ap.add_argument("--pp-dt", type=float, default=0.0175)
    ap.add_argument("--st-dt", type=float, default=0.175)
    ap.add_argument("--tmax", type=float, default=300.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows, route_path, route_pose = [], {}, {}
    with open(args.meta, newline="", encoding="utf-8") as f:
        metas = list(csv.DictReader(f))
    for m in metas:
        alg, suf = m["algoritmo"], m["suffix"]
        g = os.path.join(args.data_dir, f"goal_{alg}{suf}.txt")
        r = os.path.join(args.data_dir, f"robot_{alg}{suf}.txt")
        if not (os.path.exists(g) and os.path.exists(r)):
            continue
        path = np.loadtxt(g, delimiter=",", ndmin=2)[:, :2]
        real = np.loadtxt(r, delimiter=",", ndmin=2)
        pose = (real[0, 0], real[0, 1], real[0, 2])
        route_path.setdefault(m["route"], path)
        route_pose.setdefault(m["route"], pose)
        # RMS real con el mismo criterio que analyze_path_following.py
        pts = real[:, :2]
        k_end = int(np.flatnonzero(real[:, 3] > 0)[-1]) + 1
        changed = np.r_[True, np.any(np.diff(pts, axis=0) != 0.0, axis=1)]
        sel = np.zeros(len(pts), dtype=bool)
        sel[:k_end] = True
        sel &= changed
        d, _ = cross_track(pts, path)
        rms_real = float(np.sqrt(np.mean(d[sel] ** 2)))
        p1, p2 = (float(m["alpha"]), float(m["beta"])) if alg == "pure_pursuit" else (float(m["Kd"]), float(m["Ka"]))
        tr, t, ok, _ = simulate(alg, path, pose, p1, p2, args.pp_dt, args.st_dt, args.tmax)
        rows.append({"suffix": suf, "algoritmo": alg, "ruta": m["route"], "p1": p1, "p2": p2,
                     "rms_real_m": rms_real, "rms_sim_m": rmse(tr, path) if ok else float("nan"),
                     "t_sim_s": t, "termina_sim": ok})

    extras = []
    for e in args.extra:
        a, p1, p2, rt = e.split(":")
        alg = "pure_pursuit" if a == "pp" else "stanley"
        tr, t, ok, last = simulate(alg, route_path[rt], route_pose[rt], float(p1), float(p2),
                                   args.pp_dt, args.st_dt, args.tmax)
        dmin = float(np.min(np.hypot(tr[:, 0] - route_path[rt][-1, 0], tr[:, 1] - route_path[rt][-1, 1])))
        extras.append({"corrida": e, "termina_sim": ok, "t_s": t, "dist_min_a_meta_m": dmin,
                       "pos_final": (float(tr[-1, 0]), float(tr[-1, 1])),
                       "v_final": last[0] if last else None, "et_final": last[1] if last else None,
                       "alpha_final": last[2] if last else None})

    with open(os.path.join(args.out, "validacion_cinematica.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    ok_rows = [r for r in rows if r["termina_sim"]]
    xr = np.array([r["rms_real_m"] for r in ok_rows])
    ys = np.array([r["rms_sim_m"] for r in ok_rows])
    rel = np.abs(ys - xr) / xr
    print(f"{len(ok_rows)}/{len(rows)} corridas reproducidas por el modelo ideal")
    print(f"  correlación (Pearson, RMS real vs sim): {np.corrcoef(xr, ys)[0, 1]:.3f}")
    print(f"  error relativo mediano: {np.median(rel) * 100:.0f} %   (máx {np.max(rel) * 100:.0f} %)")
    for alg in ("pure_pursuit", "stanley"):
        s = [r for r in ok_rows if r["algoritmo"] == alg]
        if s:
            rr = np.array([abs(r["rms_sim_m"] - r["rms_real_m"]) / r["rms_real_m"] for r in s])
            print(f"  {alg:<13} n={len(s):2d}  error relativo mediano {np.median(rr) * 100:.0f} %")
    for e in extras:
        print(f"  extra {e['corrida']}: termina={e['termina_sim']}  dist. mínima a la meta={e['dist_min_a_meta_m']:.2f} m"
              + ("" if e["termina_sim"] else
                 f"  | estado final: pos=({e['pos_final'][0]:.2f},{e['pos_final'][1]:.2f}) v={e['v_final']:.4f} "
                 f"et={e['et_final']:.2f} alpha={e['alpha_final']:.2f}"))

    # figura: RMS simulado vs medido
    plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                         "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "axes.grid": True, "grid.color": GRID, "axes.spines.top": False, "axes.spines.right": False,
                         "legend.frameon": False, "figure.dpi": 100, "savefig.dpi": 200})
    fig, ax = plt.subplots(figsize=(5.4, 5.0))
    lim = [0.004, 0.35]
    ax.plot(lim, lim, color=INK2, lw=1.0, ls="--", label="sim = real")
    for k, (alg, name) in enumerate([("pure_pursuit", "Pure pursuit"), ("stanley", "Stanley")]):
        s = [r for r in ok_rows if r["algoritmo"] == alg]
        ax.plot([r["rms_real_m"] for r in s], [r["rms_sim_m"] for r in s], "o", ms=6, color=SERIES[k],
                mec=SURFACE, mew=1.0, label=name)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.set_aspect("equal")
    ax.set_xlabel("Error transversal RMS medido en Gazebo (m)")
    ax.set_ylabel("Error transversal RMS en el modelo ideal (m)")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "fig_validacion_cinematica.png"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
