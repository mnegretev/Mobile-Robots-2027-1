#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verificación de orden de magnitud del error de régimen de Stanley en tramos curvos.

Stanley (versión del curso):  w = Ka*psi_e + Kd*e_t   (sin término de curvatura).
Para seguir un arco de curvatura k a velocidad v hace falta w = k*v. Si el robot
queda alineado con la tangente (psi_e ~ 0) el error de equilibrio es

        e_t* = k * v / Kd .

Aquí se compara esa predicción con el error transversal RMS medido (sólo corridas
de Stanley), usando k_rms = raíz de la media cuadrática de la curvatura de la ruta
(heading de la polilínea vs longitud de arco, suavizado con una ventana de 0.5 m) y
la velocidad media v_media del resumen de métricas.

Uso:
    python3 steady_state_check.py DATOS/ --meta runs.csv --metrics resumen_metricas.csv --out RESULTADOS/
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_path_following import cross_track  # noqa: E402


def curvatura(path, ventana_m=0.5):
    """Curvatura con signo k(s) = d(theta)/ds de la polilínea, promediada en 'ventana_m'."""
    seg = np.diff(path, axis=0)
    ds = np.hypot(seg[:, 0], seg[:, 1])
    keep = ds > 1e-9
    seg, ds = seg[keep], ds[keep]
    th = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    s = np.r_[0.0, np.cumsum(ds)]
    sm = 0.5 * (s[1:] + s[:-1])
    # derivada por diferencias en una ventana de longitud de arco
    k = np.zeros_like(sm)
    h = ventana_m / 2.0
    for i, si in enumerate(sm):
        a = np.searchsorted(sm, si - h)
        b = np.searchsorted(sm, si + h) - 1
        b = min(max(b, a + 1), len(sm) - 1)
        a = min(a, b - 1) if b > 0 else 0
        k[i] = (th[b] - th[a]) / max(sm[b] - sm[a], 1e-9)
    return sm, k


def sesgo_en_curvas(data_dir, metas, k_umbral=0.3):
    """Offset medio hacia el EXTERIOR de la curva (m) en los tramos con |k| > k_umbral.

    d_sgn > 0 = robot a la izquierda de la ruta (analyze_path_following.cross_track).
    En una curva a la izquierda (k > 0) el exterior es la derecha (d_sgn < 0), por eso
    exterior = -d_sgn * sign(k).  Positivo = el robot se abre; negativo = corta la curva.
    """
    out = []
    for m in metas:
        alg, suf = m["algoritmo"], m["suffix"]
        g = os.path.join(data_dir, f"goal_{alg}{suf}.txt")
        r = os.path.join(data_dir, f"robot_{alg}{suf}.txt")
        if not (os.path.exists(g) and os.path.exists(r)):
            continue
        path = np.loadtxt(g, delimiter=",", ndmin=2)[:, :2]
        real = np.loadtxt(r, delimiter=",", ndmin=2)
        k_end = int(np.flatnonzero(real[:, 3] > 0)[-1]) + 1
        pts = real[:k_end, :2]
        keep = np.r_[True, np.any(np.diff(pts, axis=0) != 0.0, axis=1)]
        pts = pts[keep]
        _, d_sgn = cross_track(pts, path)
        sm, k = curvatura(path)
        # curvatura en el vértice más cercano (k está definida en puntos medios de segmento)
        idx = np.argmin(np.hypot(pts[:, None, 0] - path[None, :-1, 0], pts[:, None, 1] - path[None, :-1, 1]), axis=1)
        kk = k[np.minimum(idx, len(k) - 1)]
        sel = np.abs(kk) > k_umbral
        ext = -d_sgn[sel] * np.sign(kk[sel])
        out.append({"suffix": suf, "algoritmo": alg, "ruta": m["route"],
                    "n_curva": int(sel.sum()), "n_total": int(len(pts)),
                    "exterior_medio_m": float(ext.mean()) if sel.any() else float("nan"),
                    "frac_exterior": float((ext > 0).mean()) if sel.any() else float("nan")})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir")
    ap.add_argument("--meta", required=True)
    ap.add_argument("--metrics", required=True)
    ap.add_argument("--out", default="resultados")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    with open(args.metrics, newline="", encoding="utf-8") as f:
        met = {r["suffix"]: r for r in csv.DictReader(f)}
    with open(args.meta, newline="", encoding="utf-8") as f:
        metas = list(csv.DictReader(f))

    rutas = {}
    rows = []
    for m in metas:
        if m["algoritmo"] != "stanley" or m["suffix"] not in met:
            continue
        g = os.path.join(args.data_dir, f"goal_stanley{m['suffix']}.txt")
        path = np.loadtxt(g, delimiter=",", ndmin=2)[:, :2]
        if m["route"] not in rutas:
            _, k = curvatura(path)
            rutas[m["route"]] = (float(np.sqrt(np.mean(k ** 2))), float(np.max(np.abs(k))))
        krms, kmax = rutas[m["route"]]
        mm = met[m["suffix"]]
        Kd, Ka = float(m["Kd"]), float(m["Ka"])
        v = float(mm["v_media_ms"])
        rms = float(mm["cte_rmse_m"])
        rows.append({"ruta": m["route"], "Kd": Kd, "Ka": Ka, "v_media": v, "k_rms": krms,
                     "k_max": kmax, "e_pred_m": krms * v / Kd, "rms_medido_m": rms,
                     "cociente": rms / (krms * v / Kd)})
    rows.sort(key=lambda r: (r["ruta"], r["Kd"], r["Ka"]))
    with open(os.path.join(args.out, "estado_estacionario_stanley.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    sesgo = sesgo_en_curvas(args.data_dir, metas)
    with open(os.path.join(args.out, "sesgo_en_curvas.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(sesgo[0].keys()))
        wr.writeheader()
        wr.writerows(sesgo)
    prom = {}
    for r in sesgo:
        prom.setdefault((r["algoritmo"], r["ruta"]), []).append(r["exterior_medio_m"])
    for (alg, rt), v in sorted(prom.items()):
        v = np.array(v)
        print(f"sesgo exterior medio {alg:<12} {rt}: media de corridas {np.nanmean(v):+.3f} m  "
              f"(min {np.nanmin(v):+.3f}, max {np.nanmax(v):+.3f}, n={len(v)})")
    for r, (kr, km) in rutas.items():
        print(f"{r}: curvatura rms = {kr:.3f} 1/m, máxima = {km:.3f} 1/m (radio mínimo {1 / km:.2f} m)")
    print(f"{'ruta':<4}{'Kd':>5}{'Ka':>5}{'v_med':>8}{'e_pred':>9}{'RMS med':>9}{'cociente':>10}")
    for r in rows:
        print(f"{r['ruta']:<4}{r['Kd']:>5g}{r['Ka']:>5g}{r['v_media']:>8.3f}{r['e_pred_m']:>9.3f}"
              f"{r['rms_medido_m']:>9.3f}{r['cociente']:>10.2f}")


if __name__ == "__main__":
    main()
