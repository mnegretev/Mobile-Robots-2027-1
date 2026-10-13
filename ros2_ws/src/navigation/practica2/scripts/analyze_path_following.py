#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Análisis de la Práctica 2 (seguimiento de rutas) - Robots Móviles FI-UNAM 2027-1.

Lee los archivos que guardan los nodos pure_pursuit.py y stanley.py:
    goal_<alg><sufijo>.txt    -> ruta deseada, columnas: x, y
    robot_<alg><sufijo>.txt   -> datos del robot, columnas: x, y, theta, v, w
                                 (una 6a columna opcional = tiempo en s)
y calcula métricas y figuras para el reporte.

Uso básico (carpeta con todos los .txt):
    python3 analyze_path_following.py DATOS/ --out RESULTADOS/ --meta runs.csv

runs.csv (opcional, una fila por corrida) - columnas:
    suffix      sufijo de fecha del archivo, p. ej. 20261013T113630
    label       etiqueta corta, p. ej. "alpha=0.1, beta=0.1"
    route       nombre de la ruta, p. ej. "R1"
    duration_s  duración total de la corrida en s (opcional, ver abajo)

Comparación de varias corridas en una sola figura:
    ... --compare 20261013T113630 20261013T114501 [...]

SOBRE EL TIEMPO: los .txt originales NO guardan marca de tiempo. Si no hay 6a
columna, el tiempo se estima repartiendo 'duration_s' uniformemente entre las
muestras (duration_s = hora de "Global goal point reached" - hora de
"Following path with ..." en el log del nodo). Sin duration_s, las gráficas
usan el número de muestra y no se reporta tiempo.

MÉTRICAS (fase de seguimiento = hasta la última muestra con v > 0; después
sólo hay giro en sitio para la orientación final):
    cte_rmse, cte_max, cte_media : distancia de cada pose a la polilínea de la
                                   ruta deseada (m). Se usan sólo muestras con
                                   pose distinta de la anterior para no pesar
                                   lecturas repetidas de la TF.
    cte_ini                      : distancia de la pose inicial a la ruta (m); es
                                   un desfase de arranque, no un error del control
    cte_max_reg                  : máximo de |CTE| una vez recorridos 0.3 m (sin
                                   el desfase de arranque)
    err_pos_final                : distancia del último punto del robot a la meta (m)
    long_recorrida / long_ruta   : razón entre el camino recorrido y la ruta
    v_media, v_max, w_rms        : estadísticas de las señales de control
    var_total_w                  : suma de |w[k+1]-w[k]| (qué tan agitada es w)
    cruces_w                     : cambios de signo de w con zona muerta 0.02 rad/s
    var_w_5hz, cruces_w_5hz      : las dos anteriores calculadas sobre w promediada en
                                   ventanas de 0.2 s. PP y Stanley NO corren al mismo
                                   ritmo (PP ~60 Hz, Stanley ~6 Hz porque la plantilla
                                   hace 10 spin_once por ciclo), así que sólo estas dos
                                   son comparables entre algoritmos. Requieren tiempo.
    f_control_hz                 : ciclos de control por segundo (muestras / tiempo)
"""
import argparse
import csv
import glob
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ---------------------------------------------------------------- estilo ----
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # orden fijo de categorías

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "text.color": INK,
    "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.titlesize": 11, "axes.labelsize": 10, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "legend.frameon": False, "legend.fontsize": 9, "lines.linewidth": 1.8,
    "figure.dpi": 100, "savefig.dpi": 200,
})

W_DEADBAND = 0.02  # rad/s

plt.rcParams["mathtext.default"] = "regular"


def pretty(label):
    """'alpha=1.0, beta=0.4' -> 'α = 1, β = 0.4';  'Kd=1.0, Ka=2.0' -> '$k_d$ = 1, $k_a$ = 2' (sólo para figuras)."""
    sym = {"alpha": "α", "beta": "β", "Kd": "$k_d$", "Ka": "$k_a$"}

    def rep(m):
        return f"{sym[m.group(1)]} = {float(m.group(2)):g}"

    return re.sub(r"\b(alpha|beta|Kd|Ka)=([-0-9.]+)", rep, label)


# ---------------------------------------------------------------- carga -----
def load_run(robot_file, goal_file, duration=None):
    r = np.loadtxt(robot_file, delimiter=",", ndmin=2)
    g = np.loadtxt(goal_file, delimiter=",", ndmin=2)
    if r.shape[1] < 5 or g.shape[1] < 2:
        raise ValueError(f"Formato inesperado en {robot_file} / {goal_file}")
    run = {"x": r[:, 0], "y": r[:, 1], "th": r[:, 2], "v": r[:, 3], "w": r[:, 4], "path": g[:, :2]}
    n = len(r)
    if r.shape[1] >= 6:
        run["t"] = r[:, 5] - r[0, 5]
        run["t_kind"] = "medido"
    elif duration:
        run["t"] = np.linspace(0.0, duration, n)
        run["t_kind"] = "estimado"
    else:
        run["t"] = np.arange(n, dtype=float)
        run["t_kind"] = "muestra"
    return run


# --------------------------------------------------------------- métricas ---
def cross_track(points, path):
    """Distancia con signo (+ izquierda de la ruta) de cada punto a la polilínea."""
    a, b = path[:-1], path[1:]
    ab = b - a
    L2 = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    d_abs = np.empty(len(points))
    d_sgn = np.empty(len(points))
    for i, p in enumerate(points):
        t = np.clip(np.sum((p - a) * ab, axis=1) / L2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d = np.hypot(*(p - proj).T)
        j = int(np.argmin(d))
        cross = ab[j, 0] * (p[1] - a[j, 1]) - ab[j, 1] * (p[0] - a[j, 0])
        d_abs[i] = d[j]
        d_sgn[i] = d[j] if cross >= 0 else -d[j]
    return d_abs, d_sgn


def binned_mean(t, sig, dt):
    """Promedio de sig en ventanas de dt segundos (quita la dependencia de la tasa de muestreo)."""
    idx = np.floor((t - t[0]) / dt).astype(int)
    return np.array([sig[idx == i].mean() for i in np.unique(idx)])


def compute_metrics(run):
    x, y, v, w, path = run["x"], run["y"], run["v"], run["w"], run["path"]
    n = len(x)
    moving_idx = np.flatnonzero(v > 0.0)
    k_end = int(moving_idx[-1]) + 1 if len(moving_idx) else n          # fin de seguimiento
    pts = np.column_stack([x, y])
    cte_abs, cte_sgn = cross_track(pts, path)
    run["cte_abs"], run["cte"] = cte_abs, cte_sgn

    # muestras de seguimiento con pose distinta de la anterior
    changed = np.r_[True, np.any(np.diff(pts, axis=0) != 0.0, axis=1)]
    track = np.zeros(n, dtype=bool)
    track[:k_end] = True
    sel = track & changed
    run["k_end"] = k_end

    dist_cum = np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    reg = sel & (dist_cum > 0.3)
    if not reg.any():
        reg = sel

    seg = np.hypot(np.diff(x[:k_end]), np.diff(y[:k_end]))
    path_len = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))
    w_t = w[:k_end]
    dead = np.where(np.abs(w_t) < W_DEADBAND, 0.0, np.sign(w_t))
    nz = dead[dead != 0.0]
    crossings = int(np.sum(nz[1:] != nz[:-1])) if len(nz) > 1 else 0

    m = {
        "n_muestras": n,
        "n_muestras_seguimiento": k_end,
        "cte_rmse_m": float(np.sqrt(np.mean(cte_abs[sel] ** 2))),
        "cte_max_m": float(np.max(cte_abs[sel])),
        "cte_ini_m": float(cte_abs[0]),
        "cte_max_reg_m": float(np.max(cte_abs[reg])),
        "cte_media_m": float(np.mean(cte_abs[sel])),
        "err_pos_final_m": float(np.hypot(*(pts[-1] - path[-1]))),
        "long_recorrida_m": float(np.sum(seg)),
        "long_ruta_m": path_len,
        "razon_long": float(np.sum(seg) / path_len),
        "v_media_ms": float(np.mean(v[:k_end])),
        "v_max_ms": float(np.max(v[:k_end])),
        "w_rms_rads": float(np.sqrt(np.mean(w_t ** 2))),
        "w_max_abs_rads": float(np.max(np.abs(w_t))),
        "var_total_w": float(np.sum(np.abs(np.diff(w_t)))),
        "cruces_w": crossings,
    }
    if run["t_kind"] != "muestra":
        t_seg = float(run["t"][k_end - 1])
        m["t_seguimiento_s"] = t_seg
        m["t_total_s"] = float(run["t"][-1])
        m["f_control_hz"] = float((k_end - 1) / t_seg) if t_seg > 0 else float("nan")
        wb = binned_mean(run["t"][:k_end], w_t, 0.2)
        m["var_w_5hz"] = float(np.sum(np.abs(np.diff(wb))))
        db = np.where(np.abs(wb) < W_DEADBAND, 0.0, np.sign(wb))
        nzb = db[db != 0.0]
        m["cruces_w_5hz"] = int(np.sum(nzb[1:] != nzb[:-1])) if len(nzb) > 1 else 0
    return m


# ---------------------------------------------------------------- figuras ---
def xlabel_for(run):
    return {"medido": "Tiempo (s)", "estimado": "Tiempo estimado (s)", "muestra": "Muestra"}[run["t_kind"]]


def is_tall(runs):
    """Rutas más altas que anchas (p. ej. R2): la trayectoria va en una columna angosta a la izquierda."""
    pts = np.vstack([np.column_stack([r["x"], r["y"]]) for r in runs] + [runs[0]["path"]])
    dx, dy = np.ptp(pts[:, 0]), np.ptp(pts[:, 1])
    return dy > 1.3 * dx


def draw_trajectory(ax, runs, labels, title, extra=None):
    ax.plot(runs[0]["path"][:, 0], runs[0]["path"][:, 1], color=INK2, lw=1.4, ls="--", label="Ruta deseada")
    for i, (r, lab) in enumerate(zip(runs, labels)):
        ax.plot(r["x"], r["y"], color=SERIES[i], lw=1.8, label=pretty(lab))
        if extra:
            ax.plot(r["x"][0], r["y"][0], "o", ms=7, color=SERIES[i], mec=SURFACE, mew=1.5)
    if extra:
        ax.plot(runs[0]["path"][-1, 0], runs[0]["path"][-1, 1], "s", ms=7, color=SERIES[1], mec=SURFACE, mew=1.5,
                label="Meta")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(title, loc="left", fontsize=10)


def _layout(runs, n_sig, figsize_wide):
    """Devuelve (fig, ax_trayectoria, [ejes de señales], tall)."""
    tall = is_tall(runs)
    if tall:
        fig = plt.figure(figsize=(11, 2.35 * n_sig + 1.0))
        gs = GridSpec(n_sig, 2, width_ratios=[0.62, 1.38], hspace=0.5, wspace=0.28, figure=fig)
        ax = fig.add_subplot(gs[:, 0])
        sig = [fig.add_subplot(gs[i, 1]) for i in range(n_sig)]
        ax.set_aspect("equal", adjustable="datalim")
    else:
        fig = plt.figure(figsize=figsize_wide)
        gs = GridSpec(2, n_sig, height_ratios=[1.25, 1], hspace=0.42, wspace=0.28, figure=fig)
        ax = fig.add_subplot(gs[0, :])
        sig = [fig.add_subplot(gs[1, i]) for i in range(n_sig)]
        ax.set_aspect("equal", adjustable="datalim")
    return fig, ax, sig, tall


def fig_single(run, title, outfile):
    t, k = run["t"], run["k_end"]
    fig, ax, sig, tall = _layout([run], 3, (11, 7.2))
    draw_trajectory(ax, [run], ["Trayectoria del robot"], title, extra=True)
    if tall:
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=8)
    else:
        ax.legend(loc="best", ncol=4)
    for a, (key, lab, color) in zip(sig, [("v", "v (m/s)", SERIES[0]),
                                          ("w", "ω (rad/s)", SERIES[1]),
                                          ("cte", "Error transversal (m)", SERIES[2])]):
        a.plot(t, run[key], color=color, lw=1.6, label=lab)
        a.axvline(t[k - 1], color=INK2, lw=0.9, ls=":", label="Fin del seguimiento")
        if key in ("w", "cte"):
            a.axhline(0, color=INK2, lw=0.8)
        a.set_xlabel(xlabel_for(run)); a.set_ylabel(lab)
        a.legend(loc="best", fontsize=8)
    fig.savefig(outfile, bbox_inches="tight")
    plt.close(fig)


def fig_compare(runs, labels, outfile, title=None):
    fig, ax, sig, tall = _layout(runs, 3, (11, 7.2))
    draw_trajectory(ax, runs, labels, title or "Trayectorias")
    if tall:
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=1, fontsize=8)
    else:
        ax.legend(loc="best", ncol=min(len(runs) + 1, 3))
    for a, (key, lab) in zip(sig, [("v", "v (m/s)"), ("w", "ω (rad/s)"), ("cte", "Error transversal (m)")]):
        for i, (r, l) in enumerate(zip(runs, labels)):
            a.plot(r["t"][:r["k_end"]], r[key][:r["k_end"]], color=SERIES[i], lw=1.6, label=pretty(l))
        if key != "v":
            a.axhline(0, color=INK2, lw=0.8)
        a.set_xlabel(xlabel_for(runs[0])); a.set_ylabel(lab)
    fig.savefig(outfile, bbox_inches="tight")
    plt.close(fig)


# ------------------------------------------------- promedios y barridos ------
PARAM_KEYS = ["alpha", "beta", "Kd", "Ka"]
MEAN_METRICS = ["cte_rmse_m", "cte_max_m", "cte_max_reg_m", "cte_media_m", "err_pos_final_m",
                "razon_long", "v_media_ms", "w_rms_rads", "var_total_w", "cruces_w",
                "t_seguimiento_s", "var_w_5hz", "cruces_w_5hz"]
SD_METRICS = ["cte_rmse_m", "t_seguimiento_s"]
ALG_NAME = {"pure_pursuit": "Pure pursuit", "stanley": "Stanley"}
# (parámetro que varía, parámetros fijos del barrido, etiqueta del eje x)
SWEEPS = {
    "pure_pursuit": [("alpha", {"beta": 0.4}, "α  (β = 0.4)"), ("beta", {"alpha": 1.0}, "β  (α = 1.0)")],
    "stanley": [("Ka", {"Kd": 1.0}, "$k_a$  ($k_d$ = 1.0)"), ("Kd", {"Ka": 1.0}, "$k_d$  ($k_a$ = 1.0)")],
}
BASELINE = {"pure_pursuit": {"alpha": 1.0, "beta": 0.4}, "stanley": {"Kd": 1.0, "Ka": 1.0}}


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def group_means(rows):
    """Promedia las réplicas (mismo algoritmo, ruta y parámetros)."""
    groups = {}
    for r in rows:
        key = (r["algoritmo"], r.get("ruta", ""), *(str(r.get(k, "")) for k in PARAM_KEYS),
               str(r.get("v_max", "")), str(r.get("w_max", "")))
        groups.setdefault(key, []).append(r)
    out = []
    for rs in groups.values():
        g = {"algoritmo": rs[0]["algoritmo"], "ruta": rs[0].get("ruta", "")}
        for k in PARAM_KEYS + ["v_max", "w_max"]:
            g[k] = _num(rs[0].get(k))
        g["n_rep"] = len(rs)
        g["sufijos"] = " ".join(r["suffix"] for r in rs)
        for m in MEAN_METRICS:
            vals = [r[m] for r in rs if isinstance(r.get(m), (int, float))]
            if vals:
                g[m] = float(np.mean(vals))
                if m in SD_METRICS:
                    g[m + "_sd"] = float(np.std(vals, ddof=1)) if len(vals) > 1 else float("nan")
        out.append(g)
    out.sort(key=lambda g: (g["algoritmo"], g["ruta"], *(g[k] if g[k] is not None else -1 for k in PARAM_KEYS)))
    return out


def _match(g, fixed):
    return all(g.get(k) is not None and abs(g[k] - v) < 1e-9 for k, v in fixed.items())


def fig_sweeps(means, alg, outfile):
    """Una fila por métrica, una columna por parámetro; una línea por ruta."""
    sw = SWEEPS[alg]
    sel = [g for g in means if g["algoritmo"] == alg]
    routes = sorted({g["ruta"] for g in sel})
    metrics = [("cte_rmse_m", "Error transversal RMS (m)")]
    if any("t_seguimiento_s" in g for g in sel):
        metrics += [("t_seguimiento_s", "Tiempo de seguimiento (s)"),
                    ("var_w_5hz", "Variación total de ω, 5 Hz (rad/s)")]
    else:
        metrics += [("w_rms_rads", "ω RMS (rad/s)")]
    fig, axes = plt.subplots(len(metrics), len(sw), figsize=(9.2, 2.9 * len(metrics)), squeeze=False)
    for c, (par, fixed, xlab) in enumerate(sw):
        pts_all = [g for g in sel if _match(g, fixed) and g.get(par) is not None]
        xs_all = sorted({g[par] for g in pts_all})
        for rr, (mk, ylab) in enumerate(metrics):
            ax = axes[rr][c]
            for i, rt in enumerate(routes):
                pts = sorted([g for g in pts_all if g["ruta"] == rt and mk in g], key=lambda g: g[par])
                if not pts:
                    continue
                ax.plot([g[par] for g in pts], [g[mk] for g in pts], "-o", color=SERIES[i], ms=5,
                        mec=SURFACE, mew=1.0, label=f"Ruta {rt}")
                if mk + "_sd" in pts[0]:
                    sd = [g.get(mk + "_sd", float("nan")) for g in pts]
                    if np.any(np.isfinite(sd)):
                        ax.errorbar([g[par] for g in pts], [g[mk] for g in pts], yerr=np.nan_to_num(sd),
                                    fmt="none", ecolor=SERIES[i], elinewidth=1.0, capsize=3)
            ax.set_xscale("log")
            ax.set_xticks(xs_all)
            ax.set_xticklabels([f"{x:g}" for x in xs_all])
            ax.minorticks_off()
            ax.set_ylim(bottom=0)
            ax.set_ylabel(ylab)
            if rr == len(metrics) - 1:
                ax.set_xlabel(xlab)
            if rr == 0 and c == 0:
                ax.legend(loc="best")
    fig.suptitle(f"{ALG_NAME[alg]}: efecto de cada parámetro (se varía uno a la vez)", x=0.01, ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(outfile, bbox_inches="tight")
    plt.close(fig)


def pick_configs(means):
    """Por algoritmo y ruta: configuración base y la de menor RMS transversal."""
    picks = []
    for alg in ALG_NAME:
        for rt in sorted({g["ruta"] for g in means if g["algoritmo"] == alg}):
            cand = [g for g in means if g["algoritmo"] == alg and g["ruta"] == rt]
            base = [g for g in cand if _match(g, BASELINE[alg])]
            best = min(cand, key=lambda g: g["cte_rmse_m"])
            picks.append({"alg": alg, "ruta": rt, "base": base[0] if base else None, "mejor": best})
    return picks


def _cfg_str(g):
    ks = ["alpha", "beta"] if g["algoritmo"] == "pure_pursuit" else ["Kd", "Ka"]
    sym = {"alpha": "α", "beta": "β"}
    return ", ".join(f"{sym.get(k, k)}={g[k]:g}" for k in ks)


def fig_algoritmos(means, outfile, md_file):
    from matplotlib.patches import Patch
    picks = pick_configs(means)
    routes = sorted({p["ruta"] for p in picks})
    has_t = any("t_seguimiento_s" in p["mejor"] for p in picks)
    metrics = [("cte_rmse_m", "Error transversal RMS (m)")]
    if has_t:
        metrics += [("t_seguimiento_s", "Tiempo de seguimiento (s)"), ("var_w_5hz", "Variación total de ω, 5 Hz (rad/s)")]
    else:
        metrics += [("w_rms_rads", "ω RMS (rad/s)")]
    fig, axes = plt.subplots(1, len(metrics), figsize=(3.7 * len(metrics) + 0.6, 3.6), squeeze=False)
    color = {"pure_pursuit": SERIES[0], "stanley": SERIES[1]}
    width = 0.2
    offs = {("pure_pursuit", "base"): -1.5, ("pure_pursuit", "mejor"): -0.5,
            ("stanley", "base"): 0.5, ("stanley", "mejor"): 1.5}
    for a, (mk, ylab) in zip(axes[0], metrics):
        for p in picks:
            xi = routes.index(p["ruta"])
            for kind in ("base", "mejor"):
                g = p[kind]
                if g is None or mk not in g:
                    continue
                a.bar(xi + offs[(p["alg"], kind)] * width, g[mk], width * 0.92, color=color[p["alg"]],
                      hatch=None if kind == "base" else "////", edgecolor=SURFACE if kind == "base" else "white",
                      linewidth=0.0 if kind == "base" else 0.0)
        a.set_xticks(range(len(routes)))
        a.set_xticklabels([f"Ruta {r}" for r in routes])
        a.set_ylabel(ylab)
        a.grid(axis="x", visible=False)
    axes[0][0].legend(handles=[Patch(color=color["pure_pursuit"], label="Pure pursuit, base"),
                               Patch(facecolor=color["pure_pursuit"], hatch="////", edgecolor="white", label="Pure pursuit, mejor RMS"),
                               Patch(color=color["stanley"], label="Stanley, base"),
                               Patch(facecolor=color["stanley"], hatch="////", edgecolor="white", label="Stanley, mejor RMS")],
                      loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(outfile, bbox_inches="tight")
    plt.close(fig)
    with open(md_file, "w", encoding="utf-8") as f:
        f.write("| algoritmo | ruta | configuración | parámetros | CTE RMS (m) | t seg. (s) | var. ω 5 Hz | réplicas |\n"
                "|---|---|---|---|---|---|---|---|\n")
        for p in picks:
            for kind in ("base", "mejor"):
                g = p[kind]
                if g is None:
                    continue
                f.write(f"| {ALG_NAME[p['alg']]} | {p['ruta']} | {kind} | {_cfg_str(g)} | {g['cte_rmse_m']:.4f} | "
                        f"{g.get('t_seguimiento_s', float('nan')):.1f} | {g.get('var_w_5hz', float('nan')):.2f} | {g['n_rep']} |\n")


# ------------------------------------------------------------------- main ---
def find_runs(data_dir):
    runs = {}
    for rf in sorted(glob.glob(os.path.join(data_dir, "robot_*.txt"))):
        name = os.path.basename(rf)
        mt = re.match(r"robot_(pure_pursuit|stanley)(\d{8}T\d{6})\.txt$", name)
        if not mt:
            continue
        alg, suf = mt.groups()
        gf = os.path.join(data_dir, f"goal_{alg}{suf}.txt")
        if os.path.exists(gf):
            runs[suf] = {"alg": alg, "robot": rf, "goal": gf}
        else:
            print(f"[aviso] falta {os.path.basename(gf)}; se omite {name}", file=sys.stderr)
    return runs


def read_meta(path):
    meta = {}
    if path:
        with open(path, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                meta[row["suffix"].strip()] = row
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir")
    ap.add_argument("--out", default="resultados")
    ap.add_argument("--meta", default=None, help="runs.csv con suffix,label,route,duration_s")
    ap.add_argument("--compare", nargs="+", default=None, help="sufijos a superponer (máx. 4)")
    ap.add_argument("--compare-out", default="fig_comparacion.png", help="nombre del archivo de --compare")
    ap.add_argument("--compare-title", default=None, help="título opcional de la figura de --compare")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    meta = read_meta(args.meta)
    found = find_runs(args.data_dir)
    if not found:
        sys.exit("No encontré pares robot_*/goal_* en " + args.data_dir)

    loaded, rows = {}, []
    for suf, info in found.items():
        mrow = meta.get(suf, {})
        dur = float(mrow["duration_s"]) if mrow.get("duration_s") else None
        run = load_run(info["robot"], info["goal"], dur)
        m = compute_metrics(run)
        label = mrow.get("label") or f"{info['alg']} {suf}"
        route = mrow.get("route", "")
        fig_single(run, f"{'Pure pursuit' if info['alg'] == 'pure_pursuit' else 'Stanley'} - {pretty(label)}"
                        + (f" - ruta {route}" if route else ""),
                   os.path.join(args.out, f"fig_{info['alg']}_{suf}.png"))
        loaded[suf] = (run, label)
        # columnas extra de runs.csv (alpha, beta, Kd, Ka, v_max, w_max, rep...) pasan a la tabla
        base_cols = {"suffix", "label", "route", "duration_s", "algoritmo"}
        extras = {k: v for k, v in mrow.items() if k not in base_cols and v not in (None, "")}
        rows.append({"suffix": suf, "algoritmo": info["alg"], "etiqueta": label, "ruta": route,
                     **extras, "tiempo": run["t_kind"], **m})

    # tabla resumen
    keys = list(rows[0].keys())
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(args.out, "resumen_metricas.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)
    with open(os.path.join(args.out, "resumen_metricas.md"), "w", encoding="utf-8") as f:
        f.write("| " + " | ".join(keys) + " |\n|" + "---|" * len(keys) + "\n")
        for r in rows:
            f.write("| " + " | ".join(f"{r.get(k, ''):.4g}" if isinstance(r.get(k), float) else str(r.get(k, ""))
                                      for k in keys) + " |\n")

    if args.compare:
        if len(args.compare) > len(SERIES):
            sys.exit(f"Máximo {len(SERIES)} corridas por figura de comparación")
        sel = [loaded[s] for s in args.compare]
        fig_compare([s[0] for s in sel], [s[1] for s in sel], os.path.join(args.out, args.compare_out), args.compare_title)

    # promedios por configuración y figuras de barrido (sólo si runs.csv trae los parámetros)
    if any(r.get("alpha") or r.get("Ka") for r in rows):
        means = group_means(rows)
        mkeys = ["algoritmo", "ruta", *PARAM_KEYS, "v_max", "w_max", "n_rep"]
        mkeys += [k for m in MEAN_METRICS for k in (m, m + "_sd") if any(k in g for g in means)]
        with open(os.path.join(args.out, "resumen_promedios.csv"), "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=mkeys + ["sufijos"], extrasaction="ignore")
            wr.writeheader()
            wr.writerows(means)
        for alg in ALG_NAME:
            if any(g["algoritmo"] == alg for g in means):
                fig_sweeps(means, alg, os.path.join(args.out, f"fig_barrido_{alg}.png"))
        fig_algoritmos(means, os.path.join(args.out, "fig_algoritmos.png"),
                       os.path.join(args.out, "comparacion_algoritmos.md"))

    print(f"{len(rows)} corrida(s) procesada(s). Salida en: {args.out}/")
    for r in rows:
        print(f"  {r['suffix']} {r['algoritmo']:<13} CTE rmse={r['cte_rmse_m']:.3f} m  max={r['cte_max_m']:.3f} m  "
              f"w_rms={r['w_rms_rads']:.3f}  cruces_w={r['cruces_w']}")


if __name__ == "__main__":
    main()
