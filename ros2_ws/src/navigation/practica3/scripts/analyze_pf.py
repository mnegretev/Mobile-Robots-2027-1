#!/usr/bin/env python3
"""
analyze_pf.py - Practica 3 (filtro de particulas), Robots Moviles FI-UNAM 2027-1
Analiza los logs del nodo `pf` generados por p3_runner.py.

Para cada iteracion del filtro toma la pose estimada (media de las particulas) y la pose real
(Gazebo) y calcula
    e_d = ||(x_est, y_est) - (x_real, y_real)||      error de posicion [m]
    e_t = |wrap(theta_est - theta_real)|             error de orientacion [rad]
    t_u = t("Estimated pose") - t("Updating")        tiempo de computo de la iteracion [ms]
Criterio de localizacion: e_d < THR_D y e_t < THR_T durante K iteraciones consecutivas; la
iteracion de localizacion k* es la primera de esa ventana.

Uso:  python3 analyze_pf.py DATOS/ --out RESULTADOS/ [--map appartment.pgm] [--batch PREFIJO]
Salidas: iteraciones.csv, corridas.csv, configuraciones.csv y figuras PNG.
"""
import argparse, csv, math, os, re
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

THR_D, THR_T, K_CONS, LAST = 0.30, 0.20, 3, 10
ORDER = ["B", "N1", "N2", "N3", "S1", "S2", "S3", "M1", "M2", "M3", "R1", "R2", "R3", "D0"]
GROUPS = [("N", "N", "N1 N2 B N3"), ("s2s", r"$\sigma_s^2$", "S1 B S2 S3"),
          ("s2m", r"$\sigma_m^2$", "M1 B M2 M3"), ("s2r", r"$\sigma_r^2$", "R1 B R2 R3")]

INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
RAMP = ["#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]       # ordinal azul (claro = valor pequeno), validado
MARKERS = ["o", "s", "^", "D"]
C_REAL, C_EST = "#2a78d6", "#eb6834"                       # par categorico validado
plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9,
                     "legend.fontsize": 8, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
                     "text.color": INK, "legend.frameon": True, "legend.framealpha": 0.92, "legend.edgecolor": GRID,
                     "figure.dpi": 150, "savefig.dpi": 200, "savefig.bbox": "tight"})


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def parse_log(path):
    rows, t_upd, est = [], None, None
    pat_t = re.compile(r"^\[\w+\] \[(\d+\.\d+)\]")
    for line in open(path, errors="replace"):
        m = pat_t.match(line)
        ts = float(m.group(1)) if m else None
        if "Updating (" in line:
            t_upd = ts
        elif "Estimated pose" in line:
            v = re.search(r"x=(\S+)\s+y=(\S+)\s+a=(\S+)", line)
            est = (tuple(map(float, v.groups())), ts)
        elif "Groundtruth pose" in line and est:
            v = re.search(r"x=(\S+)\s+y=(\S+)\s+a=(\S+)", line)
            g = tuple(map(float, v.groups()))
            (ex, ey, ea), te = est
            rows.append(dict(ex=ex, ey=ey, ea=ea, gx=g[0], gy=g[1], ga=g[2],
                             e_d=math.hypot(ex - g[0], ey - g[1]), e_t=abs(wrap(ea - g[2])),
                             t_u=(te - t_upd) * 1000 if (te and t_upd) else float("nan")))
            est = None
    return rows


def k_localized(rows):
    for i in range(len(rows) - K_CONS + 1):
        if all(r["e_d"] < THR_D and r["e_t"] < THR_T for r in rows[i:i + K_CONS]):
            return i + 1
    return None


def fmt(x, n=2):
    return "" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{n}f}"


def load_map(pgm, res=0.05, origin=(-14.406, -10.661)):
    raw = open(pgm, "rb").read()
    m = re.match(rb"P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s", raw)
    w, h = int(m.group(1)), int(m.group(2))
    img = np.frombuffer(raw[m.end():], dtype=np.uint8).reshape(h, w)
    ext = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    return img, ext


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("datos")
    ap.add_argument("--out", default="resultados")
    ap.add_argument("--map", default="")
    ap.add_argument("--batch", default="", help="usar solo corridas cuyo run_id empiece con esto")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    runs = [r for r in csv.DictReader(open(os.path.join(args.datos, "runs.csv")))
            if r["run_id"].startswith(args.batch) and r["status"] == "ok"]
    data = {}
    for r in runs:
        data[r["run_id"]] = parse_log(os.path.join(args.datos, r["log"]))

    # ---------------------------------------------------------------- CSV por iteracion
    with open(os.path.join(args.out, "iteraciones.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run_id", "config", "rep", "k", "x_est", "y_est", "th_est", "x_real", "y_real", "th_real",
                    "e_d_m", "e_th_rad", "t_iter_ms"])
        for r in runs:
            for k, it in enumerate(data[r["run_id"]], 1):
                w.writerow([r["run_id"], r["config"], r["rep"], k, *(f"{it[c]:.4f}" for c in
                            ("ex", "ey", "ea", "gx", "gy", "ga", "e_d", "e_t")), fmt(it["t_u"], 1)])

    # ---------------------------------------------------------------- resumen por corrida
    per_run = []
    for r in runs:
        rows = data[r["run_id"]]
        k = k_localized(rows)
        last = rows[-LAST:]
        post = rows[k - 1:] if k else []
        lon = [(x["ex"] - x["gx"]) * math.cos(x["ga"]) + (x["ey"] - x["gy"]) * math.sin(x["ga"]) for x in post]
        lat = [-(x["ex"] - x["gx"]) * math.sin(x["ga"]) + (x["ey"] - x["gy"]) * math.cos(x["ga"]) for x in post]
        per_run.append(dict(run_id=r["run_id"], config=r["config"], group=r["group"], N=int(r["N"]),
                            s2s=float(r["s2s"]), s2m=float(r["s2m"]), s2r=float(r["s2r"]), rep=int(r["rep"]),
                            n_iter=len(rows), k_loc=k,
                            e_d_fin=float(np.mean([x["e_d"] for x in last])) if last else float("nan"),
                            e_t_fin=float(np.mean([x["e_t"] for x in last])) if last else float("nan"),
                            t_u=float(np.nanmean([x["t_u"] for x in rows])) if rows else float("nan"),
                            lon=float(np.mean(lon)) if lon else float("nan"),
                            lat=float(np.mean(lat)) if lat else float("nan")))
    with open(os.path.join(args.out, "corridas.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run_id", "config", "N", "s2s", "s2m", "s2r", "rep", "iteraciones", "k_localizado",
                    "e_d_final_m", "e_th_final_rad", "t_iter_ms", "sesgo_long_m", "sesgo_lat_m"])
        for p in per_run:
            w.writerow([p["run_id"], p["config"], p["N"], p["s2s"], p["s2m"], p["s2r"], p["rep"], p["n_iter"],
                        p["k_loc"] or "", fmt(p["e_d_fin"], 3), fmt(p["e_t_fin"], 3), fmt(p["t_u"], 1),
                        fmt(p["lon"], 3), fmt(p["lat"], 3)])

    # ---------------------------------------------------------------- resumen por configuracion
    by_cfg = defaultdict(list)
    for p in per_run:
        by_cfg[p["config"]].append(p)
    cfg_rows = []
    for c in [c for c in ORDER if c in by_cfg]:
        ps = sorted(by_cfg[c], key=lambda p: p["rep"])
        ks = [p["k_loc"] for p in ps]
        ok = [k for k in ks if k]
        loc = [p for p in ps if p["k_loc"]]
        cfg_rows.append(dict(
            config=c, group=ps[0]["group"], N=ps[0]["N"], s2s=ps[0]["s2s"], s2m=ps[0]["s2m"], s2r=ps[0]["s2r"],
            runs=len(ps), loc=len(ok), k_list=" / ".join(str(k) if k else "–" for k in ks),
            k_mean=float(np.mean(ok)) if ok else float("nan"),
            e_d=float(np.mean([p["e_d_fin"] for p in loc])) if loc else float(np.mean([p["e_d_fin"] for p in ps])),
            e_t=float(np.mean([p["e_t_fin"] for p in loc])) if loc else float(np.mean([p["e_t_fin"] for p in ps])),
            e_d_all=float(np.mean([p["e_d_fin"] for p in ps])),
            t_u=float(np.mean([p["t_u"] for p in ps]))))
    with open(os.path.join(args.out, "configuraciones.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["config", "grupo", "N", "s2s", "s2m", "s2r", "corridas", "localizadas", "k_por_corrida",
                    "k_medio", "e_d_final_m", "e_th_final_rad", "t_iter_ms"])
        for c in cfg_rows:
            w.writerow([c["config"], c["group"], c["N"], c["s2s"], c["s2m"], c["s2r"], c["runs"], c["loc"], c["k_list"],
                        fmt(c["k_mean"], 1), fmt(c["e_d"], 3), fmt(c["e_t"], 3), fmt(c["t_u"], 1)])

    # ---------------------------------------------------------------- figura: convergencia por parametro
    lab = {"N": lambda p: f"N = {p['N']}", "s2s": lambda p: f"$\\sigma_s^2$ = {p['s2s']:g}",
           "s2m": lambda p: f"$\\sigma_m^2$ = {p['s2m']:g}", "s2r": lambda p: f"$\\sigma_r^2$ = {p['s2r']:g}"}
    fig, axs = plt.subplots(2, 2, figsize=(7.2, 6.3), sharex=True, sharey=True)
    for ax, (g, gname, ids) in zip(axs.flat, GROUPS):
        ids = [i for i in ids.split() if i in by_cfg]
        ids.sort(key=lambda i: by_cfg[i][0][g])
        for j, cid in enumerate(ids):
            col, mk = RAMP[j], MARKERS[j]
            ps = by_cfg[cid]
            curves = [np.array([x["e_d"] for x in data[p["run_id"]]]) for p in ps]
            n = min(len(cv) for cv in curves)
            for cv in curves:
                ax.plot(np.arange(1, len(cv) + 1), cv, color=col, lw=0.6, alpha=0.35)
            mean = np.exp(np.mean([np.log(cv[:n]) for cv in curves], axis=0))     # media geometrica (eje log)
            ax.plot(np.arange(1, n + 1), mean, color=col, lw=1.5, marker=mk, ms=4, markevery=(j, 5),
                    label=lab[g](ps[0]) + (" (base)" if cid == "B" else ""))
        ax.axhline(THR_D, color=INK2, ls="--", lw=0.8)
        ax.set_yscale("log")
        # leyenda encima del panel, fuera del area de datos, con el nombre del barrido como titulo
        ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False, fontsize=8,
                  title=f"Variación de {gname}", title_fontsize=10, columnspacing=1.4, handlelength=2.4,
                  borderaxespad=0.2)
    for ax in axs[1]:
        ax.set_xlabel("Iteración del filtro k")
    for ax in axs[:, 0]:
        ax.set_ylabel("Error de posición $e_d$ [m]")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "fig_convergencia.png"))
    plt.close(fig)

    # ---------------------------------------------------------------- figura: estimada vs real (base y D0)
    def rep_run(cid):
        ps = sorted(by_cfg.get(cid, []), key=lambda p: (p["k_loc"] is None, p["rep"]))
        return ps[0] if ps else None

    sel = [(c, rep_run(c)) for c in ("B", "D0") if rep_run(c)]
    if sel:
        fig, axs = plt.subplots(3, len(sel), figsize=(3.6 * len(sel), 5.8), sharex="col", squeeze=False)
        for j, (cid, p) in enumerate(sel):
            rows = data[p["run_id"]]
            k = np.arange(1, len(rows) + 1)
            # angulos "desenvueltos" para que el cruce por +-pi no dibuje saltos falsos
            th_real = np.unwrap([r["ga"] for r in rows])
            th_est = th_real + np.array([wrap(r["ea"] - r["ga"]) for r in rows])
            series = {"gx": [r["gx"] for r in rows], "ex": [r["ex"] for r in rows],
                      "gy": [r["gy"] for r in rows], "ey": [r["ey"] for r in rows], "ga": th_real, "ea": th_est}
            for i, (e, g, yl) in enumerate([("ex", "gx", "x [m]"), ("ey", "gy", "y [m]"), ("ea", "ga", r"$\theta$ [rad]")]):
                ax = axs[i, j]
                ax.plot(k, series[g], color=C_REAL, lw=1.5, label="real (Gazebo)")
                ax.plot(k, series[e], color=C_EST, lw=1.0, marker="o", ms=3, label="estimada")
                if p["k_loc"]:
                    ax.axvline(p["k_loc"], color=INK2, ls=":", lw=1.0)
                ax.set_ylabel(yl)
                if i == 0:
                    t = "Base (N=1000, 0.1, 0.01, 0.01)" if cid == "B" else "Comando de la práctica (N=100, 1, 1, 1)"
                    ax.set_title(t)
            axs[2, j].set_xlabel("Iteración del filtro k")
        # una sola leyenda para toda la figura, arriba, para no tapar datos
        from matplotlib.lines import Line2D
        handles = [Line2D([], [], color=C_REAL, lw=1.5), Line2D([], [], color=C_EST, lw=1.0, marker="o", ms=3),
                   Line2D([], [], color=INK2, ls=":", lw=1.0)]
        fig.legend(handles, ["real (Gazebo)", "estimada", "iteración de localización k*"], loc="upper center",
                   ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.0), fontsize=8.5)
        fig.tight_layout(rect=(0, 0, 1, 0.955))
        fig.savefig(os.path.join(args.out, "fig_estimada_vs_real.png"))
        plt.close(fig)

    # ---------------------------------------------------------------- figura: trayectoria sobre el mapa
    if args.map and sel:
        img, ext = load_map(args.map)
        fig, axs = plt.subplots(1, len(sel), figsize=(3.6 * len(sel), 3.4), squeeze=False)
        for ax, (cid, p) in zip(axs[0], sel):
            rows = data[p["run_id"]]
            ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=255)
            gx = [r["gx"] for r in rows]; gy = [r["gy"] for r in rows]
            oranges = matplotlib.colors.LinearSegmentedColormap.from_list("or", ["#f6c3a6", "#eb6834", "#8a2e0b"])
            sc = ax.scatter([r["ex"] for r in rows], [r["ey"] for r in rows], c=np.arange(1, len(rows) + 1),
                            cmap=oranges, s=10, zorder=3, edgecolors="white", linewidths=0.3, label="estimada")
            ax.plot(gx, gy, color=C_REAL, lw=2.0, zorder=4, label="real")
            ax.set_xlim(-6, 7); ax.set_ylim(-4.5, 5.0); ax.set_aspect("equal")
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            ax.set_title("Base" if cid == "B" else "Comando de la práctica")
            ax.legend(loc="lower left", fontsize=7)
            cb = fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02); cb.set_label("iteración k")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "fig_trayectoria_mapa.png"))
        plt.close(fig)

    # ---------------------------------------------------------------- resumen en consola
    lon_all = [p["lon"] for p in per_run if not math.isnan(p["lon"])]
    lat_all = [p["lat"] for p in per_run if not math.isnan(p["lat"])]
    if lon_all:
        print(f"Sesgo tras localizarse (media de corridas localizadas): longitudinal {np.mean(lon_all):+.3f} m, "
              f"lateral {np.mean(lat_all):+.3f} m")
    print(f"{len(runs)} corridas, criterio: e_d < {THR_D} m y e_t < {THR_T} rad durante {K_CONS} iteraciones")
    for c in cfg_rows:
        print(f"  {c['config']:>3}  N={c['N']:<5} s2s={c['s2s']:<6g} s2m={c['s2m']:<6g} s2r={c['s2r']:<6g} "
              f"loc {c['loc']}/{c['runs']}  k*={c['k_list']:<9} e_d={c['e_d']:.3f} m  e_t={c['e_t']:.3f} rad  t={c['t_u']:.1f} ms")


if __name__ == "__main__":
    main()
