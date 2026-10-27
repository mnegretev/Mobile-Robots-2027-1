#!/usr/bin/env python3
"""
analyze_t10.py - Tarea 10 (extraccion de caracteristicas geometricas), Robots Moviles FI-UNAM 2027-1
Omar Rosario Carrasco.

Analiza un lote de t10_runner.py y genera tablas, figuras y capturas para el reporte.

Uso:  python3 analyze_t10.py ../datos/<lote> --out ../resultados [--warmup 5] [--md CARPETA]

Por corrida se descartan los primeros --warmup cuadros (arranque del nodo) y, con el resto, se
calculan media, desviacion estandar, mediana, p10, p90, p95, minimo y maximo del tiempo de deteccion.
El sistema operativo no es de tiempo real: a veces llegan bloques de cuadros varias veces mas lentos.
Una corrida con p90/p10 > 2 se marca como perturbada; para comparar configuraciones se usa el p10
(el costo del algoritmo cuando el sistema no lo interrumpe), que esos bloques no mueven.

Salida (en --out):
  resumen_corridas.csv        estadisticas por corrida
  fig_canny_bordes.png        % de pixeles de borde vs umbrales y vs ancho del filtro
  fig_hough_parametros.png    tiempo y lineas vs umbral, angulos del acumulador y rho
  fig_houghP_parametros.png   tiempo y segmentos vs min_len, max_gap y umbral
  fig_tiempo_vs_votos.png     tiempo (p10) vs votos = pixeles de borde x angulos, con ajuste lineal
  fig_series_tiempo.png       tiempo cuadro a cuadro de corridas estables y perturbadas
  fig_distribucion.png        tiempos por cuadro de las corridas base, repeticiones, filtro y PDF
  mosaico_canny.png, mosaico_hough.png, mosaico_houghP.png             8 configuraciones, 4 columnas
  fig_capturas_canny.png, fig_capturas_hough.png, fig_capturas_houghP.png   6 configuraciones, 2 columnas
  capturas/<id>.png           original | resultado de cada corrida con sus parametros
  tiempo_vs_votos.csv, series_tiempo.csv   datos de las dos figuras de tiempo
  hallazgos.json              numeros clave para la discusion
Con --md CARPETA tambien escribe tablas.md (tablas de parametros y tiempos) en esa carpeta, fuera
del repositorio; sin --md no se escribe.
Requiere NumPy y Matplotlib.
"""
import argparse, csv, json, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.ticker

# Paleta (slots 1 y 2 validados para daltonismo) y tinta del grafico
HOUGH, HOUGHP = "#2a78d6", "#eb6834"
SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
COLOR = {"canny": HOUGH, "hough": HOUGH, "houghP": HOUGHP}
GRUPO = {"practica": "PDF", "base": "base", "umbrales": "umbrales", "filtro": "filtro", "umbral": "umbral",
         "theta": "θ", "rho": "ρ", "canny": "Canny", "reloj": "reloj sim.", "repeticion": "repetición",
         "min_len": "min_len", "max_gap": "max_gap"}
PERTURB = 2.0        # p90/p10 por encima de esto = corrida con bloques de cuadros lentos
BASE = {"canny": "C2", "hough": "HB", "houghP": "PB"}

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2,
    "axes.titlesize": 10.5, "axes.titleweight": "bold", "axes.titlecolor": INK,
    "axes.labelsize": 9, "xtick.color": MUTED, "ytick.color": MUTED,
    "xtick.labelcolor": INK2, "ytick.labelcolor": INK2, "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "grid.linestyle": "-",
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "legend.fontsize": 8.5, "font.size": 9, "figure.dpi": 100,
})


# ----------------------------------------------------------------------------------------------
def load_runs(lote, warmup):
    with open(os.path.join(lote, "corridas.csv")) as f:
        meta = list(csv.DictReader(f))
    runs = []
    for m in meta:
        d = os.path.join(lote, m["carpeta"])
        if m["estado"] != "ok" or not os.path.exists(os.path.join(d, "done")):
            print("  (se omite %s: %s)" % (m["id"], m["estado"]))
            continue
        with open(os.path.join(d, "params.json")) as f:
            p = json.load(f)
        with open(os.path.join(d, "frames.csv")) as f:
            all_rows = [r for r in csv.DictReader(f)]
        rows = [r for r in all_rows if int(r["frame"]) > warmup] or all_rows
        img = mpimg.imread(os.path.join(d, "original.png"))
        h, w = img.shape[:2]
        r = dict(id=m["id"], nodo=m["nodo"], grupo=m["grupo"], g=GRUPO.get(m["grupo"], m["grupo"]),
                 dir=d, p=p, w=w, h=h, n=len(rows))
        edge = np.array([float(x["edge_px"]) for x in rows])
        r["edge_px"] = float(np.median(edge))
        r["edge_pct"] = 100.0 * r["edge_px"] / (w * h)
        if m["nodo"] != "canny":
            t = np.array([float(x["t_ms"]) for x in rows])
            lines = np.array([float(x["lines"]) for x in rows])
            p10, p90, p95 = np.percentile(t, [10, 90, 95])
            r.update(t=t, t_mean=t.mean(), t_std=t.std(ddof=1) if len(t) > 1 else 0.0,
                     t_med=float(np.median(t)), t_p10=float(p10), t_p90=float(p90), t_p95=float(p95),
                     t_min=t.min(), t_max=t.max(), lines=float(np.median(lines)),
                     lines_min=lines.min(), lines_max=lines.max())
            r["perturbada"] = bool(p10 > 0 and p90 / p10 > PERTURB)
            r["frames_all"] = np.array([int(x["frame"]) for x in all_rows])
            r["t_all"] = np.array([float(x["t_ms"]) for x in all_rows])
            r["n_theta"] = int(round(math.pi / p["theta"]))
            r["n_rho"] = int(round((2 * (w + h) + 1) / p["rho"]))
            r["votos"] = r["edge_px"] * r["n_theta"]
            # acumulador de OpenCV (enteros de 4 bytes; HoughLines agrega un borde de una celda)
            pad = 2 if m["nodo"] == "hough" else 0
            r["acc_MB"] = (r["n_theta"] + pad) * (r["n_rho"] + pad) * 4 / 1e6
        runs.append(r)
    return runs


def by_id(runs):
    return {r["id"]: r for r in runs}


def deg(theta):
    return math.degrees(theta)


def fmt_theta(theta):
    d = deg(theta)
    return "%.4g (%.2g°)" % (theta, d) if d < 1 else "%.4g (%.3g°)" % (theta, d)


def blur_txt(b):
    return "%d×%d" % (b, b) if b and b > 1 else "sin filtro"


# ----------------------------------------------------------------------------------------------
def write_summary(runs, out):
    cols = ["id", "nodo", "grupo", "canny_l", "canny_u", "blur", "rho", "theta", "hough_th", "min_len",
            "max_gap", "use_sim_time", "cuadros", "edge_px", "edge_pct", "lineas_mediana", "lineas_min",
            "lineas_max", "t_media_ms", "t_desv_ms", "t_mediana_ms", "t_p95_ms", "t_min_ms", "t_max_ms",
            "n_theta", "votos", "t_p10_ms", "t_p90_ms", "perturbada", "acumulador_MB"]
    with open(os.path.join(out, "resumen_corridas.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in runs:
            p = r["p"]
            row = [r["id"], r["nodo"], r["grupo"]] + [p.get(k, "") for k in
                   ["canny_l", "canny_u", "blur", "rho", "theta", "hough_th", "min_len", "max_gap", "use_sim_time"]]
            row += [r["n"], "%.0f" % r["edge_px"], "%.3f" % r["edge_pct"]]
            if r["nodo"] == "canny":
                row += [""] * 15
            else:
                row += ["%.0f" % r["lines"], "%.0f" % r["lines_min"], "%.0f" % r["lines_max"]]
                row += ["%.4f" % r[k] for k in ["t_mean", "t_std", "t_med", "t_p95", "t_min", "t_max"]]
                row += [r["n_theta"], "%.0f" % r["votos"], "%.4f" % r["t_p10"], "%.4f" % r["t_p90"],
                        int(r["perturbada"]), "%.3f" % r["acc_MB"]]
            w.writerow(row)


def write_tables(runs, out, warmup):
    L = []
    L.append("# Tarea 10 — tablas de resultados\n")
    L.append("Tiempos: solo la detección de líneas (`HoughLines` / `HoughLinesP`), medidos con "
             "`get_clock().now()`; media ± desviación estándar, mediana y percentiles sobre los cuadros de "
             "cada corrida (se descartan los primeros %d). El p10 es el tiempo que no superan el 10 %% de "
             "los cuadros más rápidos. Líneas/segmentos: mediana por cuadro.\n" % warmup)
    c = [r for r in runs if r["nodo"] == "canny"]
    if c:
        L.append("## Detección de bordes (Canny)\n")
        L.append("| Corrida | Grupo | Umbral inferior | Umbral superior | Filtro gaussiano | Píxeles de borde |")
        L.append("|---|---|---:|---:|---|---:|")
        for r in c:
            p = r["p"]
            L.append("| %s | %s | %d | %d | %s | %.2f %% |" % (r["id"], r["g"], p["canny_l"], p["canny_u"],
                                                          blur_txt(p["blur"]), r["edge_pct"]))
        L.append("")
    for nodo, titulo, unidad in [("hough", "Transformada Hough", "Líneas"),
                                 ("houghP", "Transformada Hough probabilística", "Segmentos")]:
        rr = [r for r in runs if r["nodo"] == nodo]
        if not rr:
            continue
        extra_h = " min_len | max_gap |" if nodo == "houghP" else ""
        extra_a = "---:|---:|" if nodo == "houghP" else ""
        L.append("## %s: parámetros\n" % titulo)
        L.append("| Corrida | Grupo | Canny (inf/sup) | Filtro | ρ (px) | θ (rad) | Umbral |%s %s |" % (extra_h, unidad))
        L.append("|---|---|---|---|---:|---|---:|%s---:|" % extra_a)
        for r in rr:
            p = r["p"]
            ex = " %d | %d |" % (p["min_len"], p["max_gap"]) if nodo == "houghP" else ""
            L.append("| %s | %s | %d/%d | %s | %d | %s | %d |%s %.0f |" % (
                r["id"], r["g"], p["canny_l"], p["canny_u"], blur_txt(p["blur"]), p["rho"],
                fmt_theta(p["theta"]), p["hough_th"], ex, r["lines"]))
        L.append("")
        L.append("## %s: tiempos de ejecución\n" % titulo)
        L.append("| Corrida | Grupo | Cuadros | Píxeles de borde | t medio ± σ (ms) | Mediana (ms) | p10 (ms) | p95 (ms) | Mín–máx (ms) |")
        L.append("|---|---|---:|---:|---:|---:|---:|---:|---|")
        for r in rr:
            note = " ¹" if r["p"].get("use_sim_time") else (" †" if r["perturbada"] else "")
            L.append("| %s%s | %s | %d | %.0f | %.3f ± %.3f | %.3f | %.3f | %.3f | %.3f–%.3f |" % (
                r["id"], note, r["g"], r["n"], r["edge_px"], r["t_mean"], r["t_std"], r["t_med"],
                r["t_p10"], r["t_p95"], r["t_min"], r["t_max"]))
        if any(r["p"].get("use_sim_time") for r in rr):
            L.append("\n¹ Con `use_sim_time:=true` el reloj del nodo es el de la simulación (`/clock`).")
        if any(r["perturbada"] for r in rr):
            L.append("\n† Corrida perturbada (p90/p10 > %g): tuvo bloques de cuadros mucho más lentos por la carga "
                     "del sistema. Su media no representa al algoritmo; para comparar se usa el p10." % PERTURB)
        L.append("")
    with open(os.path.join(out, "tablas.md"), "w") as f:
        f.write("\n".join(L))


# ----------------------------------------------------------------------------------------------
def style_point_line(ax, x, y, err=None, color=HOUGH, labels=None):
    if err is not None:
        ax.errorbar(x, y, yerr=err, fmt="none", ecolor=color, elinewidth=1.0, capsize=2.5, alpha=0.8, zorder=2)
    ax.plot(x, y, "-", color=color, lw=1.6, solid_capstyle="round", zorder=3)
    ax.plot(x, y, "o", color=color, ms=6.5, mec=SURFACE, mew=1.5, zorder=4)
    if labels:
        for xi, yi, txt in labels:
            ax.annotate(txt, (xi, yi), textcoords="offset points", xytext=(5, 6), fontsize=8, color=INK2)


def series(B, ids, key_x, key_y, key_e=None):
    rr = [B[i] for i in ids if i in B]
    rr.sort(key=key_x)
    x = [key_x(r) for r in rr]
    y = [key_y(r) for r in rr]
    e = [key_e(r) for r in rr] if key_e else None
    return rr, x, y, e


def fig_canny(B, out):
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.3))
    ids = [i for i in ["C0", "C1", "C2", "C3", "C4"] if i in B]
    if ids:
        x = list(range(len(ids)))
        y = [B[i]["edge_pct"] for i in ids]
        style_point_line(axs[0], x, y, labels=[(x[0], y[0], "%.1f %%" % y[0]), (x[-1], y[-1], "%.1f %%" % y[-1])])
        axs[0].set_xticks(x, ["%d/%d" % (B[i]["p"]["canny_l"], B[i]["p"]["canny_u"]) for i in ids])
        axs[0].set_xlabel("umbrales inferior/superior (filtro %s)" % blur_txt(B[ids[0]]["p"]["blur"]))
    axs[0].set_title("Umbrales de histéresis")
    ids = [i for i in ["C5", "C6", "C2", "C7"] if i in B]
    if ids:
        ids.sort(key=lambda i: B[i]["p"]["blur"])
        x = [B[i]["p"]["blur"] for i in ids]
        y = [B[i]["edge_pct"] for i in ids]
        style_point_line(axs[1], x, y, labels=[(x[0], y[0], "%.1f %%" % y[0]), (x[-1], y[-1], "%.1f %%" % y[-1])])
        axs[1].set_xticks(x, [blur_txt(b) if b > 1 else "sin" for b in x])
        axs[1].set_xlabel("kernel del filtro gaussiano (umbrales %d/%d)" % (B[ids[0]]["p"]["canny_l"], B[ids[0]]["p"]["canny_u"]))
    axs[1].set_title("Ancho del filtro gaussiano")
    for ax in axs:
        ax.set_ylabel("píxeles de borde (% de la imagen)")
        ax.set_ylim(bottom=0)
    fig.suptitle("Canny: proporción de píxeles marcados como borde", x=0.01, ha="left",
                 fontsize=11.5, fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_canny_bordes.png"), dpi=200)
    plt.close(fig)


def fig_params(B, out, nodo, columns, fname, title, unidad):
    color = COLOR[nodo]
    fig, axs = plt.subplots(2, len(columns), figsize=(3.4 * len(columns), 5.6), sharex="col")
    for j, (ids, xfun, xlabel, xticks) in enumerate(columns):
        rr, x, y, e = series(B, ids, xfun, lambda r: r["t_mean"], lambda r: r["t_std"])
        if not rr:
            continue
        base = [(xi, yi, "base") for r, xi, yi in zip(rr, x, y) if r["grupo"] == "base"]
        style_point_line(axs[0, j], x, y, e, color, labels=base)
        rr, x, y, _ = series(B, ids, xfun, lambda r: r["lines"])
        style_point_line(axs[1, j], x, y, None, color)
        if max(y) / max(min(y), 1) > 40:
            axs[1, j].set_yscale("log")
            axs[1, j].yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: "%g" % v))
            axs[1, j].yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        axs[1, j].set_xlabel(xlabel)
        if xticks:
            axs[1, j].set_xticks(x, xticks(rr))
        axs[0, j].set_ylim(bottom=0)
    axs[0, 0].set_ylabel("tiempo de detección (ms)\nmedia ± σ")
    axs[1, 0].set_ylabel(unidad)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11.5, fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, fname), dpi=200)
    plt.close(fig)


def linfit(x, y):
    A = np.vstack([np.ones_like(x), x]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return coef, 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def fig_votes(runs, out, H):
    """Ambas transformadas votan, por cada pixel de borde, en todos los angulos del acumulador:
    el costo deberia crecer con votos V = pixeles de borde x (pi/theta). Se grafica el p10 (el costo
    sin interrupciones del sistema); las corridas perturbadas van huecas. Ademas se guarda el ajuste
    con la media, para comparar."""
    timed = [r for r in runs if r["nodo"] != "canny" and not r["p"].get("use_sim_time")]
    with open(os.path.join(out, "tiempo_vs_votos.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "nodo", "grupo", "votos_millones", "t_p10_ms", "t_mediana_ms", "t_media_ms",
                    "t_desv_ms", "perturbada"])
        for r in timed:
            w.writerow([r["id"], r["nodo"], r["g"], "%.4f" % (r["votos"] / 1e6), "%.4f" % r["t_p10"],
                        "%.4f" % r["t_med"], "%.4f" % r["t_mean"], "%.4f" % r["t_std"], int(r["perturbada"])])
    panels = [("hough", HOUGH, "Transformada Hough"), ("houghP", HOUGHP, "Transformada Hough probabilística")]
    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.4), sharex=True, sharey=True)
    ymax = max([r["t_p10"] for r in timed] + [1.0]) * 1.12
    xmax = max([r["votos"] / 1e6 for r in timed] + [1.0]) * 1.06
    for ax, (nodo, color, name) in zip(axs, panels):
        rr = [r for r in timed if r["nodo"] == nodo]
        ax.set_title(name, loc="left")
        if len(rr) < 3:
            continue
        x = np.array([r["votos"] for r in rr]) / 1e6
        y = np.array([r["t_p10"] for r in rr])
        coef, r2 = linfit(x, y)
        ym = np.array([r["t_mean"] for r in rr])
        coef_m, r2_m = linfit(x, ym)
        xs = np.linspace(0, xmax, 50)
        ax.plot(xs, coef[0] + coef[1] * xs, "-", color=color, lw=1.1, alpha=0.6, zorder=1,
                label="ajuste: t = %.2f + %.2f·V ms\nR² = %.2f (%d corridas)" % (coef[0], coef[1], r2, len(rr)))
        pert = np.array([r["perturbada"] for r in rr])
        ax.plot(x[~pert], y[~pert], "o", color=color, ms=7, mec=SURFACE, mew=1.4, zorder=3)
        if pert.any():
            ax.plot(x[pert], y[pert], "o", mfc=SURFACE, mec=color, ms=7, mew=1.6, zorder=3,
                    label="corrida perturbada (su p10)")
        # etiquetas: puntos aislados a la derecha; en la columna de V repetida, solo los extremos
        col = np.isclose(x, np.median(x), rtol=1e-3)
        for i, r in enumerate(rr):
            if not col[i]:
                ax.annotate(r["id"], (x[i], y[i]), textcoords="offset points", xytext=(6, -3), fontsize=8,
                            color=INK2)
        if col.sum() > 1:
            ii = np.where(col)[0]
            hi, lo = ii[np.argmax(y[ii])], ii[np.argmin(y[ii])]
            top = [rr[i]["id"] for i in ii if y[i] >= y[hi] - 0.6]
            ax.annotate(", ".join(top), (x[hi], y[hi]), textcoords="offset points", xytext=(7, 2), fontsize=8,
                        color=INK2)
            ax.annotate(rr[lo]["id"], (x[lo], y[lo]), textcoords="offset points", xytext=(7, -8), fontsize=8,
                        color=INK2)
            # la base y su repeticion, que la discusion compara
            idx = {rr[i]["id"]: i for i in ii}
            pair = [k for k in (BASE[nodo], BASE[nodo] + "2") if k in idx and k not in top and k != rr[lo]["id"]]
            if len(pair) == 2 and abs(y[idx[pair[0]]] - y[idx[pair[1]]]) < 0.8:
                i = idx[pair[0]]
                ax.annotate(", ".join(pair), (x[i], max(y[idx[k]] for k in pair)), textcoords="offset points",
                            xytext=(7, -3), fontsize=8, color=INK2)
            else:
                for k, side in zip(pair, (-1, 1)):
                    i = idx[k]
                    ax.annotate(k, (x[i], y[i]), textcoords="offset points", xytext=(7 * side, -3), fontsize=8,
                                color=INK2, ha="left" if side > 0 else "right")
            ax.annotate("%d corridas con\nV = %.1f M:\n%.1f–%.1f ms" % (col.sum(), x[hi], y[lo], y[hi]),
                        (x[hi], (y[hi] + y[lo]) / 2), textcoords="offset points", xytext=(-26, -4), fontsize=8,
                        color=MUTED, ha="right", va="center", zorder=4,
                        bbox=dict(facecolor=SURFACE, edgecolor="none", alpha=0.9, pad=1.5))
        ax.legend(loc="upper left", fontsize=8.5, handlelength=1.6)
        ax.set_xlabel("V = píxeles de borde × ángulos (π/θ), millones de votos")
        H["ajuste_" + nodo] = {"x": "votos (millones)", "y": "t medio (ms)", "a_ms": coef_m[0],
                               "b_ms_por_millon": coef_m[1], "r2": r2_m, "corridas": len(rr)}
        H["ajuste_" + nodo + "_p10"] = {"x": "votos (millones)", "y": "t p10 (ms)", "a_ms": coef[0],
                                        "b_ms_por_millon": coef[1], "r2": r2, "corridas": len(rr),
                                        "ns_por_voto": coef[1]}   # ms por millón de votos = ns por voto
    axs[0].set_ylabel("tiempo de detección, p10 (ms)")
    axs[0].set_xlim(0, xmax)
    axs[0].set_ylim(0, ymax)
    fig.suptitle("El tiempo crece con el número de votos V", x=0.01, ha="left", fontsize=11.5,
                 fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_tiempo_vs_votos.png"), dpi=200)
    plt.close(fig)


def fig_series(B, out, warmup):
    """Tiempo cuadro a cuadro: el sistema operativo no es de tiempo real y a veces llegan bloques de
    cuadros varias veces mas lentos. Muestra dos corridas estables y cuatro perturbadas."""
    ids = [i for i in ["HB", "HB2", "HA1", "H0", "HR2", "PB"] if i in B]
    if not ids:
        return
    with open(os.path.join(out, "series_tiempo.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cuadro"] + ids)
        n = max(len(B[i]["t_all"]) for i in ids)
        for k in range(n):
            w.writerow([k + 1] + ["%.4f" % B[i]["t_all"][k] if k < len(B[i]["t_all"]) else "" for i in ids])
    cols = 3
    rows = int(math.ceil(len(ids) / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(11, 3.0 * rows), sharex=True)
    axs = np.atleast_2d(axs)
    for k in range(rows * cols):
        ax = axs[k // cols, k % cols]
        if k >= len(ids):
            ax.axis("off")
            continue
        r = B[ids[k]]
        c = COLOR[r["nodo"]]
        ax.axvspan(0.5, warmup + 0.5, color=GRID, alpha=0.6, lw=0, zorder=0)
        ax.plot(r["frames_all"], r["t_all"], "-", color=c, lw=1.3, zorder=3)
        ax.axhline(r["t_p10"], color=INK2, lw=0.9, ls=(0, (4, 3)), zorder=2)
        estado = "perturbada" if r["perturbada"] else "estable"
        if r["grupo"] == "practica":
            que = "PDF"
        elif r["id"] == BASE[r["nodo"]]:
            que = "base de " + ("Hough" if r["nodo"] == "hough" else "Hough P")
        elif r["grupo"] == "repeticion":
            que = "repetición de " + BASE[r["nodo"]]
        else:
            que = ", ".join(param_txt(k, r["p"]) for k in changed(r, B)) or r["g"]
        ax.set_title("%s · %s · %s" % (r["id"], que, estado), loc="left", fontsize=10)
        ax.text(0.985, 0.96, "media %.1f · mediana %.1f · p10 %.1f ms" % (r["t_mean"], r["t_med"], r["t_p10"]),
                transform=ax.transAxes,
                fontsize=8, color=INK2, va="top", ha="right", zorder=5,
                bbox=dict(facecolor=SURFACE, edgecolor="none", alpha=0.85, pad=1.5))
        ax.set_ylim(0, max(r["t_all"].max() * 1.25, 1.0))
        ax.set_xlim(0.5, len(r["t_all"]) + 0.5)
        if k % cols == 0:
            ax.set_ylabel("tiempo (ms)")
        if k // cols == rows - 1:
            ax.set_xlabel("cuadro")
    fig.suptitle("Tiempo de detección cuadro a cuadro (línea discontinua: p10; gris: cuadros de arranque "
                 "descartados)", x=0.01,
                 ha="left", fontsize=11.5, fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_series_tiempo.png"), dpi=200)
    plt.close(fig)


def fig_distribution(B, out):
    ids = [i for i in ["H0", "HB", "HB2", "HF", "P0", "PB", "PB2", "PF"] if i in B]
    if not ids:
        return
    fig, ax = plt.subplots(figsize=(9, 3.8))
    data = [B[i]["t"] for i in ids]
    bp = ax.boxplot(data, widths=0.45, patch_artist=True, showfliers=True,
                    medianprops=dict(color=INK, lw=1.4), whiskerprops=dict(lw=1.0),
                    capprops=dict(lw=1.0), flierprops=dict(marker="o", ms=3.5, mew=0))
    for k, i in enumerate(ids):
        c = COLOR[B[i]["nodo"]]
        bp["boxes"][k].set(facecolor=c + "26", edgecolor=c, lw=1.2)
        for part in ("whiskers", "caps"):
            for line in bp[part][2 * k:2 * k + 2]:
                line.set_color(c)
        bp["fliers"][k].set(markerfacecolor=c, alpha=0.7)
    ax.set_xticks(range(1, len(ids) + 1), ["%s\n%s" % (i, B[i]["g"]) for i in ids])
    ax.set_ylabel("tiempo por cuadro (ms)")
    ax.set_ylim(bottom=0)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor=HOUGH + "26", edgecolor=HOUGH, label="Hough"),
                       Patch(facecolor=HOUGHP + "26", edgecolor=HOUGHP, label="Hough probabilística")],
              loc="upper right")
    n = min(len(d) for d in data)
    fig.suptitle("Variación del tiempo entre cuadros con la misma imagen (≥ %d cuadros por corrida)" % n,
                 x=0.01, ha="left", fontsize=11.5, fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_distribucion.png"), dpi=200)
    plt.close(fig)


# ----------------------------------------------------------------------------------------------
def caption(r):
    p = r["p"]
    if r["nodo"] == "canny":
        filtro = ("filtro " + blur_txt(p["blur"])) if p["blur"] > 1 else "sin filtro"
        return "%s · canny %d/%d · %s · bordes %.2f %%" % (r["id"], p["canny_l"], p["canny_u"], filtro, r["edge_pct"])
    txt = "%s · canny %d/%d · ρ=%d · θ=%s · umbral=%d" % (r["id"], p["canny_l"], p["canny_u"], p["rho"],
                                                       fmt_theta(p["theta"]), p["hough_th"])
    if r["nodo"] == "houghP":
        txt += " · min_len=%d · max_gap=%d" % (p["min_len"], p["max_gap"])
    if p.get("blur", 0) > 1:
        txt += " · filtro %s" % blur_txt(p["blur"])
    txt += "\nt = %.2f ± %.2f ms · %s = %.0f" % (r["t_mean"], r["t_std"],
                                                 "líneas" if r["nodo"] == "hough" else "segmentos", r["lines"])
    return txt


KEYS = {"canny": ["canny_l", "blur"], "hough": ["canny_l", "blur", "rho", "theta", "hough_th"],
        "houghP": ["canny_l", "blur", "rho", "theta", "hough_th", "min_len", "max_gap"]}


def param_txt(k, p):
    if k == "canny_l":
        return "Canny %d/%d" % (p["canny_l"], p["canny_u"])
    if k == "blur":
        return ("filtro %s" % blur_txt(p["blur"])) if p["blur"] > 1 else "sin filtro"
    if k == "rho":
        return "ρ = %g" % p["rho"]
    if k == "theta":
        return "θ = %.2g°" % deg(p["theta"])
    if k == "hough_th":
        return "umbral %d" % p["hough_th"]
    return "%s %d" % (k, p[k])


def changed(r, B):
    """Parametros en los que la corrida difiere de la base de su detector."""
    b = B.get(BASE[r["nodo"]])
    if b is None or b is r:
        return []
    out = []
    for k in KEYS[r["nodo"]]:
        same = ((r["p"]["canny_l"], r["p"]["canny_u"]) == (b["p"]["canny_l"], b["p"]["canny_u"]) if k == "canny_l"
                else r["p"].get(k) == b["p"].get(k))
        if not same:
            out.append(k)
    return out


def base_txt(B, nodo):
    b = B.get(BASE[nodo])
    return ", ".join(param_txt(k, b["p"]) for k in KEYS[nodo]) if b else ""


def caption_short(r, B):
    """Titulo de panel para las figuras del reporte: solo lo que cambia respecto a la base."""
    if r["id"] == BASE[r["nodo"]]:
        head = "%s · base" % r["id"]
    else:
        txt = ", ".join(param_txt(k, r["p"]) for k in changed(r, B)) or r["g"]
        head = "%s · %s%s" % (r["id"], "PDF: " if r["grupo"] == "practica" else "", txt)
    if r["nodo"] == "canny":
        tail = "bordes: %.1f %% de la imagen" % r["edge_pct"]
    else:
        tail = "p10 %.1f ms · mediana %.1f ms · %.0f %s" % (r["t_p10"], r["t_med"], r["lines"],
                                                           "líneas" if r["nodo"] == "hough" else "segmentos")
    return head + "\n" + tail


def result_image(r):
    name = "bordes.png" if r["nodo"] == "canny" else "resultado.png"
    return mpimg.imread(os.path.join(r["dir"], name))


def captures(runs, out):
    d = os.path.join(out, "capturas")
    os.makedirs(d, exist_ok=True)
    titles = {"canny": "Canny", "hough": "Hough", "houghP": "Hough P"}
    for r in runs:
        fig, axs = plt.subplots(1, 2, figsize=(12.8, 5.5))
        for ax, img, t in [(axs[0], mpimg.imread(os.path.join(r["dir"], "original.png")), "BGR Original"),
                           (axs[1], result_image(r), titles[r["nodo"]])]:
            ax.imshow(img, cmap="gray" if img.ndim == 2 else None, interpolation="nearest")
            ax.set_title(t, fontsize=10, color=INK2, fontweight="normal")
            ax.axis("off")
        fig.suptitle(caption(r), fontsize=10, color=INK, x=0.01, ha="left")
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(os.path.join(d, "%s_%s.png" % (r["id"], r["nodo"])), dpi=100)
        plt.close(fig)


def mosaic(B, ids, out, fname, title, cols=4, report=False):
    """report=False: 8 paneles chicos con todos los parametros. report=True: paneles grandes (2 columnas)
    con solo lo que cambia respecto a la base, para el reporte."""
    ids = [i for i in ids if i in B]
    if not ids:
        return
    rows = int(math.ceil(len(ids) / cols))
    pw, ph, fs, tfs, dpi = (6.4, 5.45, 12, 14, 100) if report else (4.2, 4.0, 7.5, 12, 110)
    fig, axs = plt.subplots(rows, cols, figsize=(pw * cols, ph * rows), layout="constrained")
    axs = np.atleast_2d(axs)
    for k in range(rows * cols):
        ax = axs[k // cols, k % cols]
        ax.axis("off")
        if k >= len(ids):
            continue
        r = B[ids[k]]
        img = result_image(r)
        ax.imshow(img, cmap="gray" if img.ndim == 2 else None, interpolation="antialiased")
        cap = (caption_short(r, B) if report else
               caption(r).replace(" · θ=", "\nθ=").replace(" · bordes", "\nbordes"))
        ax.set_title(cap, fontsize=fs, color=INK, fontweight="normal", loc="left")
    fig.suptitle(title, x=0.01, ha="left", fontsize=tfs, fontweight="bold", color=INK)
    fig.savefig(os.path.join(out, fname), dpi=dpi)
    plt.close(fig)


def scene_check(runs):
    """Todas las corridas deben ver la misma escena (robot quieto): diferencia media vs la primera."""
    ref = None
    diffs = {}
    for r in runs:
        img = mpimg.imread(os.path.join(r["dir"], "original.png"))[..., :3] * 255.0
        if ref is None:
            ref = img
        diffs[r["id"]] = float(np.abs(img - ref).mean())
    return diffs


# ----------------------------------------------------------------------------------------------
def findings(B, runs, H):
    def ratio(a, b):
        return B[a]["t_mean"] / B[b]["t_mean"] if a in B and b in B and B[b]["t_mean"] > 0 else None

    def rel(a, b):
        return 100.0 * (B[a]["t_mean"] - B[b]["t_mean"]) / B[b]["t_mean"] if a in B and b in B else None

    for k, v in {"t_HA1_sobre_HB (0.5° vs 1°)": ratio("HA1", "HB"), "t_HA3_sobre_HB (5.7° vs 1°)": ratio("HA3", "HB"),
                 "t_H0_sobre_HB (PDF vs base)": ratio("H0", "HB"), "t_HF_sobre_HB (filtro 5x5)": ratio("HF", "HB"),
                 "t_PA1_sobre_PB (5.7° vs 1°)": ratio("PA1", "PB"), "t_PF_sobre_PB (filtro 5x5)": ratio("PF", "PB"),
                 "t_PB_sobre_HB (HoughP vs Hough en base)": ratio("PB", "HB"),
                 "repetibilidad_HB2_vs_HB_pct": rel("HB2", "HB"),
                 "repetibilidad_PB2_vs_PB_pct": rel("PB2", "PB")}.items():
        if v is not None:
            H[k] = v
    def ratio10(a, b):
        return B[a]["t_p10"] / B[b]["t_p10"] if a in B and b in B and B[b]["t_p10"] > 0 else None

    for k, v in {"p10_HA1_sobre_HA3 (0.5° vs 5.7°)": ratio10("HA1", "HA3"),
                 "p10_HA3_sobre_HB (5.7° vs 1°)": ratio10("HA3", "HB"),
                 "p10_HF_sobre_HB (filtro 5x5)": ratio10("HF", "HB"),
                 "p10_P0_sobre_H0 (HoughP vs Hough, parametros del PDF)": ratio10("P0", "H0"),
                 "p10_PA1_sobre_HA3 (HoughP vs Hough, θ=5.7°)": ratio10("PA1", "HA3"),
                 "p10_PF_sobre_HF (HoughP vs Hough, filtro 5x5)": ratio10("PF", "HF"),
                 "p10_PB_sobre_HB (HoughP vs Hough, base)": ratio10("PB", "HB"),
                 "p10_HB2_sobre_HB (repeticion)": ratio10("HB2", "HB"),
                 "p10_PB2_sobre_PB (repeticion)": ratio10("PB2", "PB")}.items():
        if v is not None:
            H[k] = v
    timed = [r for r in runs if r["nodo"] != "canny"]
    H["perturbadas"] = [r["id"] for r in timed if r["perturbada"]]
    H["p10_ms"] = {r["id"]: round(r["t_p10"], 3) for r in timed}
    H["acumulador_MB"] = {r["id"]: round(r["acc_MB"], 3) for r in timed}
    if "HS" in B:
        H["HS_use_sim_time_t_media_ms"] = B["HS"]["t_mean"]
        H["HS_use_sim_time_cuadros_con_t_0"] = int((B["HS"]["t"] == 0).sum())
        H["HS_cuadros"] = int(B["HS"]["n"])
    H["lineas"] = {r["id"]: r["lines"] for r in runs if r["nodo"] != "canny"}
    H["bordes_pct"] = {r["id"]: round(r["edge_pct"], 3) for r in runs}
    H["cv_tiempo_pct"] = {r["id"]: 100.0 * r["t_std"] / r["t_mean"] for r in runs
                          if r["nodo"] != "canny" and r["t_mean"] > 0}


def main():
    ap = argparse.ArgumentParser(description="Analisis de la Tarea 10")
    ap.add_argument("lote", help="carpeta del lote (../datos/AAAAMMDD_HHMMSS)")
    ap.add_argument("--out", default=None, help="carpeta de resultados (default: ../resultados/<lote>)")
    ap.add_argument("--warmup", type=int, default=5, help="cuadros iniciales a descartar por corrida")
    ap.add_argument("--md", default=None, help="carpeta donde escribir tablas.md (sin esta opcion no se escribe)")
    args = ap.parse_args()
    lote = os.path.abspath(args.lote)
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados",
                                   os.path.basename(lote))
    os.makedirs(out, exist_ok=True)

    runs = load_runs(lote, args.warmup)
    B = by_id(runs)
    print("Corridas validas: %d" % len(runs))
    H = {"lote": os.path.basename(lote), "warmup": args.warmup, "corridas": len(runs)}
    try:
        with open(os.path.join(lote, "sistema.json")) as f:
            H["sistema"] = json.load(f)
    except OSError:
        pass

    write_summary(runs, out)
    if args.md:
        os.makedirs(args.md, exist_ok=True)
        write_tables(runs, args.md, args.warmup)
    fig_canny(B, out)
    th = lambda r: r["p"]["hough_th"]
    fig_params(B, out, "hough", [
        (["HT1", "HB", "HT2", "HT3"], th, "umbral de votos", None),
        (["HA1", "HB", "HA2", "HA3"], lambda r: r["n_theta"], "ángulos en el acumulador (π/θ)",
         lambda rr: ["%d\n(%.2g°)" % (r["n_theta"], deg(r["p"]["theta"])) for r in rr]),
        (["HB", "HR1", "HR2"], lambda r: r["p"]["rho"], "resolución ρ (px)", None),
    ], "fig_hough_parametros.png", "Transformada Hough: efecto de cada parámetro (base: umbral 150, θ = 1°, ρ = 1)",
       "líneas detectadas")
    fig_params(B, out, "houghP", [
        (["PL1", "PB", "PL2", "PL3"], lambda r: r["p"]["min_len"], "longitud mínima min_len (px)", None),
        (["PG1", "PB", "PG2"], lambda r: r["p"]["max_gap"], "hueco máximo max_gap (px)", None),
        (["PT1", "PB", "PT2"], th, "umbral de votos", None),
    ], "fig_houghP_parametros.png",
       "Transformada Hough probabilística: efecto de cada parámetro (base: min_len 30, max_gap 5, umbral 50)",
       "segmentos detectados")
    fig_votes(runs, out, H)
    fig_series(B, out, args.warmup)
    fig_distribution(B, out)
    mosaic(B, ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7"], out, "mosaico_canny.png", "Canny: bordes por configuración")
    mosaic(B, ["H0", "HB", "HT1", "HT3", "HA1", "HA3", "HR2", "HF"], out, "mosaico_hough.png",
           "Transformada Hough: resultados por configuración")
    mosaic(B, ["P0", "PB", "PL1", "PL3", "PG1", "PG2", "PT1", "PF"], out, "mosaico_houghP.png",
           "Transformada Hough probabilística: resultados por configuración")
    for nodo, ids, fname, name in [
            ("canny", ["C0", "C1", "C2", "C4", "C5", "C7"], "fig_capturas_canny.png", "Canny"),
            ("hough", ["H0", "HB", "HT2", "HA3", "HR2", "HF"], "fig_capturas_hough.png", "Transformada Hough"),
            ("houghP", ["P0", "PB", "PL1", "PL3", "PG2", "PF"], "fig_capturas_houghP.png",
             "Transformada Hough probabilística")]:
        mosaic(B, ids, out, fname, "%s\nbase: %s" % (name, base_txt(B, nodo)), cols=2, report=True)
    captures(runs, out)
    diffs = scene_check(runs)
    H["escena_dif_media_vs_primera"] = diffs
    moved = [k for k, v in diffs.items() if v > 6.0]
    if moved:
        print("  AVISO: la escena cambio en %s (diferencia media > 6 niveles de gris)" % ", ".join(moved))
    findings(B, runs, H)
    with open(os.path.join(out, "hallazgos.json"), "w") as f:
        json.dump(H, f, indent=1, ensure_ascii=False, default=float)
    print("Resultados en %s" % os.path.abspath(out))


if __name__ == "__main__":
    main()
