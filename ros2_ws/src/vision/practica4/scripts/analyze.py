#!/usr/bin/env python3
#
# PRACTICA 4 - ENTRENAMIENTO DE UNA RED NEURONAL (Robots Moviles 2027-1, FI-UNAM)
# Omar Rosario Carrasco
#
# Analisis de los experimentos: lee los CSV que genera run_experiments.sh (un renglon por
# combinacion de tasa de aprendizaje, epocas y tamano de lote) y produce las figuras y las
# tablas de resumen del reporte.
#
# Uso, desde esta carpeta (solo necesita NumPy y Matplotlib):
#   python3 analyze.py                          # lee ../datos y escribe en ../resultados
#   python3 analyze.py --dataset ../../handwritten_digits/dataset   # ademas, las figuras que usan las imagenes
#   python3 analyze.py --fonts CARPETA          # usa los .ttf de esa carpeta en las figuras
#
import argparse
import csv
import glob
import os
import random
import sys

import numpy
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))

LEARNING_RATES = [0.5, 1.0, 3.0, 10.0]
EPOCHS         = [3, 10, 50, 100]
BATCH_SIZES    = [5, 10, 30, 100]
SWEEPS = [("one-hot", "784-30-10"), ("binario", "784-30-4"),
          ("one-hot", "784-30-20-10"), ("binario", "784-30-20-4")]

# Colores: tinta y lineas de la hoja del reporte; los datos usan una rampa de un solo tono
# para lo que tiene orden (lote, tasa de aprendizaje) y dos tonos para lo que no (etiquetas).
INK, INK2, MUTED = "#1E2924", "#4B5952", "#66736C"
GRID, AXIS, SURFACE = "#E3E9DF", "#B9C4B4", "#FFFFFF"
RAMP  = ["#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]      # claro -> oscuro
BLUE, ORANGE, GRAY = "#2a78d6", "#eb6834", "#9AA39D"


def setup_style(fonts_folder):
    family = ["DejaVu Sans"]
    if fonts_folder and os.path.isdir(fonts_folder):
        names = []
        for path in sorted(glob.glob(os.path.join(fonts_folder, "*.ttf"))):
            font_manager.fontManager.addfont(path)
            names.append(font_manager.FontProperties(fname=path).get_name())
        if names:
            family = [min(names, key=len)] + family     # la familia regular; lo que le falte se toma de DejaVu Sans
    plt.rcParams.update({
        "font.family": family, "font.size": 8.5, "text.color": INK,
        "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.labelsize": 8.5,
        "axes.titlesize": 9, "axes.titleweight": "normal", "axes.titlecolor": INK,
        "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7, "grid.linestyle": "-",
        "xtick.color": AXIS, "ytick.color": AXIS, "xtick.labelcolor": MUTED, "ytick.labelcolor": MUTED,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "xtick.major.size": 3, "ytick.major.size": 0,
        "legend.frameon": False, "legend.fontsize": 8.5, "legend.title_fontsize": 8.5,
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "figure.dpi": 110, "savefig.dpi": 220, "mathtext.fontset": "dejavusans",
    })


def load_data(folder):
    #
    # Devuelve {(etiquetas, arquitectura): {(tasa, epocas, lote): renglon}} con los numeros ya convertidos
    #
    data = {}
    for path in sorted(glob.glob(os.path.join(folder, "*.csv"))):
        for row in csv.DictReader(open(path, newline="")):
            key = (row["etiquetas"], row["arquitectura"])
            values = {k: float(v) for k, v in row.items() if k not in ("etiquetas", "arquitectura")}
            values["s_por_epoca"] = values["tiempo_entrenamiento_s"]/values["epocas"]
            # Avance acumulado: tasa de aprendizaje x epocas / lote (proporcional a tasa x numero de actualizaciones)
            values["avance"] = values["tasa_aprendizaje"]*values["epocas"]/values["lote"]
            data.setdefault(key, {})[(values["tasa_aprendizaje"], int(values["epocas"]), int(values["lote"]))] = values
    return data


def column(sweep, name, combos=None):
    combos = sorted(sweep.keys()) if combos is None else combos
    return numpy.asarray([sweep[c][name] for c in combos])


def fmt(x):
    return ("%g" % x)


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path)
    plt.close(fig)
    print("  figura:", path)


# ------------------------------------------------------------------------------------------- figuras
def figure_epochs(data, labels_archs, out, name):
    #
    # Aciertos en prueba contra epocas: un panel por tasa de aprendizaje, una linea por tamano de lote
    #
    fig, axes = plt.subplots(len(labels_archs), 4, figsize=(6.0, 1.75*len(labels_archs) + 0.75), sharex=True, sharey=True)
    axes = numpy.atleast_2d(axes)
    for r, (key, row_title) in enumerate(labels_archs):
        sweep = data[key]
        for c, eta in enumerate(LEARNING_RATES):
            ax = axes[r, c]
            for b, batch in enumerate(BATCH_SIZES):
                y = [sweep[(eta, e, batch)]["aciertos_prueba_pct"] for e in EPOCHS if (eta, e, batch) in sweep]
                x = [e for e in EPOCHS if (eta, e, batch) in sweep]
                ax.plot(x, y, color=RAMP[b], linewidth=1.5, marker="o", markersize=4.6, markeredgecolor=SURFACE,
                        markeredgewidth=0.9, solid_capstyle="round", label="%d" % batch, zorder=3 + b)
            ax.set_xscale("log")
            ax.set_xticks(EPOCHS)
            ax.set_xticklabels([str(e) for e in EPOCHS])
            ax.minorticks_off()
            ax.set_xlim(2.3, 130)
            ax.set_ylim(0, 100)
            ax.set_yticks([0, 25, 50, 75, 100])
            ax.grid(axis="x", visible=False)
            if r == 0:
                ax.set_title("η = " + fmt(eta))
            if c == 0:
                ax.set_ylabel(row_title)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, title="Tamaño de lote", loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.0),
               columnspacing=1.6, handlelength=1.8)
    fig.text(0.5, 0.012, "Épocas (escala logarítmica)", ha="center", va="bottom", fontsize=8.5, color=INK2)
    fig.text(0.008, 0.47, "Aciertos en las 5000 imágenes de prueba (%)", ha="left", va="center", rotation=90, fontsize=8.5, color=INK2)
    fig.tight_layout(rect=(0.03, 0.045, 1, 0.9 if len(labels_archs) > 1 else 0.84), w_pad=0.6, h_pad=0.9)
    save(fig, out, name)


def figure_progress(data, keys_titles, out, name):
    #
    # Aciertos en prueba contra el avance acumulado (tasa x epocas / lote); el tono indica la tasa
    #
    fig, axes = plt.subplots(1, len(keys_titles), figsize=(6.0, 2.85), sharey=True)
    axes = numpy.atleast_1d(axes)
    for ax, (key, title) in zip(axes, keys_titles):
        sweep = data[key]
        for i, eta in enumerate(LEARNING_RATES):
            combos = [c for c in sorted(sweep) if c[0] == eta]
            ax.scatter(column(sweep, "avance", combos), column(sweep, "aciertos_prueba_pct", combos), s=26,
                       color=RAMP[i], edgecolor=SURFACE, linewidth=0.8, label=fmt(eta), zorder=3 + i)
        ax.set_xscale("log")
        ax.set_xlim(0.009, 330)
        ax.set_xticks([0.01, 0.1, 1, 10, 100])
        ax.set_xticklabels(["0.01", "0.1", "1", "10", "100"])
        ax.minorticks_off()
        ax.set_ylim(0, 100)
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_title(title)
    axes[0].set_ylabel("Aciertos en prueba (%)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, title="Tasa de aprendizaje η", loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.0),
               columnspacing=1.4, handletextpad=0.2)
    fig.text(0.5, 0.015, "Avance acumulado  η · épocas / lote  (escala logarítmica)", ha="center", va="bottom", fontsize=8.5, color=INK2)
    fig.tight_layout(rect=(0, 0.07, 1, 0.84), w_pad=1.2)
    save(fig, out, name)


def figure_generalization(data, key, out, name):
    #
    # Aciertos con las imagenes de entrenamiento y con las de prueba contra el avance acumulado
    #
    sweep = data[key]
    fig, ax = plt.subplots(figsize=(4.4, 2.75))
    x = column(sweep, "avance")
    ax.scatter(x, column(sweep, "aciertos_entrenamiento_pct"), s=24, color=GRAY, edgecolor=SURFACE, linewidth=0.8,
               label="Imágenes de entrenamiento", zorder=3)
    ax.scatter(x, column(sweep, "aciertos_prueba_pct"), s=24, color=BLUE, edgecolor=SURFACE, linewidth=0.8,
               label="Imágenes de prueba", zorder=4)
    ax.set_xscale("log")
    ax.set_xlim(0.009, 330)
    ax.set_xticks([0.01, 0.1, 1, 10, 100])
    ax.set_xticklabels(["0.01", "0.1", "1", "10", "100"])
    ax.minorticks_off()
    ax.set_ylim(0, 100)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_xlabel("Avance acumulado  η · épocas / lote  (escala logarítmica)")
    ax.set_ylabel("Aciertos (%)")
    ax.legend(loc="lower right", handletextpad=0.2, borderaxespad=0.6)
    fig.tight_layout()
    save(fig, out, name)


def scatter_pair(ax, x, y, title, xlabel, ylabel, above, below, lim=(0, 100)):
    ax.plot(lim, lim, color=AXIS, linewidth=0.9, zorder=2)
    ax.scatter(x, y, s=22, color=BLUE, edgecolor=SURFACE, linewidth=0.8, zorder=3)
    ax.set_xlim(*lim)
    ax.set_ylim(*lim)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.text(0.04, 0.95, above, transform=ax.transAxes, ha="left", va="top", fontsize=8, color=MUTED)
    ax.text(0.96, 0.05, below, transform=ax.transAxes, ha="right", va="bottom", fontsize=8, color=MUTED)


def figure_labels(data, key_onehot, key_binary, out, name):
    #
    # Cada punto es una combinacion de parametros: aciertos con etiquetas binarias contra one-hot
    #
    a, b = data[key_onehot], data[key_binary]
    combos = sorted(set(a) & set(b))
    fig, axes = plt.subplots(1, 2, figsize=(6.0, 3.25))
    scatter_pair(axes[0], column(a, "aciertos_prueba_pct", combos), column(b, "aciertos_prueba_pct", combos),
                 "Criterio de la práctica:  ‖t − y‖ < 0.5", "One-hot (%)", "Binario (%)",
                 "Acierta más\nel binario", "Acierta más\nel one-hot")
    scatter_pair(axes[1], column(a, "digito_prueba_pct", combos), column(b, "digito_prueba_pct", combos),
                 "Dígito decodificado", "One-hot, neurona más activa (%)", "Binario, 4 bits redondeados (%)",
                 "Acierta más\nel binario", "Acierta más\nel one-hot")
    fig.tight_layout(w_pad=1.6)
    save(fig, out, name)


def figure_architectures(data, pairs, out, name):
    fig, axes = plt.subplots(1, len(pairs), figsize=(6.0, 3.25))
    for ax, (key_a, key_b, title) in zip(numpy.atleast_1d(axes), pairs):
        a, b = data[key_a], data[key_b]
        combos = sorted(set(a) & set(b))
        scatter_pair(ax, column(a, "aciertos_prueba_pct", combos), column(b, "aciertos_prueba_pct", combos), title,
                     "Red %s (%%)" % key_a[1], "Red %s (%%)" % key_b[1],
                     "Acierta más la red\ncon dos capas ocultas", "Acierta más la red\ncon una capa oculta")
    fig.tight_layout(w_pad=1.6)
    save(fig, out, name)


def figure_time(data, out, name):
    #
    # Izquierda: tiempo de entrenamiento contra epocas. Derecha: segundos por epoca contra tamano de lote.
    # En los dos paneles, la mediana de las corridas y una barra del minimo al maximo, por arquitectura.
    #
    groups = [("Una capa oculta (784-30-10 y 784-30-4)", BLUE, [k for k in data if k[1].count("-") == 2]),
              ("Dos capas ocultas (784-30-20-10 y 784-30-20-4)", ORANGE, [k for k in data if k[1].count("-") == 3])]
    fig, axes = plt.subplots(1, 2, figsize=(6.0, 2.9))
    for g, (label, color, keys) in enumerate(groups):
        if not keys:
            continue
        rows = [v for k in keys for v in data[k].values()]
        offset = (-1, 1)[g]
        med = [numpy.median([v["tiempo_entrenamiento_s"] for v in rows if v["epocas"] == e]) for e in EPOCHS]
        lo  = [min([v["tiempo_entrenamiento_s"] for v in rows if v["epocas"] == e]) for e in EPOCHS]
        hi  = [max([v["tiempo_entrenamiento_s"] for v in rows if v["epocas"] == e]) for e in EPOCHS]
        x = numpy.asarray(EPOCHS, dtype=float) + 1.2*offset
        axes[0].vlines(x, lo, hi, color=color, linewidth=1.4, alpha=0.55, zorder=2)
        axes[0].plot(x, med, color=color, linewidth=1.5, marker="o", markersize=4.6, markeredgecolor=SURFACE,
                     markeredgewidth=0.9, label=label, zorder=3)
        axes[0].annotate("%.0f s" % med[-1], (x[-1], med[-1]), xytext=(7, -3), textcoords="offset points",
                         ha="left", fontsize=8, color=INK2)
        med = [numpy.median([v["s_por_epoca"] for v in rows if v["lote"] == b]) for b in BATCH_SIZES]
        lo  = [min([v["s_por_epoca"] for v in rows if v["lote"] == b]) for b in BATCH_SIZES]
        hi  = [max([v["s_por_epoca"] for v in rows if v["lote"] == b]) for b in BATCH_SIZES]
        x = numpy.arange(len(BATCH_SIZES)) + 0.07*offset
        axes[1].vlines(x, lo, hi, color=color, linewidth=1.4, alpha=0.55, zorder=2)
        axes[1].plot(x, med, color=color, linewidth=1.5, marker="o", markersize=4.6, markeredgecolor=SURFACE,
                     markeredgewidth=0.9, zorder=3)
    axes[0].set_xticks([0, 25, 50, 75, 100])
    axes[0].set_xlim(-4, 124)
    axes[0].set_ylim(bottom=0)
    axes[0].set_xlabel("Épocas")
    axes[0].set_ylabel("Tiempo de entrenamiento (s)")
    axes[1].set_xticks(range(len(BATCH_SIZES)))
    axes[1].set_xticklabels([str(b) for b in BATCH_SIZES])
    axes[1].set_xlim(-0.4, len(BATCH_SIZES) - 0.6)
    axes[1].set_ylim(bottom=0)
    axes[1].grid(axis="x", visible=False)
    axes[1].set_xlabel("Tamaño de lote")
    axes[1].set_ylabel("Tiempo por época (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=1, bbox_to_anchor=(0.5, 1.0), handlelength=1.8)
    fig.tight_layout(rect=(0, 0, 1, 0.86), w_pad=1.6)
    save(fig, out, name)


def figure_dataset(dataset, out, name, rows=6):
    #
    # Muestra del conjunto de datos: las primeras imagenes de cada digito (tonos invertidos: trazo oscuro)
    #
    fig, axes = plt.subplots(rows, 10, figsize=(6.0, 0.6*rows + 0.1))
    for d in range(10):
        raw = numpy.frombuffer(open(os.path.join(dataset, "data%d" % d), "rb").read(784*rows), dtype=numpy.uint8)
        for r in range(rows):
            ax = axes[r, d]
            ax.imshow(255 - raw[784*r:784*(r + 1)].reshape(28, 28), cmap="gray", vmin=0, vmax=255, interpolation="nearest")
            ax.set_xticks([])
            ax.set_yticks([])
            ax.grid(False)
            for s in ax.spines.values():
                s.set_visible(True)
                s.set_color(GRID)
                s.set_linewidth(0.6)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01, wspace=0.06, hspace=0.06)
    save(fig, out, name)


def figure_examples(dataset, fc_folder, out, name, eta=3.0, epochs=10, batch=5, seed=2027, count=6):
    #
    # Entrena una red 784-30-10 con fc.py y muestra, para algunas imagenes de prueba, las 10 salidas
    # de la red, la norma del error y si la prueba cuenta como acierto. Se eligen las primeras imagenes
    # (en un orden al azar con semilla fija) de tres casos: acierto, neurona correcta sin cumplir el
    # criterio de la norma, y neurona equivocada.
    #
    sys.path.insert(0, fc_folder)
    import contextlib, io
    import fc
    with contextlib.redirect_stdout(io.StringIO()):
        training_x, training_t, testing_x, testing_t = fc.load_dataset(dataset)
        numpy.random.seed(seed)
        random.seed(seed)
        nn = fc.FCNeuralNetwork([784, 30, 10])
        nn.train_by_SGD(training_x, training_t, epochs, batch, eta)
    order = numpy.random.RandomState(1).permutation(len(testing_x))
    cases = {"hit": [], "argmax": [], "miss": []}
    for i in order:
        y, t = nn.feedforward(testing_x[i])[-1], testing_t[i]
        error = numpy.linalg.norm(t - y)
        kind = "hit" if error < 0.5 else ("argmax" if numpy.argmax(y) == numpy.argmax(t) else "miss")
        if len(cases[kind]) < count//3:
            cases[kind].append((i, y, t, error))
        if all(len(v) == count//3 for v in cases.values()):
            break
    chosen = cases["hit"] + cases["argmax"] + cases["miss"]
    fig, axes = plt.subplots(2, len(chosen), figsize=(6.0, 2.45), gridspec_kw={"height_ratios": [1, 1.25]})
    for k, (i, y, t, error) in enumerate(chosen):
        ax = axes[0, k]
        ax.imshow(1 - testing_x[i].reshape(28, 28), cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for s in ax.spines.values():
            s.set_visible(True)
            s.set_color(GRID)
        ax.set_title("‖t − y‖ = %.2f\n%s" % (error, "acierto" if error < 0.5 else "fallo"), fontsize=8, color=INK)
        ax = axes[1, k]
        target = int(numpy.argmax(t))
        colors = [BLUE if d == target else GRAY for d in range(10)]
        ax.bar(range(10), y.ravel(), width=0.62, color=colors, zorder=3)
        ax.set_ylim(0, 1.05)
        ax.set_yticks([0, 0.5, 1])
        ax.set_yticklabels(["0", "0.5", "1"] if k == 0 else [])
        ax.set_xticks(range(10))
        ax.set_xticklabels([str(d) for d in range(10)], fontsize=6.5)
        ax.tick_params(axis="x", length=0, pad=1.5)
        ax.grid(axis="x", visible=False)
        if k == 0:
            ax.set_ylabel("Salida")
    fig.text(0.5, 0.015, "Neurona de salida (en azul, la del dígito correcto)", ha="center", va="bottom", fontsize=8.5, color=INK2)
    fig.tight_layout(rect=(0, 0.085, 1, 1), w_pad=0.5, h_pad=0.25)
    save(fig, out, name)
    hits, decoded, _ = (100.0*numpy.mean([numpy.linalg.norm(t - nn.feedforward(x)[-1]) < 0.5 for x, t in zip(testing_x, testing_t)]),
                        100.0*numpy.mean([numpy.argmax(nn.feedforward(x)[-1]) == numpy.argmax(t) for x, t in zip(testing_x, testing_t)]), 0)
    return {"eta": eta, "epocas": epochs, "lote": batch, "aciertos": hits, "digito": decoded,
            "casos": [(int(i), float(e)) for i, _, _, e in chosen]}


# ------------------------------------------------------------------------------------------- tablas
def summary(data, out):
    lines = []
    def w(text=""):
        lines.append(text)
    for key in SWEEPS:
        if key not in data:
            continue
        sweep = data[key]
        w("=== Etiquetas %s, red %s: %d combinaciones ===" % (key[0], key[1], len(sweep)))
        w("Aciertos en prueba (%), criterio ||t - y|| < 0.5. Renglones: tasa, lote. Columnas: epocas " + str(EPOCHS))
        for eta in LEARNING_RATES:
            for batch in BATCH_SIZES:
                cells = ["%5.1f" % sweep[(eta, e, batch)]["aciertos_prueba_pct"] if (eta, e, batch) in sweep else "   - " for e in EPOCHS]
                w("  eta=%4.1f lote=%3d | %s" % (eta, batch, "  ".join(cells)))
        rows = list(sweep.values())
        best = max(rows, key=lambda v: v["aciertos_prueba_pct"])
        worst = min(rows, key=lambda v: v["aciertos_prueba_pct"])
        for tag, v in (("Mejor", best), ("Peor", worst)):
            w("%s: eta=%g epocas=%d lote=%d -> %.2f %% (digito %.2f %%, entrenamiento %.2f %%) en %.1f s" % (
              tag, v["tasa_aprendizaje"], v["epocas"], v["lote"], v["aciertos_prueba_pct"], v["digito_prueba_pct"],
              v["aciertos_entrenamiento_pct"], v["tiempo_entrenamiento_s"]))
        w("Tiempo de entrenamiento por numero de epocas (mediana [min, max], s):")
        for e in EPOCHS:
            t = [v["tiempo_entrenamiento_s"] for v in rows if v["epocas"] == e]
            if t:
                w("  %3d epocas: %6.2f [%6.2f, %6.2f]" % (e, numpy.median(t), min(t), max(t)))
        w("Segundos por epoca segun el lote (mediana [min, max]):")
        for b in BATCH_SIZES:
            t = [v["s_por_epoca"] for v in rows if v["lote"] == b]
            if t:
                w("  lote %3d: %.3f [%.3f, %.3f]" % (b, numpy.median(t), min(t), max(t)))
        w("Segundos por epoca segun la tasa de aprendizaje (mediana):  " +
          "  ".join(["%g: %.3f" % (eta, numpy.median([v["s_por_epoca"] for v in rows if v["tasa_aprendizaje"] == eta]))
                     for eta in LEARNING_RATES if any(v["tasa_aprendizaje"] == eta for v in rows)]))
        d = numpy.asarray([v["aciertos_100_pct"] - v["aciertos_prueba_pct"] for v in rows])
        w("100 pruebas menos 5000 pruebas (puntos): media %+.2f, desviacion %.2f, maximo absoluto %.1f" % (d.mean(), d.std(), numpy.abs(d).max()))
        w()
    open(os.path.join(out, "resumen.txt"), "w").write("\n".join(lines) + "\n")
    print("  resumen:", os.path.join(out, "resumen.txt"))


def main():
    parser = argparse.ArgumentParser(description="Figuras y tablas de la Practica 4")
    parser.add_argument("--datos", default=os.path.join(HERE, "..", "datos"))
    parser.add_argument("--out", default=os.path.join(HERE, "..", "resultados"))
    parser.add_argument("--dataset", default=None, help="carpeta con data0 ... data9 (para las figuras con imagenes)")
    parser.add_argument("--fc", default=os.path.join(HERE, "..", "..", "handwritten_digits", "handwritten_digits"),
                        help="carpeta donde esta fc.py")
    parser.add_argument("--fonts", default=None, help="carpeta con archivos .ttf para las figuras")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    setup_style(args.fonts)
    data = load_data(args.datos)
    print("Barridos:", ", ".join(["%s %s (%d)" % (k[0], k[1], len(v)) for k, v in sorted(data.items())]))
    summary(data, args.out)
    A1, B1, A2, B2 = SWEEPS
    if A1 in data and B1 in data:
        figure_epochs(data, [(A1, "One-hot"), (B1, "Binario")], args.out, "fig_aciertos_epocas.png")
        figure_progress(data, [(A1, "Etiquetas one-hot (784-30-10)"), (B1, "Etiquetas binarias (784-30-4)")], args.out, "fig_avance.png")
        figure_generalization(data, A1, args.out, "fig_generalizacion.png")
        figure_labels(data, A1, B1, args.out, "fig_etiquetas.png")
    if A2 in data and B2 in data:
        figure_architectures(data, [(A1, A2, "Etiquetas one-hot"), (B1, B2, "Etiquetas binarias")], args.out, "fig_arquitecturas.png")
    figure_time(data, args.out, "fig_tiempo.png")
    if args.dataset:
        figure_dataset(args.dataset, args.out, "fig_dataset.png")
        info = figure_examples(args.dataset, args.fc, args.out, "fig_ejemplos.png")
        print("  ejemplos: red 784-30-10 con eta=%(eta)g, %(epocas)d epocas, lote %(lote)d -> %(aciertos).2f %% de aciertos, %(digito).2f %% de digitos" % info)


if __name__ == "__main__":
    main()
