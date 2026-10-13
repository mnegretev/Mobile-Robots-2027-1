#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Figuras esquemáticas del reporte de la Práctica 2 (no dependen de los datos):
  fig_geometria.png     variables de pure pursuit (e_a) y de Stanley (delta_t, delta_theta)
  fig_arquitectura.png  banco de pruebas (nodos, servicios y archivos)

Uso:  python3 figuras_marco_teorico.py --out RESULTADOS/
"""
import argparse
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch, FancyBboxPatch, Polygon

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, GREEN = "#2a78d6", "#eb6834", "#1baf7a"
plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "text.color": INK, "font.size": 10, "mathtext.default": "regular", "savefig.dpi": 200})


def robot(ax, x, y, th, color, size=0.13):
    """Robot diferencial esquemático: triángulo orientado + flecha de orientación."""
    c, s = math.cos(th), math.sin(th)
    pts = np.array([[1.3, 0], [-0.9, 0.8], [-0.9, -0.8]]) * size
    R = np.array([[c, -s], [s, c]])
    pts = pts @ R.T + np.array([x, y])
    ax.add_patch(Polygon(pts, closed=True, facecolor=color, edgecolor=INK, lw=1.0, zorder=5))


def arrow(ax, p, q, color=INK, lw=1.4, style="-|>", ls="-", ms=11, z=4):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=ms, color=color, lw=lw, ls=ls, zorder=z,
                                 shrinkA=0, shrinkB=0))


def arc(ax, c, r, a0, a1, color=INK, lw=1.2):
    ax.add_patch(Arc(c, 2 * r, 2 * r, theta1=a0, theta2=a1, color=color, lw=lw, zorder=3))


def polar(c, r, ang):
    return (c[0] + r * math.cos(math.radians(ang)), c[1] + r * math.sin(math.radians(ang)))


def fig_geometria(out):
    fig, (a, b) = plt.subplots(1, 2, figsize=(10.4, 4.5))
    for ax in (a, b):
        ax.set_aspect("equal")
        ax.axis("off")

    # ---------- pure pursuit ----------
    t = np.linspace(0, 1, 100)
    px, py = 0.2 + 3.0 * t, 0.15 + 0.9 * t + 1.1 * t ** 2
    a.plot(px, py, ls="--", color=INK2, lw=1.4, zorder=1)
    a.text(3.28, 2.12, "ruta", color=INK2, ha="left", va="center")
    r0, th = (0.0, 0.0), math.radians(12)
    robot(a, *r0, th, BLUE, 0.18)
    # ejes de referencia
    arrow(a, (-0.35, -0.55), (0.55, -0.55), INK2, 1.0, ms=9)
    arrow(a, (-0.35, -0.55), (-0.35, 0.3), INK2, 1.0, ms=9)
    a.text(0.62, -0.55, "x", color=INK2, va="center")
    a.text(-0.35, 0.4, "y", color=INK2, ha="center")
    # orientación y línea de visión
    g = (2.55, 1.9)
    arrow(a, r0, (2.3 * math.cos(th), 2.3 * math.sin(th)), BLUE, 1.6)
    a.text(2.38, 0.30, r"$\theta_r$ (orientación)", color=BLUE, ha="left", va="center")
    a.plot([0, g[0]], [0, g[1]], color=ORANGE, lw=1.6, zorder=3)
    ang_g = math.degrees(math.atan2(g[1], g[0]))
    arc(a, r0, 1.25, 12, ang_g, ORANGE, 1.6)
    mid = polar(r0, 1.5, ang_g + 9)
    a.text(mid[0], mid[1], r"$e_a$", color=ORANGE, fontsize=13, ha="center", va="center")
    a.plot(*g, "o", color=ORANGE, ms=8, zorder=6)
    a.text(g[0] - 0.05, g[1] + 0.22, r"$(x_g,\ y_g)$", ha="center", color=ORANGE)
    a.text(0.42, -0.05, r"$(x_r,\ y_r)$", ha="left", va="top", color=BLUE)
    a.set_xlim(-0.6, 4.2)
    a.set_ylim(-0.9, 2.7)
    a.set_title("a) Pure pursuit: punto meta móvil sobre la ruta", fontsize=10.5, loc="left", color=INK)

    # ---------- Stanley ----------
    t = np.linspace(0, 1, 120)
    sx, sy = 0.1 + 3.6 * t, 1.9 - 1.0 * np.sin(1.7 * t) * 1.0 + 0.0 * t
    # ruta suave: curva ligera
    sy = 1.7 - 0.7 * t - 0.45 * np.sin(2.3 * t)
    b.plot(sx, sy, ls="--", color=INK2, lw=1.4, zorder=1)
    b.text(3.85, sy[-1] + 0.05, "ruta", color=INK2, ha="left")
    i = 52
    pi = (sx[i], sy[i])
    tang = math.atan2(sy[i + 1] - sy[i - 1], sx[i + 1] - sx[i - 1])
    b.plot(*pi, "o", color=INK, ms=6, zorder=6)
    b.text(pi[0] - 0.05, pi[1] + 0.2, r"$(x_i,\ y_i)$", ha="right", color=INK)
    # tangente
    L = 1.55
    arrow(b, pi, (pi[0] + L * math.cos(tang), pi[1] + L * math.sin(tang)), GREEN, 1.7)
    b.text(pi[0] + L * math.cos(tang) + 0.08, pi[1] + L * math.sin(tang) - 0.28, r"$\theta_i$", color=GREEN, fontsize=13)
    # robot a la derecha de la ruta (δ_t > 0)
    nrm = (math.sin(tang), -math.cos(tang))  # normal hacia la derecha de la tangente
    d = 0.8
    rp = (pi[0] + d * nrm[0] + 0.12, pi[1] + d * nrm[1] - 0.05)
    th_r = tang - math.radians(32)
    robot(b, rp[0], rp[1], th_r, BLUE, 0.17)
    arrow(b, rp, (rp[0] + 1.25 * math.cos(th_r), rp[1] + 1.25 * math.sin(th_r)), BLUE, 1.6)
    b.text(rp[0] + 1.25 * math.cos(th_r) + 0.03, rp[1] + 1.25 * math.sin(th_r) - 0.12, r"$\theta_r$", color=BLUE, fontsize=13)
    # δ_t
    b.plot([pi[0], rp[0]], [pi[1], rp[1]], color=ORANGE, lw=1.7, zorder=3)
    mx, my = (pi[0] + rp[0]) / 2, (pi[1] + rp[1]) / 2
    b.text(mx - 0.1, my - 0.02, r"$\delta_t$", color=ORANGE, fontsize=13, ha="right", va="center")
    # δ_θ: entre la tangente y la orientación del robot (arco en el robot)
    arrow(b, rp, (rp[0] + 1.0 * math.cos(tang), rp[1] + 1.0 * math.sin(tang)), GREEN, 1.1, ls="--", ms=9)
    arc(b, rp, 0.62, math.degrees(th_r), math.degrees(tang), GREEN, 1.5)
    ma = polar(rp, 0.88, math.degrees((th_r + tang) / 2))
    b.text(ma[0] + 0.04, ma[1] - 0.02, r"$\delta_\theta$", color=GREEN, fontsize=13, ha="left", va="center")
    # θ_e: entre la tangente y el vector p_i -> robot
    ang_v = math.degrees(math.atan2(rp[1] - pi[1], rp[0] - pi[0]))
    arc(b, pi, 0.5, ang_v, math.degrees(tang), INK2, 1.1)
    mt = polar(pi, 0.66, (ang_v + math.degrees(tang)) / 2)
    b.text(mt[0] + 0.02, mt[1] + 0.0, r"$\theta_e$", color=INK2, fontsize=12, ha="left", va="center")
    b.set_xlim(-0.2, 4.6)
    b.set_ylim(-0.9, 2.7)
    b.set_title("b) Stanley: punto más cercano de la ruta y sus errores", fontsize=10.5, loc="left", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_geometria.png"), bbox_inches="tight")
    plt.close(fig)


def box(ax, x, y, w, h, text, fc, ec=INK2, fs=9.5, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06", fc=fc, ec=ec, lw=1.2, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, zorder=3,
            fontweight="bold" if bold else "normal", linespacing=1.35)


def fig_arquitectura(out):
    fig, ax = plt.subplots(figsize=(10.4, 4.1))
    ax.set_xlim(0, 10.4)
    ax.set_ylim(0, 4.1)
    ax.axis("off")
    light_b, light_o, light_g, light_y = "#e3eefb", "#fdeadf", "#dff3ea", "#f4f1e6"
    box(ax, 0.1, 2.55, 1.9, 1.1, "RViz2\n(2D Goal Pose)", light_y)
    box(ax, 2.95, 1.55, 2.8, 2.1, "Nodo seguidor de rutas\npure_pursuit / stanley\n(máquina de estados:\nespera → planea → sigue)", light_b)
    box(ax, 2.95, 0.1, 2.8, 0.95, "Servicios del planificador\nplan_path (A*) · smooth_path", light_g)
    box(ax, 7.6, 2.55, 2.7, 1.1, "Gazebo Sim (house_simul)\nbase diferencial", light_o)
    box(ax, 7.6, 0.1, 2.7, 1.05, "Archivos por corrida\ngoal_*.txt (x, y)\nrobot_*.txt (x, y, θ, v, ω)", light_y)
    arrow(ax, (2.0, 3.1), (2.95, 3.1))
    ax.text(2.48, 3.28, "/goal_pose", ha="center", fontsize=8.5, color=INK2)
    arrow(ax, (4.35, 1.55), (4.35, 1.05), style="<|-|>")
    ax.text(4.47, 1.3, "ruta", ha="left", fontsize=8.5, color=INK2)
    arrow(ax, (5.75, 3.25), (7.6, 3.25))
    ax.text(6.67, 3.42, "/cmd_vel (v, ω)", ha="center", fontsize=8.5, color=INK2)
    arrow(ax, (7.6, 2.85), (5.75, 2.85), color=BLUE)
    ax.text(6.67, 2.58, "TF map→base_link", ha="center", fontsize=8.5, color=BLUE)
    ax.text(6.67, 2.32, "(x, y, θ)", ha="center", fontsize=8.5, color=BLUE)
    arrow(ax, (5.75, 1.8), (7.6, 0.9))
    ax.text(6.95, 1.62, "guarda al terminar", ha="center", fontsize=8.5, color=INK2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_arquitectura.png"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="resultados")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fig_geometria(a.out)
    fig_arquitectura(a.out)
