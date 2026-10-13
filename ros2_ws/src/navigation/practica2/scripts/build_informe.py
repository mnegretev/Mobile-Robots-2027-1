#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Construye el reporte de la Práctica 2 en formato .odt (editable en LibreOffice).

Entradas:   informe.md (Markdown de pandoc con marcadores propios), carpeta fig/ y los
            CSV de resultados (resumen_metricas.csv, linealizacion_sobreimpulso.csv,
            estado_estacionario_stanley.csv).
Marcadores: [[clave|ubicación]]   cita IEEE numerada por orden de aparición
            {{eq:x}} {{fig:x}} {{tab:x}}   definen números;  {{ref:x}} {{eqref:x}}
            {{figref:x}} {{tabref:x}}      los referencian;   {{TAB:x}} inserta una tabla
            §T  tabulador de las ecuaciones (centro y derecha);  {{REFS}} lista de referencias.
Requiere:   pandoc >= 3.1, lxml, y LibreOffice sólo para verificar el resultado.

Uso:  python3 build_informe.py --md informe.md --fig fig --data RESULTADOS/ --out Practica2.odt
"""
import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

from lxml import etree

NS = {
    "office": "urn:oasis:names:tc:opendocument:xmlns:office:1.0",
    "style": "urn:oasis:names:tc:opendocument:xmlns:style:1.0",
    "text": "urn:oasis:names:tc:opendocument:xmlns:text:1.0",
    "table": "urn:oasis:names:tc:opendocument:xmlns:table:1.0",
    "fo": "urn:oasis:names:tc:opendocument:xmlns:xsl-fo-compatible:1.0",
    "svg": "urn:oasis:names:tc:opendocument:xmlns:svg-compatible:1.0",
    "draw": "urn:oasis:names:tc:opendocument:xmlns:drawing:1.0",
}
DECL = " ".join(f'xmlns:{k}="{v}"' for k, v in NS.items())

TEXT_W = 16.59  # cm, ancho útil de la página carta con márgenes de 2.5 cm

# ----------------------------------------------------------------- referencias ---
REFS = {
    "macenski": 'S. Macenski, T. Foote, B. Gerkey, C. Lalancette y W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Sci. Robot.*, vol. 7, no. 66, art. eabm6074, may. 2022, doi: 10.1126/scirobotics.abm6074.',
    "koenig": 'N. Koenig y A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," en *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, Sendai, Japón, 2004, pp. 2149–2154, doi: 10.1109/IROS.2004.1389727.',
    "lynch": 'K. M. Lynch y F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*. Cambridge, Reino Unido: Cambridge Univ. Press, 2017.',
    "samson": 'C. Samson, P. Morin y R. Lenain, "Modeling and control of wheeled mobile robots," en *Springer Handbook of Robotics*, 2.ª ed., B. Siciliano y O. Khatib, Eds. Berlín, Alemania: Springer-Verlag, 2016, cap. 49, pp. 1235–1265.',
    "siegwart": 'R. Siegwart e I. R. Nourbakhsh, *Introduction to Autonomous Mobile Robots*. Cambridge, MA, EE. UU.: MIT Press, 2004.',
    "tzafestas": 'S. G. Tzafestas, *Introduction to Mobile Robot Control*. Ámsterdam, Países Bajos: Elsevier, 2014.',
    "thrun06": 'S. Thrun *et al.*, "Stanley: The robot that won the DARPA Grand Challenge," *J. Field Robot.*, vol. 23, no. 9, pp. 661–692, sep. 2006, doi: 10.1002/rob.20147.',
    "hoffmann": 'G. M. Hoffmann, C. J. Tomlin, M. Montemerlo y S. Thrun, "Autonomous automobile trajectory tracking for off-road driving: Controller design, experimental validation and racing," en *Proc. American Control Conf. (ACC)*, Nueva York, NY, EE. UU., 2007, pp. 2296–2301, doi: 10.1109/ACC.2007.4282788.',
    "hart": 'P. E. Hart, N. J. Nilsson y B. Raphael, "A formal basis for the heuristic determination of minimum cost paths," *IEEE Trans. Syst. Sci. Cybern.*, vol. 4, no. 2, pp. 100–107, jul. 1968, doi: 10.1109/TSSC.1968.300136.',
    "lavalle": 'S. M. LaValle, *Planning Algorithms*. Cambridge, Reino Unido: Cambridge Univ. Press, 2006.',
    "thrun05": 'S. Thrun, W. Burgard y D. Fox, *Probabilistic Robotics*. Cambridge, MA, EE. UU.: MIT Press, 2005.',
}

MACROS = {}


# --------------------------------------------------------------------- tablas ---
def f(x, n=3):
    return f"{float(x):.{n}f}"


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def md_table(header, rows, aligns=None):
    aligns = aligns or ["l"] + ["c"] * (len(header) - 1)
    sep = {"l": ":---", "c": ":---:", "r": "---:"}
    out = ["| " + " | ".join(header) + " |", "|" + "|".join(sep[a] for a in aligns) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def find(rows, **kw):
    for r in rows:
        if all(abs(float(r[k]) - v) < 1e-9 if k in ("alpha", "beta", "Kd", "Ka") else r[k] == v for k, v in kw.items()):
            return r
    return None


def metrics_cells(r):
    if r is None:
        return ["–"] * 4
    return [f(r["cte_rmse_m"]), f(r["cte_max_reg_m"]), f(r["t_seguimiento_s"], 1), f(r["var_w_5hz"], 2)]


def tab_pp(data):
    rows = read_csv(os.path.join(data, "resumen_metricas.csv"))
    cfgs = [(0.1, 0.4), (0.5, 0.4), (1.0, 0.4), (4.0, 0.4), (1.0, 0.1), (1.0, 1.0), (1.0, 4.0)]
    body = []
    for a, b in cfgs:
        cells = []
        for rt in ("R1", "R2"):
            cells += metrics_cells(find(rows, algoritmo="pure_pursuit", ruta=rt, alpha=a, beta=b))
        row = [f"{a:g}", f"{b:g}"] + cells
        if (a, b) == (1.0, 0.4):
            row = [f"**{c}**" for c in row]
        body.append(row)
    hdr = ["*α*", "*β*"] + [f"{r} {m}" for r in ("R1", "R2") for m in ("*e*~rms~", "*e*~max~", "*t*~s~", "*V*~ω~")]
    return md_table(hdr, body, ["c"] * 10)


def tab_st(data):
    rows = read_csv(os.path.join(data, "resumen_metricas.csv"))
    cfgs = [(1.0, 0.5), (1.0, 1.0), (1.0, 2.0), (1.0, 5.0), (0.5, 1.0), (2.0, 1.0), (5.0, 1.0)]
    body = []
    for kd, ka in cfgs:
        cells = []
        for rt in ("R1", "R2"):
            cells += metrics_cells(find(rows, algoritmo="stanley", ruta=rt, Kd=kd, Ka=ka))
        row = [f"{kd:g}", f"{ka:g}"] + cells
        if (kd, ka) == (1.0, 1.0):
            row = [f"**{c}**" for c in row]
        body.append(row)
    hdr = ["*k*~d~", "*k*~a~"] + [f"{r} {m}" for r in ("R1", "R2") for m in ("*e*~rms~", "*e*~max~", "*t*~s~", "*V*~ω~")]
    return md_table(hdr, body, ["c"] * 10)


def tab_cmp(data):
    rows = read_csv(os.path.join(data, "resumen_metricas.csv"))
    sel = [
        ("R1", "PP", "base", "*α* = 1, *β* = 0.4", dict(algoritmo="pure_pursuit", alpha=1.0, beta=0.4)),
        ("R1", "ST", "base", "*k*~d~ = 1, *k*~a~ = 1", dict(algoritmo="stanley", Kd=1.0, Ka=1.0)),
        ("R1", "PP", "mejor", "*α* = 1, *β* = 0.1", dict(algoritmo="pure_pursuit", alpha=1.0, beta=0.1)),
        ("R1", "ST", "mejor", "*k*~d~ = 1, *k*~a~ = 5", dict(algoritmo="stanley", Kd=1.0, Ka=5.0)),
        ("R2", "PP", "base", "*α* = 1, *β* = 0.4", dict(algoritmo="pure_pursuit", alpha=1.0, beta=0.4)),
        ("R2", "ST", "base", "*k*~d~ = 1, *k*~a~ = 1", dict(algoritmo="stanley", Kd=1.0, Ka=1.0)),
        ("R2", "PP", "mejor", "*α* = 0.1, *β* = 0.4", dict(algoritmo="pure_pursuit", alpha=0.1, beta=0.4)),
        ("R2", "ST", "mejor", "*k*~d~ = 5, *k*~a~ = 1", dict(algoritmo="stanley", Kd=5.0, Ka=1.0)),
    ]
    body = []
    for rt, alg, kind, cfg, kw in sel:
        r = find(rows, ruta=rt, **kw)
        body.append([rt, alg, kind, cfg, f(r["cte_rmse_m"]), f(r["cte_max_reg_m"]), f(r["t_seguimiento_s"], 1),
                     f(r["v_media_ms"]), f(r["var_w_5hz"], 2)])
    hdr = ["Ruta", "Alg.", "Config.", "Parámetros", "*e*~rms~ (m)", "*e*~max~ (m)", "*t*~s~ (s)", "*v*~med~ (m/s)", "*V*~ω~ (rad/s)"]
    return md_table(hdr, body, ["c"] * 9)


def tab_lin(data):
    rows = read_csv(os.path.join(data, "linealizacion_sobreimpulso.csv"))
    body = []
    for r in rows:
        cfg = r["config"].replace("alpha", "*α*").replace("beta", "*β*").replace("kd", "*k*~d~").replace("ka", "*k*~a~")
        body.append(["PP" if r["algoritmo"] == "pure_pursuit" else "ST", cfg, f(r["ka_eq"], 2), f(r["kd_eq"], 2),
                     f(r["wn"], 2), f(r["zeta"], 2), f(float(r["sobreimpulso_pred"]) * 100, 1), f(float(r["sobreimpulso_sim"]) * 100, 1)])
    hdr = ["Alg.", "Configuración", "*k*~a,eq~", "*k*~d,eq~", "*ω*~n~ (rad/s)", "*ζ*", "*S* previsto (%)", "*S* simulado (%)"]
    return md_table(hdr, body, ["c"] * 8)


def tab_ess(data):
    rows = [r for r in read_csv(os.path.join(data, "estado_estacionario_stanley.csv")) if r["ruta"] == "R2"]
    body = []
    for r in rows:
        body.append([f"{float(r['Kd']):g}", f"{float(r['Ka']):g}", f(r["v_media"]), f(r["e_pred_m"]),
                     f(r["rms_medido_m"]), f(r["cociente"], 2)])
    hdr = ["*k*~d~", "*k*~a~", "*v*~med~ (m/s)", "Previsto (m)", "*e*~rms~ medido (m)", "Medido/previsto"]
    return md_table(hdr, body, ["c"] * 6)


def tab_ppparam():
    hdr = ["Cantidad", "Valor 1", "Valor 2", "Valor 3", "Valor 4"]
    rows = [
        ["*β*", "0.1", "0.4", "1", "4"],
        ["Ganancia *K* = *ω*~max~/(2*β*) (rad/s por rad)", "5", "1.25", "0.5", "0.125"],
        ["Error angular de saturación, 4*β* (rad)", "0.4", "1.6", "4 (no se alcanza)", "16 (no se alcanza)"],
        ["Avance del robot mientras corrige, *β* (m)", "0.1", "0.4", "1", "4"],
        ["*α*", "0.1", "0.5", "1", "4"],
        ["Error angular con *v* = *v*~max~/*e*, √*α* (rad)", "0.32", "0.71", "1.0", "2.0"],
    ]
    return md_table(hdr, rows, ["l", "c", "c", "c", "c"])


def tab_rutas():
    hdr = ["Ruta", "Meta (m)", "Longitud (m)", "Puntos", "*κ* RMS (m^−1^)", "*κ* máx. (m^−1^)", "Descripción"]
    rows = [
        ["R1", "(4.47, −0.76)", "4.53", "89", "0.24", "0.71", "Casi recta; curva suave al final"],
        ["R2", "(0.00, 5.00)", "5.74", "112", "0.73", "1.86", "Forma de \"S\" (≈ 1 m de excursión lateral); orientación inicial 90° fuera de la ruta"],
    ]
    return md_table(hdr, rows, ["c", "c", "c", "c", "c", "c", "l"])


def tab_diseno():
    hdr = ["Algoritmo", "Parámetro que varía", "Valores", "Parámetros fijos"]
    rows = [
        ["Pure pursuit", "*α*", "0.1, 0.5, **1**, 4", "*β* = 0.4"],
        ["Pure pursuit", "*β*", "0.1, **0.4**, 1, 4", "*α* = 1"],
        ["Stanley", "*k*~a~", "0.5, **1**, 2, 5", "*k*~d~ = 1"],
        ["Stanley", "*k*~d~", "0.5, **1**, 2, 5", "*k*~a~ = 1"],
        ["Ambos", "Ruta", "R1, R2", "*v*~max~ = 0.5 m/s, *ω*~max~ = 1.0 rad/s, `tol_dist` = 0.3 m, `tol_angle` = 0.1 rad, *k*~v~ = 5 (Stanley)"],
    ]
    return md_table(hdr, rows, ["l", "c", "c", "l"])


TABLES = {"pp": tab_pp, "st": tab_st, "cmp": tab_cmp, "lin": tab_lin, "ess": tab_ess}
STATIC_TABLES = {"ppparam": tab_ppparam, "rutas": tab_rutas, "diseno": tab_diseno}


# ----------------------------------------------------------------- preproceso ---
INLINE_MACROS = {}


def preprocess(md, data):
    # 1) numeración de ecuaciones, figuras y tablas por orden de definición
    nums = {"eq": {}, "fig": {}, "tab": {}}
    for kind in nums:
        for m in re.finditer(r"\{\{%s:([A-Za-z0-9_]+)\}\}" % kind, md):
            nums[kind].setdefault(m.group(1), len(nums[kind]) + 1)

    def sub_def(kind, text):
        if kind == "eq":
            return re.sub(r"\{\{eq:([A-Za-z0-9_]+)\}\}", lambda m: f"({nums['eq'][m.group(1)]})", text)
        return re.sub(r"\{\{%s:([A-Za-z0-9_]+)\}\}" % kind, lambda m: str(nums[kind][m.group(1)]), text)

    for kind in nums:
        md = sub_def(kind, md)

    def ref(kind, fmt):
        pat = {"eqref": ("eq", "Ec. ({n})"), "ref": ("eq", "({n})"), "figref": ("fig", "Figura {n}"),
               "tabref": ("tab", "Tabla {n}")}[kind]
        nonlocal md
        md = re.sub(r"\{\{%s:([A-Za-z0-9_]+)\}\}" % kind,
                    lambda m: pat[1].format(n=nums[pat[0]][m.group(1)]), md)

    for k in ("eqref", "ref", "figref", "tabref"):
        ref(k, None)

    # 2) tablas generadas
    def tab(m):
        key = m.group(1)
        return (TABLES.get(key) or STATIC_TABLES[key])(data) if key in TABLES else STATIC_TABLES[key]()

    md = re.sub(r"\{\{TAB:([A-Za-z0-9_]+)\}\}", tab, md)

    # 3) citas IEEE por orden de primera aparición
    order = []

    def cite(m):
        key, _, loc = m.group(1).partition("|")
        if key not in REFS:
            sys.exit(f"Referencia desconocida: {key}")
        if key not in order:
            order.append(key)
        n = order.index(key) + 1
        return f"[{n}, {loc}]" if loc else f"[{n}]"

    md = re.sub(r"\[\[([^\]]+)\]\]", cite, md)
    refs = "\n\n".join(
        '::: {custom-style="Referencia"}\n[%d] %s\n:::' % (i + 1, REFS[k]) for i, k in enumerate(order))
    md = md.replace("{{REFS}}", refs)
    unused = [k for k in REFS if k not in order]
    if unused:
        print("AVISO: referencias definidas y no citadas:", unused)

    # 4) tabuladores de ecuaciones: sin espacios alrededor de las fórmulas
    md = re.sub(r"§T\s+\$", "§T$", md)
    md = re.sub(r"\$\s+§T", "$§T", md)
    left = re.findall(r"\{\{[^}]*\}\}", md)
    if left:
        sys.exit("Marcadores sin resolver: " + ", ".join(sorted(set(left))))
    return md, order, nums


# ------------------------------------------------------------ documento base ---
def para_style(name, parent="Standard", pp="", tp="", extra="", family="paragraph", display=None, next_=None):
    disp = f' style:display-name="{display}"' if display else ""
    nxt = f' style:next-style-name="{next_}"' if next_ else ""
    par = f' style:parent-style-name="{parent}"' if parent else ""
    pp_xml = f"<style:paragraph-properties {pp}>{extra}</style:paragraph-properties>" if (pp or extra) else ""
    tp_xml = f"<style:text-properties {tp}/>" if tp else ""
    return (f'<style:style {DECL} style:name="{name}" style:family="{family}"{par}{nxt}{disp}>'
            f"{pp_xml}{tp_xml}</style:style>")


def build_reference(tmp):
    ref = os.path.join(tmp, "ref.odt")
    with open(ref, "wb") as fh:
        fh.write(subprocess.check_output(["pandoc", "--print-default-data-file", "reference.odt"]))
    d = os.path.join(tmp, "ref")
    with zipfile.ZipFile(ref) as z:
        z.extractall(d)
    sp = os.path.join(d, "styles.xml")
    tree = etree.parse(sp)
    root = tree.getroot()
    styles = root.find("office:styles", NS)
    auto = root.find("office:automatic-styles", NS)
    master = root.find("office:master-styles", NS)

    def upsert(xml):
        el = etree.fromstring(xml)
        name = el.get(f"{{{NS['style']}}}name")
        for old in styles.findall("style:style", NS):
            if old.get(f"{{{NS['style']}}}name") == name:
                styles.remove(old)
        styles.append(el)

    # estilo por omisión de párrafo y texto
    for ds in styles.findall("style:default-style", NS):
        if ds.get(f"{{{NS['style']}}}family") == "paragraph":
            for ch in list(ds):
                ds.remove(ch)
            ds.append(etree.fromstring(
                f'<style:paragraph-properties {DECL} fo:hyphenation-ladder-count="no-limit" style:text-autospace="ideograph-alpha" '
                f'style:line-break="strict" style:writing-mode="page"/>'))
            ds.append(etree.fromstring(
                f'<style:text-properties {DECL} style:font-name="Times New Roman" fo:font-size="12pt" fo:language="es" fo:country="MX" '
                f'style:font-name-asian="Times New Roman" style:font-name-complex="Times New Roman"/>'))
    # fuentes
    ff = root.find("office:font-face-decls", NS)
    if not any(x.get(f"{{{NS['style']}}}name") == "Times New Roman" for x in ff):
        ff.append(etree.fromstring(
            f'<style:font-face {DECL} style:name="Times New Roman" svg:font-family="\'Times New Roman\'" '
            f'style:font-family-generic="roman" style:font-pitch="variable"/>'))

    S = "Standard"
    upsert(para_style("Standard", parent=None, pp='fo:line-height="115%" fo:margin-top="0cm" fo:margin-bottom="0.18cm" fo:orphans="2" fo:widows="2"',
                      tp='style:font-name="Times New Roman" fo:font-size="12pt"'))
    upsert(para_style("Text_20_body", S, pp='fo:text-align="justify" style:justify-single-word="false" fo:margin-bottom="0.22cm"',
                      display="Text body"))
    upsert(para_style("First_20_paragraph", "Text_20_body", display="First paragraph"))
    for i, (sz, mt, mb) in enumerate([(16, 0.7, 0.25), (13.5, 0.5, 0.15), (12, 0.35, 0.1)], 1):
        upsert(para_style(f"Heading_20_{i}", "Heading", pp=f'fo:margin-top="{mt}cm" fo:margin-bottom="{mb}cm" fo:keep-with-next="always" fo:text-align="start"',
                          tp=f'style:font-name="Times New Roman" fo:font-size="{sz}pt" fo:font-weight="bold" fo:color="#1f2a44"', next_="Text_20_body",
                          display=f"Heading {i}"))
    upsert(para_style("Table_20_Contents", S, pp='fo:text-align="center" fo:margin-top="0.03cm" fo:margin-bottom="0.03cm" fo:line-height="100%"',
                      tp='fo:font-size="9pt"', display="Table Contents"))
    upsert(para_style("Table_20_Heading", "Table_20_Contents", pp='fo:text-align="center"', tp='fo:font-weight="bold" fo:font-size="9pt"',
                      display="Table Heading"))
    upsert(para_style("TableCaption", S, pp='fo:text-align="start" fo:margin-top="0.3cm" fo:margin-bottom="0.15cm" fo:keep-with-next="always" fo:line-height="105%"',
                      tp='fo:font-size="10pt"'))
    upsert(para_style("FigureCaption", S, pp='fo:text-align="justify" fo:margin-top="0.1cm" fo:margin-bottom="0.45cm" fo:line-height="105%"',
                      tp='fo:font-size="10pt"'))
    upsert(para_style("FigureWithCaption", S, pp='fo:text-align="center" fo:margin-top="0.25cm" fo:margin-bottom="0cm" fo:keep-with-next="always"'))
    upsert(para_style("Figure", S, pp='fo:text-align="center" fo:keep-with-next="always"'))
    tabs = (f'<style:tab-stops><style:tab-stop style:position="{TEXT_W / 2:.3f}cm" style:type="center"/>'
            f'<style:tab-stop style:position="{TEXT_W:.3f}cm" style:type="right"/></style:tab-stops>')
    upsert(para_style("Ecuacion", S, pp='fo:text-align="start" fo:margin-top="0.15cm" fo:margin-bottom="0.2cm" fo:keep-together="always"', extra=tabs))
    upsert(para_style("RubricaPagina", S, pp='fo:text-align="center" fo:margin="0cm"  fo:line-height="100%"'))
    upsert(para_style("PortadaInst", S, pp='fo:text-align="center" fo:margin-top="1.2cm" fo:margin-bottom="0.1cm"',
                      tp='fo:font-size="13pt" fo:font-weight="bold" fo:letter-spacing="0.05cm"'))
    upsert(para_style("PortadaFac", S, pp='fo:text-align="center" fo:margin-bottom="0.1cm"', tp='fo:font-size="13pt" fo:font-weight="bold"'))
    upsert(para_style("PortadaCurso", S, pp='fo:text-align="center" fo:margin-bottom="1.1cm"', tp='fo:font-size="11.5pt"'))
    upsert(para_style("Titulo", S, pp='fo:text-align="center" fo:margin-bottom="0.2cm"', tp='fo:font-size="22pt" fo:font-weight="bold" fo:color="#1f2a44"'))
    upsert(para_style("Subtitulo", S, pp='fo:text-align="center" fo:margin-bottom="0.8cm"', tp='fo:font-size="13pt" fo:font-style="italic"'))
    upsert(para_style("PortadaDatos", S, pp='fo:text-align="center" fo:margin-bottom="0.9cm" fo:line-height="130%"', tp='fo:font-size="12pt"'))
    upsert(para_style("ResumenTitulo", S, pp='fo:text-align="start" fo:margin-top="0.2cm" fo:margin-bottom="0.1cm" fo:keep-with-next="always"',
                      tp='fo:font-size="13pt" fo:font-weight="bold" fo:color="#1f2a44"'))
    upsert(para_style("ResumenTexto", S, pp='fo:text-align="justify" style:justify-single-word="false" fo:margin-bottom="0.2cm" fo:margin-left="0.6cm" fo:margin-right="0.6cm"',
                      tp='fo:font-size="11pt"'))
    upsert(para_style("Referencia", S, pp='fo:text-align="start" fo:margin-left="1.0cm" fo:text-indent="-1.0cm" fo:margin-bottom="0.15cm" fo:line-height="105%"',
                      tp='fo:font-size="10.5pt"'))

    # página: carta vertical con márgenes de 2.5 cm y carta horizontal para la rúbrica
    for pl in auto.findall("style:page-layout", NS):
        auto.remove(pl)
    auto.append(etree.fromstring(
        f'<style:page-layout {DECL} style:name="Mpm1"><style:page-layout-properties fo:page-width="21.59cm" fo:page-height="27.94cm" '
        f'style:num-format="1" style:print-orientation="portrait" fo:margin-top="2.5cm" fo:margin-bottom="2.2cm" fo:margin-left="2.5cm" '
        f'fo:margin-right="2.5cm" style:writing-mode="lr-tb"/><style:footer-style><style:header-footer-properties fo:min-height="0.5cm" '
        f'fo:margin-top="0.4cm"/></style:footer-style></style:page-layout>'))
    auto.append(etree.fromstring(
        f'<style:page-layout {DECL} style:name="Mpm2"><style:page-layout-properties fo:page-width="27.94cm" fo:page-height="21.59cm" '
        f'style:num-format="1" style:print-orientation="landscape" fo:margin-top="1.0cm" fo:margin-bottom="1.0cm" fo:margin-left="1.75cm" '
        f'fo:margin-right="1.75cm" style:writing-mode="lr-tb"/></style:page-layout>'))
    for mp in list(master):
        master.remove(mp)
    master.append(etree.fromstring(
        f'<style:master-page {DECL} style:name="Standard" style:page-layout-name="Mpm1"><style:footer>'
        f'<text:p text:style-name="MP1"><text:page-number text:select-page="current">1</text:page-number></text:p></style:footer></style:master-page>'))
    master.append(etree.fromstring(
        f'<style:master-page {DECL} style:name="Landscape" style:page-layout-name="Mpm2" style:next-style-name="Standard"/>'))
    tree.write(sp, xml_declaration=True, encoding="UTF-8")
    out = os.path.join(tmp, "reference_custom.odt")
    zip_dir(d, out)
    return out


def zip_dir(d, out):
    with zipfile.ZipFile(out, "w") as z:
        z.write(os.path.join(d, "mimetype"), "mimetype", compress_type=zipfile.ZIP_STORED)
        for base, _, files in os.walk(d):
            for fn in files:
                full = os.path.join(base, fn)
                arc = os.path.relpath(full, d)
                if arc == "mimetype":
                    continue
                z.write(full, arc, compress_type=zipfile.ZIP_DEFLATED)


# --------------------------------------------------------------- postproceso ---
FUNC_NAMES = r"[A-Za-z][A-Za-z0-9]*"


def fix_formula(xml):
    # nombres de operadores (cos, max, tanh, atan2, sgn...) en redondas como identificadores
    xml = re.sub(r"<mo>(%s)</mo>" % FUNC_NAMES,
                 lambda m: f'<mi mathvariant="normal">{m.group(1)}</mi>' if len(m.group(1)) > 1 else m.group(0), xml)
    # acentos combinantes (U+0307, U+0308...) -> caracteres espaciados que LibreOffice Math dibuja bien
    acc = {"\u0307": "\u02d9", "\u0308": "\u00a8", "\u0302": "^", "\u0303": "~", "\u0304": "\u00af"}
    xml = re.sub(r'(<mo accent="true">)([^<])(</mo>)', lambda m: m.group(1) + acc.get(m.group(2), m.group(2)) + m.group(3), xml)
    # LibreOffice Math dibuja mal los acentos (ẋ, ë, ω̄) importados desde MathML puro; si el objeto trae
    # la anotación StarMath, usa ese texto. Se agrega sólo a las fórmulas con acentos.
    root = etree.fromstring(xml.encode("utf-8"))
    if any(mv.get("accent") == "true" or (len(mv) == 2 and mv[1].get("accent") == "true") for mv in root.iter(MN + "mover")):
        body = list(root)
        sem = etree.SubElement(root, MN + "semantics")
        for el in body:
            sem.append(el)
        ann = etree.SubElement(sem, MN + "annotation")
        ann.set("encoding", "StarMath 5.0")
        ann.text = " ".join(starmath(el) for el in body)
    return etree.tostring(root, xml_declaration=True, encoding="utf-8").decode("utf-8")


MN = "{http://www.w3.org/1998/Math/MathML}"
SM_OPS = {"\u2212": "-", "\u2211": "sum", "\u00d7": "times", "\u00b7": "cdot"}
SM_ACC = {"\u02d9": "dot", "\u00a8": "ddot", "\u00af": "bar", "^": "hat", "~": "tilde"}


def starmath(el):
    """Subconjunto de MathML (el que genera pandoc para este reporte) -> texto StarMath de LibreOffice."""
    t = el.tag.replace(MN, "")
    kids = list(el)
    if t in ("math", "mrow", "semantics"):
        inner = " ".join(starmath(k) for k in kids)
        return inner if t != "mrow" else "{" + inner + "}"
    if t == "mi":
        x = el.text or ""
        return '"%s"' % x if el.get("mathvariant") == "normal" and len(x) > 1 else x
    if t == "mn":
        return el.text or ""
    if t == "mtext":
        return '"%s"' % (el.text or "")
    if t == "mspace":
        w = float((el.get("width") or "0em").replace("em", ""))
        return "~" * 4 if w >= 1.5 else ("`" if w > 0 else "")
    if t == "mo":
        x = el.text or ""
        if x == "|":
            return "lline" if el.get("form") == "prefix" else "rline"
        return SM_OPS.get(x, x)
    if t == "mfrac":
        return "{%s} over {%s}" % (starmath(kids[0]), starmath(kids[1]))
    if t == "msub":
        if kids[0].tag == MN + "mo" and (kids[0].text or "") == "\u2211":
            return "sum from {%s}" % starmath(kids[1])
        return "{%s}_{%s}" % (starmath(kids[0]), starmath(kids[1]))
    if t == "msup":
        return "{%s}^{%s}" % (starmath(kids[0]), starmath(kids[1]))
    if t == "msubsup":
        return "{%s}_{%s}^{%s}" % (starmath(kids[0]), starmath(kids[1]), starmath(kids[2]))
    if t == "msqrt":
        return "sqrt {%s}" % " ".join(starmath(k) for k in kids)
    if t == "mover":
        return "{%s {%s}}" % (SM_ACC.get(kids[1].text, "bar"), starmath(kids[0]))
    raise ValueError("MathML sin convertir a StarMath: " + t)


def display_formulas(content_xml):
    """Números de objeto Formula-N usados en párrafos de ecuación (se dibujan en modo display)."""
    out = set()
    for m in re.finditer(r'<text:p text:style-name="Ecuacion">(.*?)</text:p>', content_xml, re.S):
        out.update(re.findall(r'xlink:href="(?:\./)?(Formula-\d+)/?"', m.group(1)))
    return out


COL_WIDTHS = {  # tabla (orden de aparición) -> pesos relativos de las columnas
    1: [3.2, 1, 1, 1, 1],
    2: [0.7, 1.4, 1.2, 0.9, 1.1, 1.1, 3.4],
    3: [1.8, 1.6, 1.4, 4.6],
    4: [1, 1, 1.3, 1.3, 1, 1, 1.3, 1.3, 1, 1],
    5: [1, 1, 1.3, 1.3, 1, 1, 1.3, 1.3, 1, 1],
    6: [0.8, 0.8, 1, 2.4, 1.1, 1.1, 1, 1.2, 1.2],
    7: [0.9, 2.4, 1, 1, 1.2, 0.8, 1.2, 1.2],
    8: [1, 1, 1.3, 1.3, 1.5, 1.4],
}


def postprocess(odt_in, odt_out, title, author):
    tmp = tempfile.mkdtemp()
    d = os.path.join(tmp, "x")
    with zipfile.ZipFile(odt_in) as z:
        z.extractall(d)
    # fórmulas
    disp = display_formulas(open(os.path.join(d, "content.xml"), encoding="utf-8").read())
    for name in os.listdir(d):
        if name.startswith("Formula-"):
            p = os.path.join(d, name, "content.xml")
            s = open(p, encoding="utf-8").read()
            open(p, "w", encoding="utf-8").write(fix_formula(s))
            if name in disp:  # ecuaciones destacadas: fracciones y límites a tamaño completo
                sp = os.path.join(d, name, "settings.xml")
                t = open(sp, encoding="utf-8").read()
                open(sp, "w", encoding="utf-8").write(t.replace('"IsTextMode" config:type="boolean">true', '"IsTextMode" config:type="boolean">false'))
    print("Fórmulas en modo display:", len(disp))
    # contenido
    cp = os.path.join(d, "content.xml")
    s = open(cp, encoding="utf-8").read()
    s = s.replace("§T", "<text:tab/>")
    # cambios de página/maestro: LibreOffice sólo los respeta en estilos automáticos
    auto_ps = ('<style:style style:name="PRub" style:family="paragraph" style:parent-style-name="RubricaPagina" '
               'style:master-page-name="Landscape"/>'
               '<style:style style:name="PCov" style:family="paragraph" style:parent-style-name="PortadaInst" '
               'style:master-page-name="Standard"/>')
    s = s.replace("</office:automatic-styles>", auto_ps + "</office:automatic-styles>", 1)
    s = s.replace('<text:p text:style-name="RubricaPagina">', '<text:p text:style-name="PRub">', 1)
    s = s.replace('<text:p text:style-name="PortadaInst">', '<text:p text:style-name="PCov">', 1)
    s = s.replace('<style:table-cell-properties fo:border="none" />', "CELLPROPS")
    # estilos de celda
    s = re.sub(r'(<style:style style:name="TableHeaderRowCell" style:family="table-cell">\s*)CELLPROPS',
               r'\1<style:table-cell-properties fo:padding="0.04cm" fo:border-left="none" fo:border-right="none" '
               r'fo:border-top="1pt solid #222222" fo:border-bottom="0.75pt solid #222222" fo:background-color="#e9edf5" style:vertical-align="middle"/>', s)
    s = re.sub(r'(<style:style style:name="TableRowCell" style:family="table-cell">\s*)CELLPROPS',
               r'\1<style:table-cell-properties fo:padding="0.04cm" fo:border-left="none" fo:border-right="none" '
               r'fo:border-top="none" fo:border-bottom="0.4pt solid #b5b5b5" style:vertical-align="middle"/>', s)
    s = s.replace("CELLPROPS", '<style:table-cell-properties fo:border="none" />')
    # ancho de columnas (pesos relativos por tabla, en orden de aparición)
    tables = re.findall(r'<style:style style:name="(Table\d+)" style:family="table">', s)
    for idx, tname in enumerate(tables, 1):
        s = s.replace('<style:style style:name="%s" style:family="table">\n      <style:table-properties table:align="center" style:rel-width="100%%" />' % tname,
                      '<style:style style:name="%s" style:family="table">\n      <style:table-properties table:align="center" style:rel-width="100%%" fo:margin-top="0.05cm" fo:margin-bottom="0.35cm"/>' % tname)
        cols = re.findall(r'<style:style style:name="(%s\.[A-Z]+)" style:family="table-column">' % tname, s)
        w = COL_WIDTHS.get(idx)
        if not w or len(w) != len(cols):
            print(f"AVISO: tabla {idx}: {len(cols)} columnas, {len(w) if w else 0} pesos")
            continue
        for c, wi in zip(cols, w):
            s = re.sub(r'(<style:style style:name="%s" style:family="table-column">\s*<style:table-column-properties style:rel-column-width=")[^"]*(")' % re.escape(c),
                       r'\g<1>%d*\g<2>' % int(wi * 1000), s)
    open(cp, "w", encoding="utf-8").write(s)
    # metadatos
    mp = os.path.join(d, "meta.xml")
    m = open(mp, encoding="utf-8").read()
    m = re.sub(r"<dc:title>.*?</dc:title>", "", m)
    m = re.sub(r"<dc:creator>.*?</dc:creator>", "", m)
    m = m.replace("</office:meta>", f"<dc:title>{title}</dc:title><dc:creator>{author}</dc:creator></office:meta>")
    open(mp, "w", encoding="utf-8").write(m)
    zip_dir(d, odt_out)
    shutil.rmtree(tmp)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--md", default="informe.md")
    ap.add_argument("--fig", default="fig")
    ap.add_argument("--data", default="resultados")
    ap.add_argument("--out", default="Practica2.odt")
    ap.add_argument("--keep-md", default=None, help="guarda el Markdown ya procesado")
    args = ap.parse_args()

    md = open(args.md, encoding="utf-8").read()
    for k, v in INLINE_MACROS.items():
        md = md.replace(f"¦{k}¦", v)
    md, order, nums = preprocess(md, args.data)
    print("Referencias por orden de cita:", ", ".join(f"[{i + 1}] {k}" for i, k in enumerate(order)))
    print("Ecuaciones:", len(nums["eq"]), " Figuras:", len(nums["fig"]), " Tablas:", len(nums["tab"]))
    tmp = tempfile.mkdtemp()
    ref = build_reference(tmp)
    md_path = os.path.join(tmp, "in.md")
    if os.path.abspath(args.fig) != os.path.join(os.path.dirname(os.path.abspath(args.md)), "fig"):
        pass
    open(md_path, "w", encoding="utf-8").write(md)
    if args.keep_md:
        open(args.keep_md, "w", encoding="utf-8").write(md)
    raw = os.path.join(tmp, "raw.odt")
    subprocess.check_call(["pandoc", md_path, "-f", "markdown+tex_math_dollars+pipe_tables+smart-auto_identifiers",
                           "-t", "odt", "--reference-doc", ref, "-o", raw,
                           "--resource-path", os.path.dirname(os.path.abspath(args.md))])
    postprocess(raw, args.out, "Práctica 2. Seguimiento de rutas", "Omar Rosario Carrasco")
    shutil.rmtree(tmp)
    print("Escrito", args.out)


if __name__ == "__main__":
    main()
