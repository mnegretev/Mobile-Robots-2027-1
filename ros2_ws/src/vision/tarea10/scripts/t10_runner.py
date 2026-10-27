#!/usr/bin/env python3
"""
t10_runner.py - Tarea 10 (extraccion de caracteristicas geometricas), Robots Moviles FI-UNAM 2027-1
Omar Rosario Carrasco.

Corre automaticamente los experimentos de la tarea (variacion de parametros de los pasos 4, 7 y
10, y comparacion de desempeno y tiempos del paso 11). Primero pone la camara del brazo mirando
al frente (t10_camara.py; en la pose de arranque mira al piso). Luego, para cada configuracion:
  1. lanza `ros2 run geometric_features <nodo>` con esos parametros en "modo experimento"
     (-p log_dir:=... -p n_frames:=N): el nodo guarda en frames.csv, por cada cuadro, el tiempo
     de deteccion de lineas, el numero de lineas y los pixeles de borde,
  2. espera a que el nodo procese N cuadros y guarde sus imagenes (original, bordes, resultado),
  3. lo detiene con Ctrl+C (SIGINT) y registra la corrida en corridas.csv.
El robot no se mueve: todas las corridas ven la misma escena, asi que las diferencias se deben
solo a los parametros.

Requisitos (en otra terminal bash de la distrobox):
  ros2 launch house_simul house_simul.launch.py realsense:=True
  y ningun canny / hough / houghP abierto.

Uso (a traves de run_experiments.sh, que compila el paquete y hace source del workspace):
  bash run_experiments.sh -n              # solo muestra el plan
  bash run_experiments.sh -m 2            # prueba de humo (las 2 primeras corridas)
  bash run_experiments.sh                 # todo
  bash run_experiments.sh --only hough    # solo un nodo, o ids: --only HB,HT1,PB
  bash run_experiments.sh --continuar     # reanuda el ultimo lote (salta las corridas terminadas)
  bash run_experiments.sh --frames 200    # cuadros por corrida de hough/houghP (default 100)
  bash run_experiments.sh --pose alta     # camara mas alta (~1.1 m); --pose ninguna: no mueve el brazo
  bash run_experiments.sh --inclinacion 5 # camara 5 grados hacia arriba (negativo: hacia abajo)
Salida: ../datos/<lote>/ con una carpeta por corrida, corridas.csv y sistema.json.
"""
import argparse, csv, filecmp, glob, json, os, platform, shutil, signal, subprocess, sys, time
from datetime import datetime

import t10_camara          # pose del brazo para que la camara mire al frente

HERE    = os.path.dirname(os.path.abspath(__file__))
WS      = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))        # .../ros2_ws
PKG_SRC = os.path.join(WS, "src", "vision", "geometric_features", "geometric_features")
CAM_TOPICS = ["/camera/color/image_raw", "/camera/image_raw"]
NODE_NAMES = {"canny": "canny_node", "hough": "hough_node", "houghP": "houghP_node"}
FRAMES_CANNY = 20          # Canny solo necesita las imagenes, no estadisticas de tiempo
OVERHEAD_S = 15.0          # arranque + cierre de cada nodo (medido en la Steam Deck: 13-20 s)

# ----------------------------------------------------------------------------------------------
# Configuraciones. Se varia UN parametro a la vez respecto a una base; *0 es el comando del PDF.
# theta en radianes: 0.0087 = 0.5 grados, 0.0175 = 1, 0.0349 = 2, 0.1 = 5.7.
BASE_C  = dict(canny_l=50, canny_u=150, blur=5)
BASE_H  = dict(canny_l=50, canny_u=150, rho=1, theta=0.0175, hough_th=150, blur=0)
BASE_P  = dict(canny_l=50, canny_u=150, rho=1, theta=0.0175, hough_th=50, min_len=30, max_gap=5, blur=0)
PDF_H   = dict(canny_l=10, canny_u=20, rho=5, theta=0.1, hough_th=50, blur=0)
PDF_P   = dict(canny_l=10, canny_u=20, rho=5, theta=0.1, hough_th=50, min_len=1, max_gap=1, blur=0)

def cfg(cid, nodo, grupo, base, **cambios):
    p = dict(base)
    p.update(cambios)
    return dict(id=cid, nodo=nodo, grupo=grupo, params=p)

CONFIGS = [
    # --- Canny (pasos 3-4): umbrales con filtro 5x5 y ancho del filtro con umbrales 50/150
    cfg("C0",  "canny",  "practica",   dict(canny_l=10, canny_u=20, blur=5)),
    cfg("C1",  "canny",  "umbrales",   BASE_C, canny_l=30,  canny_u=90),
    cfg("C2",  "canny",  "base",       BASE_C),
    cfg("C3",  "canny",  "umbrales",   BASE_C, canny_l=100, canny_u=200),
    cfg("C4",  "canny",  "umbrales",   BASE_C, canny_l=150, canny_u=300),
    cfg("C5",  "canny",  "filtro",     BASE_C, blur=0),
    cfg("C6",  "canny",  "filtro",     BASE_C, blur=3),
    cfg("C7",  "canny",  "filtro",     BASE_C, blur=9),
    # --- Transformada Hough (pasos 6-7 y 11)
    cfg("H0",  "hough",  "practica",   PDF_H),
    cfg("HB",  "hough",  "base",       BASE_H),
    cfg("HT1", "hough",  "umbral",     BASE_H, hough_th=100),
    cfg("HT2", "hough",  "umbral",     BASE_H, hough_th=200),
    cfg("HT3", "hough",  "umbral",     BASE_H, hough_th=250),
    cfg("HA1", "hough",  "theta",      BASE_H, theta=0.0087),
    cfg("HA2", "hough",  "theta",      BASE_H, theta=0.0349),
    cfg("HA3", "hough",  "theta",      BASE_H, theta=0.1),
    cfg("HR1", "hough",  "rho",        BASE_H, rho=2),
    cfg("HR2", "hough",  "rho",        BASE_H, rho=5),
    cfg("HC1", "hough",  "canny",      BASE_H, canny_l=30,  canny_u=90),
    cfg("HC2", "hough",  "canny",      BASE_H, canny_l=100, canny_u=200),
    cfg("HF",  "hough",  "filtro",     BASE_H, blur=5),
    cfg("HS",  "hough",  "reloj",      BASE_H, use_sim_time=True),
    cfg("HB2", "hough",  "repeticion", BASE_H),
    # --- Transformada Hough probabilistica (pasos 9-10 y 11)
    cfg("P0",  "houghP", "practica",   PDF_P),
    cfg("PB",  "houghP", "base",       BASE_P),
    cfg("PL1", "houghP", "min_len",    BASE_P, min_len=10),
    cfg("PL2", "houghP", "min_len",    BASE_P, min_len=60),
    cfg("PL3", "houghP", "min_len",    BASE_P, min_len=100),
    cfg("PG1", "houghP", "max_gap",    BASE_P, max_gap=1),
    cfg("PG2", "houghP", "max_gap",    BASE_P, max_gap=20),
    cfg("PT1", "houghP", "umbral",     BASE_P, hough_th=20),
    cfg("PT2", "houghP", "umbral",     BASE_P, hough_th=100),
    cfg("PA1", "houghP", "theta",      BASE_P, theta=0.1),
    cfg("PR1", "houghP", "rho",        BASE_P, rho=5),
    cfg("PF",  "houghP", "filtro",     BASE_P, blur=5),
    cfg("PB2", "houghP", "repeticion", BASE_P),
]

PARAM_COLS = ["canny_l", "canny_u", "blur", "rho", "theta", "hough_th", "min_len", "max_gap", "use_sim_time"]
RUN_COLS   = ["id", "nodo", "grupo"] + PARAM_COLS + ["n_frames", "estado", "filas", "inicio", "duracion_s", "carpeta"]


def fmt_val(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def ros_args(params):
    out = []
    for k, v in params.items():
        out += ["-p", "%s:=%s" % (k, fmt_val(v))]
    return out


def frames_for(c, args):
    return FRAMES_CANNY if c["nodo"] == "canny" else args.frames


def print_plan(plan, args, fps=10.0, q_brazo=None):
    print("\nPlan: %d corridas (Canny %d cuadros, Hough/HoughP %d cuadros por corrida)"
          % (len(plan), FRAMES_CANNY, args.frames))
    if q_brazo is None:
        print("Brazo: no se mueve (--pose ninguna)\n")
    else:
        pos, pitch = t10_camara.camara(q_brazo)
        print("Brazo: pose '%s' -> camara a %.2f m sobre la base, inclinacion %+.0f grados\n"
              % (args.pose, pos[2], pitch))
    print("  %-4s %-7s %-11s %s" % ("id", "nodo", "grupo", "parametros"))
    for c in plan:
        txt = "  ".join("%s=%s" % (k, fmt_val(v)) for k, v in c["params"].items())
        print("  %-4s %-7s %-11s %s" % (c["id"], c["nodo"], c["grupo"], txt))
    eta = sum(frames_for(c, args) / fps + OVERHEAD_S for c in plan)
    txt = "%.0f min" % (eta / 60.0) if eta >= 90 else "%.0f s" % eta
    print("\nDuracion estimada: ~%s (camara a %.1f Hz)\n" % (txt, fps))


# ----------------------------------------------------------------------------------------------
def check_package():
    """El geometric_features que ve Python/ROS debe ser el de ESTE repo y tener el modo experimento."""
    import importlib.util
    problems = []
    for mod in ["canny", "hough", "houghP"]:
        want = os.path.join(PKG_SRC, mod + ".py")
        try:
            spec = importlib.util.find_spec("geometric_features." + mod)
        except ModuleNotFoundError:
            spec = None
        if spec is None or spec.origin is None:
            return ["No encuentro el paquete geometric_features: corre este script con run_experiments.sh "
                    "(compila el paquete y hace source del workspace)."]
        got = os.path.realpath(spec.origin)
        if got != os.path.realpath(want) and not filecmp.cmp(got, want, shallow=False):
            problems.append("%s.py: ROS usa %s\n      pero el repo tiene %s\n      -> corre run_experiments.sh "
                            "desde ESTE repo (compila y hace source de este workspace)." % (mod, got, want))
        with open(want) as f:
            if "log_dir" not in f.read():
                problems.append("%s no tiene el modo experimento (log_dir)." % want)
    return problems


def preflight(wait_s=8.0, q_brazo=None):
    """Pone la camara en la pose pedida, revisa que publique, mide su frecuencia y el factor de
    tiempo real (/clock), y busca detectores ya abiertos (competirian por CPU y falsearian los
    tiempos). Devuelve (info, vista): vista es el ultimo cuadro de la camara (BGR) o None."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock

    info = {"camara": None, "fps": None, "rtf": None, "detectores_abiertos": [], "brazo": None}
    vista = None
    rclpy.init()
    node = rclpy.create_node("t10_preflight")
    try:
        if q_brazo is not None:
            b = info["brazo"] = t10_camara.mover_brazo(node, q_brazo, log=lambda txt: print(txt, flush=True))
            if not b["alcanzada"]:
                return info, None
            print("   brazo listo (error %.3f rad): camara a %.2f m, inclinacion %+.1f grados"
                  % (b["error_max_rad"], b["camara_xyz_m"][2], b["camara_inclinacion_grados"]), flush=True)
        print(">> Revisando la camara, el reloj de simulacion y los nodos abiertos (%.0f s)..." % wait_s, flush=True)
        cam = {t: [] for t in CAM_TOPICS}
        last = {}
        clk = []

        def on_image(m, t):
            cam[t].append((time.time(), m.width, m.height, m.encoding))
            last[t] = m

        for t in CAM_TOPICS:
            node.create_subscription(Image, t, lambda m, t=t: on_image(m, t), 1)
        node.create_subscription(Clock, "/clock",
                                 lambda m: clk.append((time.time(), m.clock.sec + 1e-9 * m.clock.nanosec)),
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        t_end = time.time() + wait_s
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        names = node.get_node_names()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    live = [t for t in CAM_TOPICS if len(cam[t]) >= 3]
    if live:
        t = live[0]
        s = cam[t]
        info["camara"] = {"topico": t, "ancho": s[-1][1], "alto": s[-1][2], "encoding": s[-1][3],
                          "mensajes": len(s), "ambos_topicos": len(live) > 1}
        info["fps"] = (len(s) - 1) / max(s[-1][0] - s[0][0], 1e-6)
        try:
            from cv_bridge import CvBridge
            vista = CvBridge().imgmsg_to_cv2(last[t], desired_encoding="bgr8")
        except Exception:
            vista = None
    if len(clk) >= 2 and clk[-1][0] > clk[0][0]:
        info["rtf"] = (clk[-1][1] - clk[0][1]) / (clk[-1][0] - clk[0][0])
    info["detectores_abiertos"] = sorted(set(names) & set(NODE_NAMES.values()))
    return info, vista


def git_info():
    def run(*cmd):
        try:
            return subprocess.run(["git", "-C", WS] + list(cmd), capture_output=True, text=True,
                                  timeout=10).stdout.strip()
        except Exception:
            return ""
    return {"commit": run("rev-parse", "--short", "HEAD"), "rama": run("rev-parse", "--abbrev-ref", "HEAD"),
            "cambios_sin_commit": run("status", "--porcelain", "--", PKG_SRC) != ""}


def system_info(pre, args):
    info = {"fecha": datetime.now().isoformat(timespec="seconds"), "host": platform.node(),
            "python": platform.python_version(), "kernel": platform.release(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""), "nucleos": os.cpu_count(),
            "frames_hough": args.frames, "frames_canny": FRAMES_CANNY}
    try:
        import cv2, numpy
        info["opencv"], info["numpy"] = cv2.__version__, numpy.__version__
    except Exception:
        pass
    try:
        with open("/proc/cpuinfo") as f:
            info["cpu"] = next((l.split(":", 1)[1].strip() for l in f if l.startswith("model name")), "")
    except OSError:
        pass
    try:
        with open("/etc/os-release") as f:
            info["so"] = next((l.split("=", 1)[1].strip().strip('"') for l in f if l.startswith("PRETTY_NAME")), "")
    except OSError:
        pass
    info.update(pre)
    info["git"] = git_info()
    return info


# ----------------------------------------------------------------------------------------------
def stop(proc):
    """Ctrl+C al grupo de procesos (ros2 run + nodo); si no responde, SIGTERM y luego SIGKILL."""
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            pass


def quick_stats(run_dir, nodo):
    """Resumen de una linea para la consola (mediana del tiempo y de las lineas)."""
    try:
        with open(os.path.join(run_dir, "frames.csv")) as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return 0, ""
    if not rows:
        return 0, ""
    def med(key):
        v = sorted(float(r[key]) for r in rows)
        return v[len(v) // 2]
    if nodo == "canny":
        return len(rows), "bordes=%.2f%%" % (100.0 * med("edge_px") / (640 * 480))
    return len(rows), "t_med=%.3f ms  %s=%d" % (med("t_ms"), "lineas" if nodo == "hough" else "segmentos",
                                                med("lines"))


def run_one(c, out_dir, n_frames, fps, k, n_total, eta_txt=""):
    run_dir = os.path.join(out_dir, "%s_%s" % (c["id"], c["nodo"]))
    if os.path.isdir(run_dir):
        shutil.rmtree(run_dir)          # corrida incompleta de un intento anterior
    os.makedirs(run_dir)
    cmd = (["ros2", "run", "geometric_features", c["nodo"], "--ros-args"] + ros_args(c["params"])
           + ["-p", "log_dir:=%s" % run_dir, "-p", "n_frames:=%d" % n_frames])
    timeout = 60.0 + 3.0 * n_frames / max(fps, 0.5)
    t0 = time.time()
    started = datetime.now().isoformat(timespec="seconds")
    with open(os.path.join(run_dir, "node.log"), "w") as log:
        log.write("$ " + " ".join(cmd) + "\n\n")
        log.flush()
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while True:
                if os.path.exists(os.path.join(run_dir, "done")):
                    estado = "ok"
                    break
                if proc.poll() is not None:
                    estado = "fallo"
                    break
                if time.time() - t0 > timeout:
                    estado = "timeout"
                    break
                time.sleep(0.25)
        finally:
            stop(proc)
    dur = time.time() - t0
    filas, resumen = quick_stats(run_dir, c["nodo"])
    print("[%2d/%d] %-4s %-7s %-11s %-7s %4d cuadros %5.1f s  %s%s"
          % (k, n_total, c["id"], c["nodo"], c["grupo"], estado, filas, dur, resumen, eta_txt(dur) if eta_txt else ""),
          flush=True)
    if estado != "ok":
        with open(os.path.join(run_dir, "node.log"), errors="replace") as f:
            tail = f.read().splitlines()[-12:]
        print("        ultimas lineas de %s/node.log:" % os.path.basename(run_dir))
        for line in tail:
            print("        | " + line)
    row = {"id": c["id"], "nodo": c["nodo"], "grupo": c["grupo"], "n_frames": n_frames, "estado": estado,
           "filas": filas, "inicio": started, "duracion_s": "%.1f" % dur, "carpeta": os.path.basename(run_dir)}
    for p in PARAM_COLS:
        row[p] = fmt_val(c["params"][p]) if p in c["params"] else ""
    return row


def write_runs(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=RUN_COLS)
        w.writeheader()
        w.writerows(rows)


# ----------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Experimentos automaticos de la Tarea 10")
    ap.add_argument("-n", action="store_true", help="solo muestra el plan")
    ap.add_argument("-m", type=int, default=0, metavar="MAX", help="corre solo las primeras MAX corridas")
    ap.add_argument("--only", default="", help="ids o nodos separados por coma (p. ej. hough o HB,PB)")
    ap.add_argument("--frames", type=int, default=100, help="cuadros por corrida de hough/houghP")
    ap.add_argument("-o", default=os.path.join(HERE, "..", "datos"), metavar="DIR", help="carpeta de datos")
    ap.add_argument("--continuar", nargs="?", const="ultimo", default=None, metavar="LOTE",
                    help="reanuda un lote (el ultimo si no se indica) y salta las corridas terminadas")
    ap.add_argument("--pose", default="frente", choices=["frente", "alta", "inicial", "ninguna"],
                    help="pose del brazo antes de las pruebas (default frente: camara horizontal)")
    ap.add_argument("--inclinacion", type=float, default=0.0,
                    help="inclinacion de la camara en grados (positivo = hacia arriba)")
    args = ap.parse_args()
    q_brazo = None
    if args.pose != "ninguna":
        try:
            q_brazo = t10_camara.pose_joints(args.pose, args.inclinacion)
        except ValueError as e:
            sys.exit("!! " + str(e))

    plan = CONFIGS
    if args.only:
        sel = [s.strip() for s in args.only.split(",") if s.strip()]
        plan = [c for c in plan if c["id"] in sel or c["nodo"] in sel]
        if not plan:
            sys.exit("--only %s no coincide con ningun id ni nodo" % args.only)
    if args.m > 0:
        plan = plan[:args.m]
    if args.n:
        print_plan(plan, args, q_brazo=q_brazo)
        return

    # 1) codigo correcto y simulacion corriendo
    problems = check_package()
    if problems:
        print("\n!! Problema con el paquete geometric_features:\n  - " + "\n  - ".join(problems))
        sys.exit(2)
    if q_brazo is not None:
        pos, pitch = t10_camara.camara(q_brazo)
        print(">> Brazo a la pose '%s' (camara a %.2f m sobre la base, inclinacion %+.0f grados)"
              % (args.pose, pos[2], pitch), flush=True)
    pre, vista = preflight(q_brazo=q_brazo)
    if q_brazo is not None and not pre["brazo"]["alcanzada"]:
        print("\n!! No se pudo mover el brazo: %s" % pre["brazo"].get("error", "error desconocido"))
        print("   Revisa que la simulacion siga abierta. Para correr sin mover el brazo: "
              "bash run_experiments.sh --pose ninguna")
        sys.exit(2)
    if pre["camara"] is None:
        print("\n!! No llegan imagenes por %s.\n   Lanza primero la simulacion en otra terminal:\n"
              "   ros2 launch house_simul house_simul.launch.py realsense:=True" % " ni ".join(CAM_TOPICS))
        sys.exit(2)
    if pre["detectores_abiertos"]:
        print("\n!! Hay detectores abiertos: %s. Cierralos (Ctrl+C) y vuelve a correr el script."
              % ", ".join(pre["detectores_abiertos"]))
        sys.exit(2)
    cam = pre["camara"]
    print("   camara %s: %dx%d %s a %.1f Hz | factor de tiempo real: %s"
          % (cam["topico"], cam["ancho"], cam["alto"], cam["encoding"], pre["fps"],
             "%.2f" % pre["rtf"] if pre["rtf"] is not None else "sin /clock"))
    if cam["ambos_topicos"]:
        print("   aviso: llegan imagenes por los dos topicos; cada cuadro se procesaria dos veces.")

    # 2) carpeta del lote
    base = os.path.abspath(args.o)
    if args.continuar:
        lotes = sorted(d for d in glob.glob(os.path.join(base, "2*")) if os.path.isdir(d))
        if args.continuar == "ultimo":
            if not lotes:
                sys.exit("No hay lotes que continuar en %s" % base)
            out_dir = lotes[-1]
        else:
            out_dir = os.path.join(base, args.continuar)
            if not os.path.isdir(out_dir):
                sys.exit("No existe el lote %s" % out_dir)
    else:
        out_dir = os.path.join(base, datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    sysinfo = system_info(pre, args)
    sysinfo["pose_brazo"] = {"pose": args.pose, "inclinacion_grados": args.inclinacion}
    with open(os.path.join(out_dir, "sistema.json"), "w") as f:
        json.dump(sysinfo, f, indent=1, ensure_ascii=False)
    if vista is not None:
        import cv2
        cv2.imwrite(os.path.join(out_dir, "vista_inicial.png"), vista)
    snap = os.path.join(out_dir, "codigo")
    os.makedirs(snap, exist_ok=True)
    for mod in ["canny", "hough", "houghP"]:
        # copyfile (sin chmod): en discos exFAT/NTFS cambiar permisos puede fallar
        shutil.copyfile(os.path.join(PKG_SRC, mod + ".py"), os.path.join(snap, mod + ".py"))

    runs_csv = os.path.join(out_dir, "corridas.csv")
    rows = {}
    if os.path.exists(runs_csv):
        with open(runs_csv) as f:
            rows = {r["id"]: r for r in csv.DictReader(f)}

    print_plan(plan, args, fps=pre["fps"], q_brazo=q_brazo)
    print("Datos en: %s\n" % out_dir)
    t_start = time.time()
    overheads = []

    def eta_after(k):
        def txt(dur):
            n = frames_for(plan[k - 1], args)
            overheads.append(max(dur - n / pre["fps"], 0.0))
            ov = sum(overheads) / len(overheads)
            rest = sum(frames_for(c, args) / pre["fps"] + ov for c in plan[k:])
            return "  | faltan ~%.0f min" % (rest / 60.0) if k < len(plan) else ""
        return txt

    try:
        for k, c in enumerate(plan, 1):
            run_dir = os.path.join(out_dir, "%s_%s" % (c["id"], c["nodo"]))
            if args.continuar and os.path.exists(os.path.join(run_dir, "done")):
                print("[%2d/%d] %-4s ya estaba terminada" % (k, len(plan), c["id"]))
                continue
            rows[c["id"]] = run_one(c, out_dir, frames_for(c, args), pre["fps"], k, len(plan), eta_after(k))
            write_runs(runs_csv, [rows[x["id"]] for x in CONFIGS if x["id"] in rows])
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n!! Interrumpido. Para reanudar: bash run_experiments.sh --continuar")
        sys.exit(130)

    ok = sum(1 for r in rows.values() if r["estado"] == "ok")
    print("\nListo: %d/%d corridas ok en %.1f min." % (ok, len(rows), (time.time() - t_start) / 60.0))
    print("Datos: %s" % out_dir)
    if ok < len(rows):
        print("Las que fallaron se pueden repetir con: bash run_experiments.sh --continuar")


if __name__ == "__main__":
    main()
