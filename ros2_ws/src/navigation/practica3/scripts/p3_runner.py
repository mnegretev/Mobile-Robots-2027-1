#!/usr/bin/env python3
"""
p3_runner.py - Practica 3 (localizacion por filtro de particulas), Robots Moviles FI-UNAM 2027-1
Omar Rosario Carrasco.

Corre automaticamente los experimentos del paso 13 de la practica. Para cada combinacion de
parametros (N, s2s, s2m, s2r) y cada repeticion:
  1. lleva al robot al origen (0, 0, 0) usando la pose real de Gazebo (lazo cerrado),
  2. lanza el nodo `pf` con esos parametros y guarda su salida en un log,
  3. mueve al robot SIEMPRE con la misma secuencia de desplazamientos pequenos por /cmd_vel
     (como la GUI): 1.5 m hacia +x en pasos de 0.25 m, giro de 180 grados en pasos de 15 grados,
     regreso al origen y otro giro de 180 grados,
  4. detiene el filtro y registra la corrida en runs.csv.

Requisitos (cada uno en su propia terminal bash de la distrobox):
  T1: ros2 launch house_simul house_simul.launch.py
  T2: ros2 launch navig_utils localization.launch.py
  y NINGUN otro nodo `pf` corriendo.

Uso:  python3 p3_runner.py [-n] [-r REPS] [-m MAX] [--only B,N1,...] [-o DIR]
  -n        solo muestra el plan
  -r REPS   repeticiones por configuracion (default 2)
  -m MAX    corre solo las primeras MAX corridas (prueba de humo)
  --only    solo esas configuraciones (ids separados por coma)
  -o DIR    carpeta de salida (default: ../datos junto a este script)
"""
import argparse, csv, math, os, re, signal, subprocess, sys, time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.srv import GetMap

# ----------------------------------------------------------------------------------------------
# Configuraciones: base = la que funciono (N=1000, s2s=0.1, s2m=0.01, s2r=0.01); se varia un
# parametro a la vez. D0 es el comando que trae la practica.
#   id     N     s2s    s2m    s2r    grupo
CONFIGS = [
    ("B",  1000, 0.1,   0.01,  0.01,  "base"),
    ("N1",  100, 0.1,   0.01,  0.01,  "N"),
    ("N2",  300, 0.1,   0.01,  0.01,  "N"),
    ("N3", 3000, 0.1,   0.01,  0.01,  "N"),
    ("S1", 1000, 0.01,  0.01,  0.01,  "s2s"),
    ("S2", 1000, 0.5,   0.01,  0.01,  "s2s"),
    ("S3", 1000, 1.0,   0.01,  0.01,  "s2s"),
    ("M1", 1000, 0.1,   0.001, 0.01,  "s2m"),
    ("M2", 1000, 0.1,   0.1,   0.01,  "s2m"),
    ("M3", 1000, 0.1,   1.0,   0.01,  "s2m"),
    ("R1", 1000, 0.1,   0.01,  0.001, "s2r"),
    ("R2", 1000, 0.1,   0.01,  0.1,   "s2r"),
    ("R3", 1000, 0.1,   0.01,  1.0,   "s2r"),
    ("D0",  100, 1.0,   1.0,   1.0,   "practica"),
]

GT_TOPIC   = "/gazebo/diff_base_with_xarm/pose"
STEP_D     = 0.25            # m por pulso de avance  (> umbral de 0.2 m del filtro)
STEP_A     = math.pi / 12    # rad por pulso de giro   (> umbral de 0.2 rad del filtro)
LEG        = 1.5             # m de cada tramo recto
V_MAX      = 0.25            # m/s
W_MAX      = 0.5             # rad/s
SETTLE     = 0.8             # s de pausa tras cada pulso (el filtro tarda < 0.2 s por iteracion)
READY_TIMEOUT = 90           # s para que el filtro publique la distribucion inicial


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fnum(x):
    """Flotante con punto decimal (ROS 2 rechaza '1' para un parametro double)."""
    return repr(float(x))


class Driver(Node):
    def __init__(self):
        super().__init__("p3_driver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseArray, GT_TOPIC, self._cb, 10)
        self.pose = None
        self.t_pose = 0.0

    def _cb(self, msg):
        if msg.poses:
            p = msg.poses[0]
            self.pose = (p.position.x, p.position.y, wrap(2 * math.atan2(p.orientation.z, p.orientation.w)))
            self.t_pose = time.time()

    def spin_for(self, dt):
        t_end = time.time() + dt
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def stop(self):
        for _ in range(4):
            self.cmd(0.0, 0.0)
            self.spin_for(0.05)

    def wait_pose(self, timeout=15.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None and time.time() - self.t_pose < 1.0:
                return True
        return False

    def rotate_to(self, target, tol=0.02, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.05)
            e = wrap(target - self.pose[2])
            if abs(e) < tol:
                break
            w = max(-W_MAX, min(W_MAX, 1.5 * e))
            if abs(w) < 0.12:
                w = math.copysign(0.12, w)
            self.cmd(0.0, w)
        self.stop()
        return abs(wrap(target - self.pose[2])) < 3 * tol

    def forward_to(self, gx, gy, tol=0.03, timeout=40.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.05)
            x, y, a = self.pose
            dx, dy = gx - x, gy - y
            ahead = dx * math.cos(a) + dy * math.sin(a)      # distancia restante sobre el rumbo
            if math.hypot(dx, dy) < tol or ahead < tol:
                break
            e_a = wrap(math.atan2(dy, dx) - a) if math.hypot(dx, dy) > 0.1 else 0.0
            v = max(0.06, min(V_MAX, 0.8 * ahead))
            w = max(-W_MAX, min(W_MAX, 2.0 * e_a))
            self.cmd(v, w)
        self.stop()
        return math.hypot(gx - self.pose[0], gy - self.pose[1]) < 3 * tol

    def go_home(self):
        x, y, a = self.pose
        if math.hypot(x, y) > 0.05:
            ok = self.rotate_to(math.atan2(-y, -x)) and self.forward_to(0.0, 0.0)
            if not ok:
                return False
        return self.rotate_to(0.0)


class PF:
    """Proceso del nodo pf con su log."""

    def __init__(self, exe, cfg, log_path):
        _, n, s2s, s2m, s2r, _ = cfg
        args = [exe, "--ros-args", "-p", "use_sim_time:=True", "-p", f"N:={int(n)}",
                "-p", f"s2s:={fnum(s2s)}", "-p", f"s2m:={fnum(s2m)}", "-p", f"s2r:={fnum(s2r)}"]
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM="0")
        self.log_path = log_path
        self.logf = open(log_path, "w")
        self.proc = subprocess.Popen(args, stdout=self.logf, stderr=subprocess.STDOUT, env=env,
                                     start_new_session=True)

    def text(self):
        try:
            with open(self.log_path, errors="replace") as f:
                return f.read()
        except OSError:
            return ""

    def alive(self):
        return self.proc.poll() is None

    def wait_ready(self, node, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            node.spin_for(0.25)
            if "Publishing initial distribution" in self.text():
                return True
            if not self.alive():
                return False
        return False

    def updates(self):
        return self.text().count("Estimated pose")

    def stop(self):
        if self.alive():
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.logf.close()


def quick_summary(log_text, thr_d=0.3, thr_a=0.2, k=3):
    est, rows = None, []
    for line in log_text.splitlines():
        m = re.search(r"Estimated pose: x=(\S+)\s+y=(\S+)\s+a=(\S+)", line)
        if m:
            est = tuple(map(float, m.groups()))
            continue
        m = re.search(r"Groundtruth pose: x=(\S+)\s+y=(\S+)\s+a=(\S+)", line)
        if m and est:
            gt = tuple(map(float, m.groups()))
            rows.append((math.hypot(est[0] - gt[0], est[1] - gt[1]), abs(wrap(est[2] - gt[2]))))
            est = None
    loc = None
    for i in range(len(rows) - k + 1):
        if all(d < thr_d and a < thr_a for d, a in rows[i:i + k]):
            loc = i + 1
            break
    last = rows[-5:]
    ed = sum(r[0] for r in last) / len(last) if last else float("nan")
    return len(rows), loc, ed


def sequence(node, pf, log):
    """Secuencia fija (36 pulsos) partiendo de (0,0,0)."""
    steps = int(round(LEG / STEP_D))
    moves = ([("fwd", i * STEP_D) for i in range(1, steps + 1)] +          # ida: +x
             [("rot", wrap(k * STEP_A)) for k in range(1, 13)] +            # giro de 180 grados
             [("fwd", i * STEP_D) for i in range(steps - 1, -1, -1)] +      # regreso al origen
             [("rot", wrap(k * STEP_A)) for k in range(13, 25)])            # giro de 180 -> rumbo 0
    for kind, target in moves:
        if kind == "fwd":
            node.forward_to(target, 0.0)
        else:
            node.rotate_to(target)
        node.spin_for(SETTLE)
        if not pf.alive():
            log("    !! el filtro termino antes de tiempo")
            return False
    node.spin_for(1.0)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", action="store_true", help="solo mostrar el plan")
    ap.add_argument("-r", type=int, default=2, help="repeticiones por configuracion")
    ap.add_argument("-m", type=int, default=0, help="maximo de corridas (prueba de humo)")
    ap.add_argument("--only", default="", help="ids de configuracion separados por coma")
    ap.add_argument("-o", default="", help="carpeta de salida")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.abspath(args.o or os.path.join(here, "..", "datos"))
    cfgs = [c for c in CONFIGS if not args.only or c[0] in args.only.split(",")]
    plan = [(c, r) for r in range(1, args.r + 1) for c in cfgs]     # repeticion por bloques
    if args.m:
        plan = plan[:args.m]
    eta = len(plan) * 1.6
    print(f"Plan: {len(plan)} corridas ({len(cfgs)} configuraciones x {args.r} rep.), ~{eta:.0f} min. Salida: {out}")
    for c, r in plan:
        print(f"  {c[0]:>3} rep {r}:  N={c[1]:<5} s2s={c[2]:<6} s2m={c[3]:<6} s2r={c[4]:<6} ({c[5]})")
    if args.n:
        return 0

    os.makedirs(out, exist_ok=True)
    batch = datetime.now().strftime("%Y%m%d_%H%M%S")
    runs_csv = os.path.join(out, "runs.csv")
    new_csv = not os.path.exists(runs_csv)
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", "particle_filter"], text=True).strip()
    exe = os.path.join(prefix, "lib", "particle_filter", "pf")

    def log(msg):
        print(msg, flush=True)

    rclpy.init()
    node = Driver()

    # --- comprobaciones previas
    node.spin_for(1.5)                                  # da tiempo al descubrimiento de nodos
    names = [n for n, _ in node.get_node_names_and_namespaces()]
    if "particle_filter" in names:
        log("Hay un nodo particle_filter corriendo. Cierralo (Ctrl+C en su terminal) y vuelve a intentar.")
        return 1
    if not node.wait_pose():
        log(f"No llega la pose real en {GT_TOPIC}. Esta corriendo house_simul.launch.py?")
        return 1
    cli = node.create_client(GetMap, "/map_server/map")
    if not cli.wait_for_service(timeout_sec=10.0):
        log("No esta el servicio /map_server/map. Esta corriendo localization.launch.py?")
        return 1
    x, y, a = node.pose
    if math.hypot(x, y) > 2.5:
        log(f"El robot esta lejos del origen ({x:.2f}, {y:.2f}). Reinicia house_simul para que reaparezca en (0, 0).")
        return 1

    with open(runs_csv, "a", newline="") as fcsv:
        w = csv.writer(fcsv)
        if new_csv:
            w.writerow(["run_id", "config", "group", "N", "s2s", "s2m", "s2r", "rep", "log",
                        "start", "end", "updates", "status"])
        for i, (cfg, rep) in enumerate(plan, 1):
            cid, n, s2s, s2m, s2r, grp = cfg
            run_id = f"{batch}_{cid}_r{rep}"
            log_path = os.path.join(out, f"{run_id}.log")
            log(f"[{i}/{len(plan)}] {cid} rep {rep}: N={n} s2s={s2s} s2m={s2m} s2r={s2r}")
            if not node.go_home():
                log("    !! no pude regresar al origen; detengo los experimentos.")
                break
            start = datetime.now().isoformat(timespec="seconds")
            pf = PF(exe, cfg, log_path)
            status = "ok"
            if not pf.wait_ready(node, READY_TIMEOUT):
                status = "no_arranco"
                log("    !! el filtro no arranco (revisa el log)")
            elif not sequence(node, pf, log):
                status = "murio"
            node.stop()
            pf.stop()
            text = open(log_path, errors="replace").read()
            n_upd, loc, ed = quick_summary(text)
            log(f"    {n_upd} iteraciones; localizado en la iteracion {loc if loc else '-'}; "
                f"error final medio {ed:.2f} m")
            w.writerow([run_id, cid, grp, n, s2s, s2m, s2r, rep, os.path.basename(log_path),
                        start, datetime.now().isoformat(timespec="seconds"), n_upd, status])
            fcsv.flush()
    node.stop()
    node.go_home()
    node.destroy_node()
    rclpy.shutdown()
    log(f"Listo. Datos en {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
