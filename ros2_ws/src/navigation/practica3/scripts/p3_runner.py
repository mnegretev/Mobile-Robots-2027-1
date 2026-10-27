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

Uso (a traves de run_experiments.sh, que compila el particle_filter de ESTE repo y hace source):
  bash run_experiments.sh [-n] [-r REPS] [-m MAX] [--only B,N1,...] [-o DIR] [--continuar [LOTE]]
  -n            solo muestra el plan
  -r REPS       repeticiones por configuracion (default 2)
  -m MAX        corre a lo mas MAX corridas (prueba de humo; con --continuar, de las pendientes)
  --only        solo esas configuraciones (ids separados por coma)
  -o DIR        carpeta de salida (default: ../datos junto a este script)
  --continuar   reanuda un lote (el ultimo de runs.csv si no se indica): usa el plan guardado del
                lote, salta las corridas que ya terminaron bien y repite las demas
Antes de mover al robot revisa que el `pf` que se va a correr se haya compilado desde ESTE repo.
Ctrl+C detiene el filtro y el robot, anota la corrida como "interrumpida" y dice como reanudar.

Salida en DIR:
  <lote>_<id>_r<rep>.log   salida del nodo pf en cada corrida que termino bien (o en curso)
  <lote>_<id>_r<rep>.<estado>.log   intentos que fallaron (no se pisan al repetir)
  runs.csv                 una fila por intento (parametros, log, inicio, fin, iteraciones, estado)
  <lote>_sistema.json      equipo, git, binario pf usado, factor de tiempo real y plan del lote
  <lote>_codigo/           copia del codigo de particle_filter con el que corrio el lote
"""
import argparse, csv, filecmp, json, math, os, platform, re, shutil, signal, subprocess, sys, time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.srv import GetMap
from rosgraph_msgs.msg import Clock

HERE    = os.path.dirname(os.path.abspath(__file__))
WS      = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))        # .../ros2_ws
PKG_DIR = os.path.join(WS, "src", "navigation", "particle_filter")
CODE    = ["src", "include", "CMakeLists.txt", "package.xml"]   # lo que se compila (y se copia)

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
MIN_POR_CORRIDA = 1.6        # duracion aproximada de una corrida (para la estimacion)
RUN_COLS = ["run_id", "config", "group", "N", "s2s", "s2m", "s2r", "rep", "log",
            "start", "end", "updates", "status"]
LOTE_RE  = re.compile(r"^(\d{8}_\d{6})_")


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

    def measure_rtf(self, dt=2.0):
        """Factor de tiempo real (avance de /clock entre tiempo de pared). La suscripcion se
        quita al terminar: /clock llega a ~1 kHz y no debe cargar el CPU durante las corridas."""
        clk = []
        sub = self.create_subscription(
            Clock, "/clock", lambda m: clk.append((time.time(), m.clock.sec + 1e-9 * m.clock.nanosec)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.spin_for(dt)
        self.destroy_subscription(sub)
        if len(clk) >= 2 and clk[-1][0] - clk[0][0] > 0.2:
            return (clk[-1][1] - clk[0][1]) / (clk[-1][0] - clk[0][0])
        return None

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
        if not self.logf.closed:
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


# ----------------------------------------------------------------------------------------------
# Que `pf` se va a correr: debe estar compilado desde ESTE repo (no desde el workspace viejo).
def find_pf():
    """Prefijo y ejecutable del particle_filter que ve ROS (el primero de AMENT_PREFIX_PATH)."""
    try:
        prefix = subprocess.run(["ros2", "pkg", "prefix", "particle_filter"], capture_output=True,
                                text=True, timeout=60).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        prefix = ""
    if not prefix:
        return None, None
    return prefix, os.path.join(prefix, "lib", "particle_filter", "pf")


def cmake_source(exe, prefix):
    """(carpeta fuente, CMakeCache.txt) del build que genero ese pf, o (None, None)."""
    cands = []
    real = os.path.realpath(exe)
    if real != os.path.abspath(exe):                     # --symlink-install: pf -> build/pf
        cands.append(os.path.dirname(real))
    p = os.path.realpath(prefix)
    cands += [os.path.join(os.path.dirname(os.path.dirname(p)), "build", "particle_filter"),  # <ws>/install/pkg
              os.path.join(os.path.dirname(p), "build", "particle_filter")]                   # --merge-install
    for d in cands:
        cache = os.path.join(d, "CMakeCache.txt")
        try:
            with open(cache, errors="replace") as f:
                for line in f:
                    if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
                        return line.split("=", 1)[1].strip(), cache
        except OSError:
            continue
    return None, None


def same_dir(a, b):
    try:
        return os.path.samefile(a, b)
    except OSError:
        return os.path.realpath(a) == os.path.realpath(b)


def code_files(root):
    """Rutas relativas de los archivos que se compilan (src/, include/, CMakeLists.txt, package.xml)."""
    out = []
    for item in CODE:
        p = os.path.join(root, item)
        if os.path.isfile(p):
            out.append(item)
        elif os.path.isdir(p):
            for dirpath, dirs, files in os.walk(p):
                dirs[:] = sorted(d for d in dirs if d != "__pycache__")
                for fn in sorted(files):
                    out.append(os.path.relpath(os.path.join(dirpath, fn), root))
    return out


def check_pf(prefix, exe):
    """Revisa que el pf exista, se haya compilado desde ESTE repo y no sea mas viejo que el codigo.
    Regresa (problemas, avisos, info)."""
    problems, warnings = [], []
    info = {"prefijo": prefix, "ejecutable": exe}
    if prefix is None:
        return (["ROS no encuentra el paquete particle_filter: corre este script con run_experiments.sh "
                 "(compila el paquete de este repo y hace source de el)."], warnings, info)
    if not os.path.isfile(exe):
        return (["No existe %s: falta compilar particle_filter (run_experiments.sh lo hace)." % exe],
                warnings, info)
    expected = os.environ.get("P3_BUILD_DIR")
    if expected and not same_dir(prefix, os.path.join(expected, "install", "particle_filter")):
        problems.append("ros2 usa %s\n      y run_experiments.sh lo compilo en %s/install/particle_filter"
                        % (prefix, expected))
    src, cache = cmake_source(exe, prefix)
    info.update(compilado_desde=src, cmake_cache=cache)
    if src is None:
        warnings.append("no encontre el CMakeCache.txt de %s: no puedo confirmar desde que carpeta se "
                        "compilo." % exe)
    elif not same_dir(src, PKG_DIR):
        problems.append("el pf que correria ros2 se compilo desde\n      %s\n      pero este repo es\n"
                        "      %s\n      -> corre run_experiments.sh desde ESTE repo (compila particle_filter "
                        "y hace source de el)." % (src, PKG_DIR))
    t_bin = os.path.getmtime(os.path.realpath(exe))
    newest, newest_f = 0.0, ""
    for rel in code_files(PKG_DIR):
        m = os.path.getmtime(os.path.join(PKG_DIR, rel))
        if m > newest:
            newest, newest_f = m, rel
    info.update(binario_fecha=datetime.fromtimestamp(t_bin).isoformat(timespec="seconds"),
                codigo_fecha=datetime.fromtimestamp(newest).isoformat(timespec="seconds") if newest else None)
    if newest > t_bin + 1.0:
        msg = ("%s es mas nuevo que el pf compilado (%s): el filtro correria codigo viejo."
               % (newest_f, info["binario_fecha"]))
        if expected:            # run_experiments.sh acaba de compilar: probablemente solo es la fecha
            warnings.append(msg)
        else:
            problems.append(msg + "\n      -> compila con run_experiments.sh.")
    return problems, warnings, info


# ----------------------------------------------------------------------------------------------
# Registro del lote: runs.csv, sistema.json y copia del codigo
def read_runs(path):
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def last_batch(rows):
    lotes = [m.group(1) for m in (LOTE_RE.match(r.get("run_id", "")) for r in rows) if m]
    return lotes[-1] if lotes else None


def load_plan(path):
    """Plan [(config, rep), ...] guardado en <lote>_sistema.json, o None."""
    try:
        with open(path) as f:
            saved = json.load(f).get("plan")
        by_id = {c[0]: c for c in CONFIGS}
        return [(by_id[cid], int(rep)) for cid, rep in saved]
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return None


def free_name(out, base, tag, ext=".log"):
    """<base>.<tag><ext> sin pisar uno que ya exista (<base>.<tag>2<ext>, ...)."""
    name, k = "%s.%s%s" % (base, tag, ext), 1
    while os.path.exists(os.path.join(out, name)):
        k += 1
        name = "%s.%s%d%s" % (base, tag, k, ext)
    return name


def snapshot_code(dst):
    for rel in code_files(PKG_DIR):
        os.makedirs(os.path.dirname(os.path.join(dst, rel)), exist_ok=True)
        # copyfile (sin chmod): en discos exFAT/NTFS cambiar permisos puede fallar
        shutil.copyfile(os.path.join(PKG_DIR, rel), os.path.join(dst, rel))


def code_changes(snap):
    """Archivos de particle_filter que difieren de la copia del lote."""
    now, old = set(code_files(PKG_DIR)), set(code_files(snap))
    diff = sorted(now ^ old)
    diff += [r for r in sorted(now & old)
             if not filecmp.cmp(os.path.join(PKG_DIR, r), os.path.join(snap, r), shallow=False)]
    return diff


def git_info():
    env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")       # solo lectura: no toma el lock del indice
    def run(*cmd):
        try:
            return subprocess.run(["git", "-C", WS] + list(cmd), capture_output=True, text=True,
                                  timeout=10, env=env).stdout.strip()
        except Exception:
            return ""
    return {"commit": run("rev-parse", "--short", "HEAD"), "rama": run("rev-parse", "--abbrev-ref", "HEAD"),
            "cambios_sin_commit_particle_filter": run("status", "--porcelain", "--", PKG_DIR) != ""}


def system_info(pf_info, rtf, plan, lote, continuando):
    info = {"lote": lote, "fecha": datetime.now().isoformat(timespec="seconds"), "continuacion": continuando,
            "host": platform.node(), "python": platform.python_version(), "kernel": platform.release(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""), "nucleos": os.cpu_count()}
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
    info["factor_tiempo_real"] = round(rtf, 3) if rtf is not None else None
    info["repo"] = WS
    info["git"] = git_info()
    info["pf"] = pf_info
    info["plan"] = [[c[0], r] for c, r in plan]
    return info


# ----------------------------------------------------------------------------------------------
def init_ros():
    """Sin los manejadores de senales de rclpy: Ctrl+C llega como KeyboardInterrupt y el nodo
    sigue vivo para frenar al robot antes de salir."""
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", action="store_true", help="solo mostrar el plan")
    ap.add_argument("-r", type=int, default=2, help="repeticiones por configuracion")
    ap.add_argument("-m", type=int, default=0, help="maximo de corridas (prueba de humo)")
    ap.add_argument("--only", default="", help="ids de configuracion separados por coma")
    ap.add_argument("-o", default="", help="carpeta de salida")
    ap.add_argument("--continuar", nargs="?", const="ultimo", default=None, metavar="LOTE",
                    help="reanuda un lote (el ultimo si no se indica) y salta las corridas terminadas")
    args = ap.parse_args()

    out = os.path.abspath(args.o or os.path.join(HERE, "..", "datos"))
    runs_csv = os.path.join(out, "runs.csv")
    only = [s.strip() for s in args.only.split(",") if s.strip()]
    unknown = [s for s in only if s not in {c[0] for c in CONFIGS}]
    if unknown:
        print("--only: no existen las configuraciones %s" % ",".join(unknown))
        return 2

    # --- plan: lote nuevo, o el plan guardado del lote que se reanuda
    prev = read_runs(runs_csv)
    if args.continuar:
        batch = last_batch(prev) if args.continuar == "ultimo" else args.continuar
        if not batch:
            print("No hay lotes que continuar en %s" % runs_csv)
            return 2
        known = {m.group(1) for m in (LOTE_RE.match(r.get("run_id", "")) for r in prev) if m}
        if batch not in known and not os.path.exists(os.path.join(out, "%s_sistema.json" % batch)):
            print("No existe el lote %s en %s (lotes: %s)" % (batch, runs_csv, ", ".join(sorted(known)) or "ninguno"))
            return 2
        plan = load_plan(os.path.join(out, "%s_sistema.json" % batch))
        if plan is None:
            plan = [(c, r) for r in range(1, args.r + 1) for c in CONFIGS]
            print("Aviso: el lote %s no tiene plan guardado; uso %d configuraciones x %d rep." %
                  (batch, len(CONFIGS), args.r))
        done = {r["run_id"] for r in prev if r.get("status") == "ok"}
    else:
        batch = datetime.now().strftime("%Y%m%d_%H%M%S")
        plan = [(c, r) for r in range(1, args.r + 1) for c in CONFIGS]     # repeticion por bloques
        done = set()
    if only:
        plan = [(c, r) for c, r in plan if c[0] in only]
    todo = [(c, r) for c, r in plan if "%s_%s_r%d" % (batch, c[0], r) not in done]
    if args.m:
        todo = todo[:args.m]

    eta = len(todo) * MIN_POR_CORRIDA
    print(f"Lote {batch}{' (continuacion)' if args.continuar else ''}: {len(todo)} corridas por hacer"
          f"{' de %d' % len(plan) if args.continuar else ''}, ~{eta:.0f} min. Salida: {out}")
    for c, r in plan:
        if (c, r) in todo:
            mark = ""
        elif "%s_%s_r%d" % (batch, c[0], r) in done:
            mark = "  (ya terminada)"
        else:
            continue
        print(f"  {c[0]:>3} rep {r}:  N={c[1]:<5} s2s={c[2]:<6} s2m={c[3]:<6} s2r={c[4]:<6} ({c[5]}){mark}")
    if args.n:
        return 0
    if not todo:
        print("No hay corridas pendientes en el lote %s." % batch)
        return 0

    def log(msg):
        print(msg, flush=True)

    # --- el pf que se va a correr debe ser el de este repo
    prefix, exe = find_pf()
    problems, warnings, pf_info = check_pf(prefix, exe)
    for w_ in warnings:
        log("Aviso: " + w_)
    if problems:
        log("\n!! Problema con particle_filter:\n  - " + "\n  - ".join(problems))
        return 2
    log(f"pf: {exe}\n    compilado desde {pf_info.get('compilado_desde')}")

    # --- si se reanuda, el codigo debe ser el mismo con el que empezo el lote
    os.makedirs(out, exist_ok=True)
    snap = os.path.join(out, "%s_codigo" % batch)
    if args.continuar:
        if os.path.isdir(snap):
            changed = code_changes(snap)
            if changed:
                log("\n!! El codigo de particle_filter cambio desde que empezo el lote %s:\n     %s\n"
                    "   Empieza un lote nuevo (sin --continuar) o regresa el codigo a como estaba."
                    % (batch, "\n     ".join(changed)))
                return 2
        else:
            log("Aviso: el lote %s no tiene copia del codigo (%s_codigo/): no puedo confirmar que el "
                "filtro sea el mismo." % (batch, batch))

    init_ros()
    node = Driver()
    try:
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
        rtf = node.measure_rtf()
        log("Factor de tiempo real de la simulacion: " + ("%.2f" % rtf if rtf is not None else "sin /clock"))

        # --- registro del lote
        info = system_info(pf_info, rtf, plan, batch, bool(args.continuar))
        sis = os.path.join(out, "%s_sistema.json" % batch)
        if os.path.exists(sis):                         # continuacion: no se pisa el del inicio
            sis = os.path.join(out, free_name(out, "%s_sistema" % batch, "continuar", ".json"))
        with open(sis, "w") as f:
            json.dump(info, f, indent=1, ensure_ascii=False)
        if not os.path.isdir(snap) and not args.continuar:
            snapshot_code(snap)
        return run_batch(node, exe, batch, todo, out, runs_csv, log)
    except KeyboardInterrupt:                           # Ctrl+C antes de lanzar algun filtro
        log("\n!! Interrumpido (Ctrl+C).")
        return 130
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


def run_batch(node, exe, batch, todo, out, runs_csv, log):
    new_csv = not os.path.exists(runs_csv)
    with open(runs_csv, "a", newline="") as fcsv:
        w = csv.writer(fcsv)
        if new_csv:
            w.writerow(RUN_COLS)

        def record(cfg, rep, run_id, log_name, start, n_upd, status):
            cid, n, s2s, s2m, s2r, grp = cfg
            w.writerow([run_id, cid, grp, n, s2s, s2m, s2r, rep, log_name,
                        start, datetime.now().isoformat(timespec="seconds"), n_upd, status])
            fcsv.flush()

        pf, cur = None, None
        try:
            for i, (cfg, rep) in enumerate(todo, 1):
                cid, n, s2s, s2m, s2r, grp = cfg
                run_id = f"{batch}_{cid}_r{rep}"
                log_name = f"{run_id}.log"
                log_path = os.path.join(out, log_name)
                if os.path.exists(log_path):            # intento que se corto sin registrarse
                    os.rename(log_path, os.path.join(out, free_name(out, run_id, "incompleta")))
                log(f"[{i}/{len(todo)}] {cid} rep {rep}: N={n} s2s={s2s} s2m={s2m} s2r={s2r}")
                if not node.go_home():
                    log("    !! no pude regresar al origen; detengo los experimentos.")
                    break
                start = datetime.now().isoformat(timespec="seconds")
                pf = PF(exe, cfg, log_path)
                cur = (cfg, rep, run_id, start)
                status = "ok"
                if not pf.wait_ready(node, READY_TIMEOUT):
                    status = "no_arranco"
                    log("    !! el filtro no arranco (revisa el log)")
                elif not sequence(node, pf, log):
                    status = "murio"
                node.stop()
                pf.stop()
                pf, cur = None, None
                with open(log_path, errors="replace") as f:
                    n_upd, loc, ed = quick_summary(f.read())
                if status != "ok":                      # el log del intento fallido no se pisa
                    log_name = free_name(out, run_id, status)
                    os.rename(log_path, os.path.join(out, log_name))
                log(f"    {n_upd} iteraciones; localizado en la iteracion {loc if loc else '-'}; "
                    f"error final medio {ed:.2f} m")
                record(cfg, rep, run_id, log_name, start, n_upd, status)
        except KeyboardInterrupt:
            log("\n!! Interrumpido (Ctrl+C): detengo el filtro y el robot...")
            try:
                node.stop()
            except Exception:
                pass
            if pf is not None:
                pf.stop()
                cfg, rep, run_id, start = cur
                n_upd = pf.updates()
                log_name = free_name(out, run_id, "interrumpida")
                os.rename(pf.log_path, os.path.join(out, log_name))
                record(cfg, rep, run_id, log_name, start, n_upd, "interrumpida")
            log(f"Para reanudar el lote: bash run_experiments.sh --continuar {batch}")
            return 130
    node.stop()
    node.go_home()
    log(f"Listo. Datos en {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
