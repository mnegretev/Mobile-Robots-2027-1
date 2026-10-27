#!/usr/bin/env python3
"""
t10_camara.py - Tarea 10, Robots Moviles FI-UNAM 2027-1. Omar Rosario Carrasco.

Mueve el brazo xArm6 para que la camara RealSense (montada en la muneca, link6) mire al frente.
En la pose de arranque (todas las articulaciones en 0) la camara apunta al piso.

Geometria (xarm6_default_kinematics.yaml y realsense_d435i.urdf.xacro): la camara mira a lo largo
del eje z de link6 y, con joint1 = joint4 = joint6 = 0, las articulaciones 2, 3 y 5 giran sobre ejes
horizontales paralelos, asi que la inclinacion de la camara solo depende de su suma:
        inclinacion = -(q2 + q3 + q5) - pi/2      (0 = horizontal, positiva = hacia arriba)
Para mirar al frente basta q5 = -pi/2 - (q2 + q3).

Uso (con la simulacion corriendo y el workspace con source):
  python3 t10_camara.py                    # pose "frente": camara horizontal a ~0.7 m sobre la base
  python3 t10_camara.py --pose alta        # camara horizontal a ~1.1 m (brazo levantado)
  python3 t10_camara.py --inclinacion 10   # 10 grados hacia arriba (negativo: hacia abajo)
  python3 t10_camara.py --pose inicial     # regresa a la pose de arranque (mira al piso)
  python3 t10_camara.py --foto vista.png   # ademas guarda lo que ve la camara al final
Lo usa t10_runner.py antes de las pruebas (opciones --pose y --inclinacion).
"""
import argparse, math, sys, time
import numpy as np

CONTROLLER = "xarm6_traj_controller"
TOPIC_TRAJ = "/%s/joint_trajectory" % CONTROLLER
TOPIC_STATES = "/joint_states"
CAM_TOPICS = ["/camera/color/image_raw", "/camera/image_raw"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LIMITS = [(-math.pi * 0.99, math.pi * 0.99), (-2.059, 2.0944), (-math.pi * 0.99, 0.19198),
          (-math.pi * 0.99, math.pi * 0.99), (-1.69297, math.pi * 0.99), (-math.pi * 0.99, math.pi * 0.99)]
# (q2, q3) de cada pose; q5 se calcula para la inclinacion pedida
POSES = {"frente": (0.0, 0.0), "alta": (-0.3, -1.2)}

# ---- cinematica directa (solo para reportar donde queda la camara) -------------------------------
_J = [((0, 0, 0.267), (0, 0, 0)), ((0, 0, 0), (-1.5708, 0, 0)), ((0.0535, -0.2845, 0), (0, 0, 0)),
      ((0.0775, 0.3425, 0), (-1.5708, 0, 0)), ((0, 0, 0), (1.5708, 0, 0)), ((0.076, 0.097, 0), (-1.5708, 0, 0))]


def _T(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    M = np.eye(4)
    M[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ \
        np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    M[:3, 3] = xyz
    return M


def camara(q, altura_brazo=0.35):
    """Posicion de la camara sobre base_link [m] e inclinacion [grados] para las articulaciones q."""
    M = _T((0, 0, altura_brazo), (0, 0, 0))
    for (xyz, rpy), qi in zip(_J, q):
        M = M @ _T(xyz, rpy) @ _T((0, 0, 0), (0, 0, qi))
    M = M @ _T((0.08746, -0.0175, 0.0237), (math.pi, -math.pi / 2, 0)) @ _T((0, 0.015, 0), (0, 0, 0))
    look = M[:3, 0]                    # Gazebo: la camara mira en +x de su marco
    return M[:3, 3], math.degrees(math.asin(max(-1.0, min(1.0, look[2]))))


def pose_joints(nombre, inclinacion_deg=0.0):
    """Articulaciones para la pose pedida. Si q5 se sale de su limite, el resto se pasa a q3."""
    if nombre == "inicial":
        return [0.0] * 6
    q2, q3 = POSES[nombre]
    phi = math.radians(inclinacion_deg)
    q5 = -math.pi / 2 - phi - (q2 + q3)
    lo5, hi5 = LIMITS[4][0] + 0.02, LIMITS[4][1] - 0.02
    if q5 < lo5:
        q3 -= lo5 - q5
        q5 = lo5
    elif q5 > hi5:
        q3 += q5 - hi5
        q5 = hi5
    q = [0.0, q2, q3, 0.0, q5, 0.0]
    for v, (lo, hi), name in zip(q, LIMITS, JOINTS):
        if not lo <= v <= hi:
            raise ValueError("la inclinacion %.1f grados deja %s fuera de su limite" % (inclinacion_deg, name))
    return q


# ---- ROS ---------------------------------------------------------------------------------------
def _list_controllers(node, cm, timeout=5.0):
    """[(nombre, estado, tipo)] de los controladores del controller_manager, o None si no responde."""
    import rclpy
    try:
        from controller_manager_msgs.srv import ListControllers
    except ImportError:
        return None
    cli = node.create_client(ListControllers, cm + "/list_controllers")
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if fut.result() is None:
            return None
        return [(c.name, c.state, c.type) for c in fut.result().controller]
    finally:
        node.destroy_client(cli)


def asegurar_controlador(node, log=print, timeout_spawner=90.0):
    """Busca el controlador de trayectoria del brazo en el controller_manager y, si no esta activo,
    lo carga y activa con el spawner (lo mismo que hace el launch al arrancar la simulacion).
    Devuelve (topico, info); topico es None si no se pudo."""
    import rclpy, subprocess
    info = {}
    cm, t0 = None, time.time()
    while cm is None and time.time() - t0 < 5.0:
        for name, _ in node.get_service_names_and_types():
            if name.endswith("/list_controllers"):
                cm = name[:-len("/list_controllers")]
                break
        if cm is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    cm = cm or "/controller_manager"
    ns = cm[:-len("/controller_manager")] if cm.endswith("/controller_manager") else ""
    info["controller_manager"] = cm
    ctrls = _list_controllers(node, cm)
    info["controladores"] = ctrls
    if ctrls is None:
        info["error"] = "no responde %s/list_controllers (la simulacion no esta corriendo o no carga ros2_control)" % cm
        return None, info

    def pick(lst):
        if any(n == CONTROLLER for n, _, _ in lst):
            return CONTROLLER
        for n, _, t in lst:
            if "JointTrajectoryController" in t and "gripper" not in n and "mbot" not in n:
                return n
        return None

    name = pick(ctrls)
    state = {n: st for n, st, _ in ctrls}.get(name)
    if state != "active":
        target = name or CONTROLLER
        log("   el controlador del brazo (%s) esta %s: lo activo con el spawner..." % (target, state or "sin cargar"))
        cmd = ["ros2", "run", "controller_manager", "spawner", target, "-c", cm]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_spawner)
            info["spawner"] = (out.stdout + out.stderr).strip()[-1500:]
        except (subprocess.TimeoutExpired, OSError) as e:
            info["spawner"] = str(e)
        ctrls = _list_controllers(node, cm) or ctrls
        info["controladores"] = ctrls
        name = pick(ctrls) or target
        state = {n: st for n, st, _ in ctrls}.get(name)
        if state != "active":
            info["error"] = "el controlador %s no se pudo activar (estado: %s)" % (name, state or "sin cargar")
            return None, info
    info["controlador"] = name
    return "%s/%s/joint_trajectory" % (ns, name), info


def describe_falla(info):
    """Texto de diagnostico: controladores y su estado, y lo ultimo que dijo el spawner."""
    lines = [info.get("error", "no se pudo mover el brazo")]
    if info.get("controladores"):
        lines.append("controladores en %s:" % info.get("controller_manager", "?"))
        lines += ["  - %s: %s (%s)" % c for c in info["controladores"]]
    if info.get("spawner"):
        lines.append("spawner:")
        lines += ["  | " + l for l in info["spawner"].splitlines()[-8:]]
    return "\n   ".join(lines)


def mover_brazo(node, q_goal, vel=0.5, timeout_extra=12.0, tol=0.02, log=print):
    """Manda la trayectoria al xarm6_traj_controller y espera a que /joint_states llegue a q_goal."""
    import rclpy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from sensor_msgs.msg import JointState

    state = {}

    def on_states(msg):
        for n, p in zip(msg.name, msg.position):
            if n in JOINTS:
                state[n] = p

    res = {"q_objetivo": [round(v, 4) for v in q_goal], "alcanzada": False}
    topic, cinfo = asegurar_controlador(node, log=log)
    res["controlador"] = {k: v for k, v in cinfo.items() if k != "spawner"}
    if topic is None:
        res["error"] = describe_falla(cinfo)
        return res
    sub = node.create_subscription(JointState, TOPIC_STATES, on_states, 10)
    pub = node.create_publisher(JointTrajectory, topic, 10)
    t0 = time.time()
    while (pub.get_subscription_count() == 0 or len(state) < 6) and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        res["error"] = "el controlador %s esta activo pero nadie escucha %s" % (cinfo.get("controlador"), topic)
        return res
    if len(state) < 6:
        res["error"] = "no llegan las articulaciones del brazo por %s" % TOPIC_STATES
        return res
    q0 = [state[n] for n in JOINTS]
    dur = max(3.0, max(abs(a - b) for a, b in zip(q_goal, q0)) / vel)
    msg = JointTrajectory()
    msg.joint_names = list(JOINTS)
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in q_goal]
    pt.velocities = [0.0] * 6
    pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1.0) * 1e9))
    msg.points = [pt]
    log("   moviendo el brazo (%.1f s)..." % dur)
    pub.publish(msg)
    t_pub = time.time()
    resent = False
    ok_since = None
    while time.time() - t_pub < dur + timeout_extra:
        rclpy.spin_once(node, timeout_sec=0.05)
        err = max(abs(state[n] - g) for n, g in zip(JOINTS, q_goal))
        if err < tol:
            ok_since = ok_since or time.time()
            if time.time() - ok_since > 0.5:
                res["alcanzada"] = True
                break
        else:
            ok_since = None
        if not resent and time.time() - t_pub > dur + 3.0:
            pub.publish(msg)               # por si el primer mensaje se perdio
            resent = True
    q_fin = [state[n] for n in JOINTS]
    res.update(q_inicial=[round(v, 4) for v in q0], q_final=[round(v, 4) for v in q_fin],
               error_max_rad=round(max(abs(a - b) for a, b in zip(q_fin, q_goal)), 4), duracion_s=round(dur, 1))
    pos, pitch = camara(q_fin)
    res.update(camara_xyz_m=[round(float(v), 3) for v in pos], camara_inclinacion_grados=round(pitch, 1))
    if not res["alcanzada"]:
        res["error"] = "el brazo no llego a la pose (error %.3f rad)" % res["error_max_rad"]
    node.destroy_subscription(sub)
    node.destroy_publisher(pub)
    return res


def foto(node, path, espera=1.5, timeout=5.0):
    """Guarda el siguiente cuadro de la camara que llegue despues de 'espera' segundos."""
    import rclpy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    last = {}
    subs = [node.create_subscription(Image, t, lambda m: last.__setitem__("img", m), 1) for t in CAM_TOPICS]
    t0 = time.time()
    while time.time() - t0 < espera:
        rclpy.spin_once(node, timeout_sec=0.05)
    last.clear()
    while "img" not in last and time.time() - t0 < espera + timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in subs:
        node.destroy_subscription(s)
    if "img" not in last:
        return False
    cv2.imwrite(path, CvBridge().imgmsg_to_cv2(last["img"], desired_encoding="bgr8"))
    return True


def main():
    ap = argparse.ArgumentParser(description="Pone la camara del brazo mirando al frente")
    ap.add_argument("--pose", default="frente", choices=["frente", "alta", "inicial"])
    ap.add_argument("--inclinacion", type=float, default=0.0, help="grados; positivo = hacia arriba")
    ap.add_argument("--foto", default="", help="guarda lo que ve la camara al final (png)")
    args = ap.parse_args()
    try:
        q = pose_joints(args.pose, args.inclinacion)
    except ValueError as e:
        sys.exit("!! " + str(e))
    pos, pitch = camara(q)
    print(">> Pose '%s': q = %s rad" % (args.pose, [round(v, 3) for v in q]))
    print("   camara esperada: %.2f m sobre base_link, %.2f m al frente, inclinacion %+.1f grados"
          % (pos[2], pos[0], pitch))
    import rclpy
    rclpy.init()
    node = rclpy.create_node("t10_camara")
    try:
        res = mover_brazo(node, q)
        if res["alcanzada"]:
            print("   listo: error %.4f rad, camara a %.2f m, inclinacion %+.1f grados"
                  % (res["error_max_rad"], res["camara_xyz_m"][2], res["camara_inclinacion_grados"]))
        else:
            print("!! " + res.get("error", "no se pudo mover el brazo"))
        if args.foto:
            print("   foto: %s" % (args.foto if foto(node, args.foto) else "no llego ninguna imagen"))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if res["alcanzada"] else 1)


if __name__ == "__main__":
    main()
