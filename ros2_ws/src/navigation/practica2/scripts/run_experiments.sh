#!/usr/bin/env bash
# =============================================================================
# run_experiments.sh - Práctica 2 (seguimiento de rutas), Robots Móviles FI-UNAM
#
# Automatiza los experimentos: para cada combinación de parámetros y cada ruta
#   1. genera una COPIA TEMPORAL de motion_planning.launch.py con los valores
#      (no modifica tu launch original ni hace falta recompilar),
#   2. reinicia house_simul + motion_planning (el robot vuelve al origen y el
#      nodo seguidor arranca con nav_data vacío),
#   3. publica la meta en /goal_pose (sin dar clic en RViz),
#   4. espera a que el nodo guarde robot_*.txt / goal_*.txt,
#   5. apaga todo y registra la corrida en runs.csv (con la duración tomada
#      del log del nodo).
# Al final llama a analyze_path_following.py si está junto a este script.
#
# USO (dentro del contenedor Ros2JJ, con `xhost +local:docker` ya hecho en el host):
#   ./run_experiments.sh -n            # solo muestra el plan (no corre nada)
#   ./run_experiments.sh               # corre todo (pure pursuit y Stanley)
#   ./run_experiments.sh -a pp         # solo pure pursuit   (-a st = solo Stanley)
#   ./run_experiments.sh -r 2          # 2 repeticiones por combinación
#   ./run_experiments.sh -R "R1"       # solo la ruta R1
#   ./run_experiments.sh -m 1          # SOLO la primera corrida (prueba de humo)
#   ./run_experiments.sh -o DIR        # carpeta de salida (default ~/practica2_exp/<fecha>)
#   ./run_experiments.sh -c -o DIR     # COMPLETAR: repite sólo las corridas que faltan
#                                      #   en DIR/runs.csv (agrega a la misma carpeta)
#
# Si una corrida falla por un problema conocido de arranque (cost_map con
# ZeroDivisionError, conexión X11 cortada, controladores de Gazebo que no
# levantan) se reintenta sola hasta RETRIES veces (default 2); los logs de los
# intentos quedan en logs/<id>.log, logs/<id>_try2.log, ...
#
# Variables de entorno opcionales: WS, V_MAX, W_MAX, SIM_WAIT, SETTLE, RETRIES,
#   RUN_TIMEOUT (s para terminar el seguimiento; p. ej. RUN_TIMEOUT=300)
# =============================================================================
# OJO: no usar `set -u`; el setup.bash de ROS/colcon falla con nounset.
set -o pipefail

# ------------------------------ CONFIGURACIÓN --------------------------------
WS="${WS:-$HOME/Addons/ROS2JJ/Mobile-Robots-2027-1/ros2_ws}"
LAUNCH_SRC="${LAUNCH_SRC:-$WS/src/navigation/navig_utils/launch/motion_planning.launch.py}"

# Mismos límites de velocidad para los dos algoritmos (comparación justa).
V_MAX="${V_MAX:-0.5}"
W_MAX="${W_MAX:-1.0}"

# Rutas: "nombre  x  y  yaw_final(rad)"  - el robot siempre parte del origen.
# R1 ya se probó a mano (4.4716, -0.7614). R2 es una meta del mismo mapa que usó
# el profesor en la Tarea 4; si A* no encuentra ruta, el script lo registra y sigue.
ROUTES=( "R1 4.4716 -0.7614 0.0"
         "R2 0.0 5.0 0.0" )

# Pure pursuit: "alpha beta"  -> barrido de alpha (beta=0.4) y de beta (alpha=1.0),
# 4 valores por parámetro, la base (1.0, 0.4) se comparte.
PP_COMBOS=( "0.1 0.4" "0.5 0.4" "1.0 0.4" "4.0 0.4"
            "1.0 0.1" "1.0 1.0" "1.0 4.0" )

# Stanley: "Kd Ka" -> barrido de Ka (Kd=1.0) y de Kd (Ka=1.0), la base (1.0, 1.0) se comparte.
ST_COMBOS=( "1.0 0.5" "1.0 1.0" "1.0 2.0" "1.0 5.0"
            "0.5 1.0" "2.0 1.0" "5.0 1.0" )

SIM_WAIT="${SIM_WAIT:-12}"     # s de espera tras lanzar house_simul (con 8 s a veces cost_map arrancaba sin mapa)
SETTLE="${SETTLE:-6}"          # s extra tras "Ready" (costmap y A* listos)
READY_TIMEOUT="${READY_TIMEOUT:-90}"   # s para que el seguidor diga "Ready"
PLAN_TIMEOUT="${PLAN_TIMEOUT:-30}"      # s para que A* devuelva ruta
RUN_TIMEOUT="${RUN_TIMEOUT:-150}"   # s para terminar el seguimiento
RETRIES="${RETRIES:-2}"        # reintentos extra si falla el arranque

REPS=1; ALGS="pp st"; DRY=0; MAXN=0; CONTINUE=0
OUT="$HOME/practica2_exp/$(date +%Y%m%d_%H%M%S)"
ROUTE_FILTER=""

while getopts "nr:a:R:o:m:ch" opt; do
  case $opt in
    n) DRY=1 ;;
    c) CONTINUE=1 ;;
    m) MAXN=$OPTARG ;;
    r) REPS=$OPTARG ;;
    a) case $OPTARG in pp) ALGS="pp";; st) ALGS="st";; both) ALGS="pp st";; *) echo "-a: pp|st|both"; exit 1;; esac ;;
    R) ROUTE_FILTER=$OPTARG ;;
    o) OUT=$OPTARG ;;
    h|*) sed -n '2,34p' "$0"; exit 0 ;;
  esac
done

# --------------------------------- UTILIDADES --------------------------------
fmt() { awk -v x="$1" 'BEGIN{printf "%.4f", x}'; }   # fuerza decimales (parámetros tipo double)

make_launch() {  # alg p1 p2 archivo_salida carpeta_datos
  local alg=$1 p1 p2 outf=$4 ddir=$5 fv fw
  p1=$(fmt "$2"); p2=$(fmt "$3"); fv=$(fmt "$V_MAX"); fw=$(fmt "$W_MAX")
  cp "$LAUNCH_SRC" "$outf" || return 1
  sed -i -E "s|os\.path\.expanduser\('~'\)|'$ddir'|g" "$outf"
  if [ "$alg" = pp ]; then
    sed -i -E "s|\{'alpha':[^}]*\}, *\{'beta':[^}]*\}|{'alpha':$p1}, {'beta':$p2}, {'v_max':$fv}, {'w_max':$fw}|" "$outf"
    grep -q "{'alpha':$p1}, {'beta':$p2}, {'v_max':$fv}" "$outf" || return 2
  else
    sed -i -E "s|\{'Kd':[^}]*\}, *\{'Ka':[^}]*\}|{'Kd':$p1}, {'Ka':$p2}, {'v_max':$fv}, {'w_max':$fw}|" "$outf"
    sed -i -E "s|^( *)pure_pursuit_node *\$|\1#pure_pursuit_node|; s|^( *)#stanley_node *\$|\1stanley_node|" "$outf"
    grep -q "{'Kd':$p1}, {'Ka':$p2}, {'v_max':$fv}" "$outf" || return 2
    grep -Eq "^ *stanley_node *\$" "$outf" || return 3
  fi
  grep -q "'$ddir'" "$outf" || return 5
  python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read())" "$outf" 2>/dev/null || return 4
}

wait_for_log() {  # archivo patrón timeout_s
  local i=0
  while [ "$i" -lt "$3" ]; do
    grep -q "$2" "$1" 2>/dev/null && return 0
    sleep 1; i=$((i+1))
  done
  return 1
}

stop_group() {  # pid del líder de grupo
  local pid=$1 i
  [ -z "$pid" ] && return 0
  kill -INT -- "-$pid" 2>/dev/null
  for i in $(seq 1 15); do kill -0 "$pid" 2>/dev/null || return 0; sleep 1; done
  kill -TERM -- "-$pid" 2>/dev/null; sleep 2
  kill -KILL -- "-$pid" 2>/dev/null
  return 0
}

wait_nodes_gone() {
  local i
  for i in $(seq 1 15); do
    [ -z "$(ros2 node list --no-daemon 2>/dev/null)" ] && return 0
    sleep 2
  done
  return 1
}

diagnose() {  # simlog mplog -> causa conocida (vacío si no se reconoce)
  local s=$1 m=$2
  if grep -q "ZeroDivisionError" "$m" 2>/dev/null; then
    echo "cost_map falló al arrancar (ZeroDivisionError: el mapa aún no había llegado)"
  elif grep -qE "X11 connection broke|XIO: +fatal IO error" "$s" "$m" 2>/dev/null; then
    echo "se cortó la conexión X11 (¿xhost +local:docker? ¿se cerró una ventana de Gazebo/RViz?)"
  elif grep -q "Could not successfully call service /controller_manager" "$s" "$m" 2>/dev/null; then
    echo "los controladores de Gazebo no arrancaron (spawner/controller_manager)"
  fi
}

SIM_PID=""; MP_PID=""
cleanup() { stop_group "$MP_PID"; stop_group "$SIM_PID"; MP_PID=""; SIM_PID=""; }
trap 'cleanup; exit 130' INT TERM

# ------------------------------ LISTA DE EXPERIMENTOS ------------------------
EXPS=()   # "alg p1 p2 ruta gx gy gyaw rep"
DONE_KEYS=""
if [ "$CONTINUE" -eq 1 ]; then
  [ -f "$OUT/runs.csv" ] || { echo "-c necesita -o DIR con un runs.csv existente (no encontré $OUT/runs.csv)."; exit 1; }
  DONE_KEYS=$(python3 - "$OUT/runs.csv" <<'PYEOF'
import csv, sys
for r in csv.DictReader(open(sys.argv[1], newline="", encoding="utf-8")):
    if r["algoritmo"] == "pure_pursuit":
        print("pp", r["alpha"], r["beta"], r["route"], r["rep"])
    else:
        print("st", r["Kd"], r["Ka"], r["route"], r["rep"])
PYEOF
)
fi
for alg in $ALGS; do
  if [ "$alg" = pp ]; then combos=("${PP_COMBOS[@]}"); else combos=("${ST_COMBOS[@]}"); fi
  for r in "${ROUTES[@]}"; do
    read -r rn gx gy gyaw <<<"$r"
    if [ -n "$ROUTE_FILTER" ] && ! [[ " $ROUTE_FILTER " == *" $rn "* ]]; then continue; fi
    for c in "${combos[@]}"; do
      read -r p1 p2 <<<"$c"
      for rep in $(seq 1 "$REPS"); do
        if [ "$CONTINUE" -eq 1 ] && grep -qxF "$alg $p1 $p2 $rn $rep" <<<"$DONE_KEYS"; then continue; fi
        EXPS+=("$alg $p1 $p2 $rn $gx $gy $gyaw $rep")
      done
    done
  done
done
[ "$MAXN" -gt 0 ] && EXPS=("${EXPS[@]:0:$MAXN}")
N=${#EXPS[@]}
[ "$N" -eq 0 ] && { if [ "$CONTINUE" -eq 1 ]; then echo "No falta ninguna corrida: todo está en $OUT/runs.csv."; exit 0; fi; echo "No hay experimentos (revisa -R/-a)."; exit 1; }

echo "Experimentos: $N   (~$((N*80/60)) min estimados)   Salida: $OUT"
echo "v_max=$V_MAX  w_max=$W_MAX"
i=0
for e in "${EXPS[@]}"; do
  i=$((i+1)); read -r alg p1 p2 rn gx gy gyaw rep <<<"$e"
  if [ "$alg" = pp ]; then lab="PP alpha=$p1 beta=$p2"; else lab="ST Kd=$p1 Ka=$p2"; fi
  printf "  %2d/%d  %-24s ruta %-3s -> (%s, %s)  rep %s\n" "$i" "$N" "$lab" "$rn" "$gx" "$gy" "$rep"
done

mkdir -p "$OUT/datos" "$OUT/logs" "$OUT/tmp"

# ----------------------------------- MODO -n ---------------------------------
if [ "$DRY" -eq 1 ]; then
  echo; echo "[dry-run] comprobando que las copias del launch se generan bien..."
  for alg in $ALGS; do
    if [ "$alg" = pp ]; then c="${PP_COMBOS[0]}"; else c="${ST_COMBOS[0]}"; fi
    read -r p1 p2 <<<"$c"
    if make_launch "$alg" "$p1" "$p2" "$OUT/tmp/dry_$alg.launch.py" "$OUT/datos"; then
      echo "  OK  $alg -> $OUT/tmp/dry_$alg.launch.py"
      diff <(cat "$LAUNCH_SRC") "$OUT/tmp/dry_$alg.launch.py" | grep '^[<>]' | sed 's/^/      /'
    else
      echo "  ERROR generando launch para $alg (código $?). ¿Cambió el formato de $LAUNCH_SRC?"; exit 2
    fi
  done
  exit 0
fi

# ---------------------------------- PRE-VUELO --------------------------------
if ! command -v ros2 >/dev/null 2>&1; then
  [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
fi
command -v ros2 >/dev/null 2>&1 || { echo "ros2 no está en el PATH (entra a Ros2JJ y haz source install/setup.bash)."; exit 1; }
for p in path_follower house_simul navig_utils; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || { echo "Falta el paquete '$p' (¿compilaste y hiciste source?)."; exit 1; }
done
[ -n "$DISPLAY" ] || echo "[aviso] DISPLAY vacío: RViz/simulador necesitan X11 (xhost +local:docker en el host)."
if [ -n "$(ros2 node list --no-daemon 2>/dev/null)" ]; then
  echo "Hay nodos ROS activos. Cierra house_simul / motion_planning (Ctrl+C) y reintenta:"
  ros2 node list --no-daemon 2>/dev/null; exit 1
fi

if [ "$CONTINUE" -eq 0 ]; then
  echo "suffix,label,route,duration_s,algoritmo,alpha,beta,Kd,Ka,v_max,w_max,rep" > "$OUT/runs.csv"
  : > "$OUT/fallos.txt"
  cp "$LAUNCH_SRC" "$OUT/motion_planning.launch.py.original" 2>/dev/null
fi

# ------------------------------------ BUCLE ----------------------------------
# run_one: una corrida completa. Devuelve 0 si salió bien; si no, deja el motivo en $FAIL.
run_one() {  # id_log
  local tag=$1 launch simlog mplog qz qw msg npts t gfile suf t0 t1 dur extra
  FAIL=""
  launch="$OUT/tmp/$tag.launch.py"
  make_launch "$alg" "$p1" "$p2" "$launch" "$OUT/datos" || { FAIL="no se pudo generar el launch (código $?)"; return 1; }

  simlog="$OUT/logs/${tag}_sim.log"; mplog="$OUT/logs/${tag}.log"
  CUR_SIMLOG=$simlog; CUR_MPLOG=$mplog
  setsid ros2 launch house_simul house_simul.launch.py >"$simlog" 2>&1 & SIM_PID=$!
  sleep "$SIM_WAIT"
  setsid ros2 launch "$launch" >"$mplog" 2>&1 & MP_PID=$!

  wait_for_log "$mplog" "Ready to execute new path" "$READY_TIMEOUT" || { FAIL="el seguidor no llegó a 'Ready' en ${READY_TIMEOUT}s"; return 1; }
  sleep "$SETTLE"

  touch "$OUT/.marker"; sleep 1
  qz=$(awk -v y="$gyaw" 'BEGIN{printf "%.6f", sin(y/2)}'); qw=$(awk -v y="$gyaw" 'BEGIN{printf "%.6f", cos(y/2)}')
  msg="{header: {frame_id: map}, pose: {position: {x: $gx, y: $gy, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}}}"
  ros2 topic pub --once -w 1 /goal_pose geometry_msgs/msg/PoseStamped "$msg" >/dev/null 2>&1 \
    || ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "$msg" >/dev/null 2>&1

  wait_for_log "$mplog" "Path planned with" "$PLAN_TIMEOUT" || { FAIL="no apareció 'Path planned with' en ${PLAN_TIMEOUT}s (A* sin respuesta)"; return 1; }
  npts=$(grep -m1 "Path planned with" "$mplog" | sed -E 's/.*planned with ([0-9]+) points.*/\1/')
  if [ -z "$npts" ] || [ "$npts" -lt 2 ]; then FAIL="A* devolvió una ruta de ${npts:-0} puntos (¿meta ($gx,$gy) en obstáculo?)"; return 1; fi

  t=0; gfile=""
  while [ "$t" -lt "$RUN_TIMEOUT" ]; do
    gfile=$(find "$OUT/datos" -name "goal_${algn}*.txt" -newer "$OUT/.marker" 2>/dev/null | head -1)
    [ -n "$gfile" ] && break
    sleep 1; t=$((t+1))
  done
  [ -z "$gfile" ] && { FAIL="el robot no terminó el seguimiento en ${RUN_TIMEOUT}s (no hay .txt; ¿no converge o se atoró?)"; return 1; }
  sleep 1

  suf=$(basename "$gfile" | sed -E "s/goal_${algn}([0-9]{8}T[0-9]{6})\.txt/\1/")
  t0=$(grep -m1 "Following path with"      "$mplog" | sed -E 's/.*\[INFO\] \[([0-9]+\.[0-9]+)\].*/\1/')
  t1=$(grep -m1 "Global goal point reached" "$mplog" | sed -E 's/.*\[INFO\] \[([0-9]+\.[0-9]+)\].*/\1/')
  dur=""; [ -n "$t0" ] && [ -n "$t1" ] && dur=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.2f", b-a}')
  if [ "$alg" = pp ]; then extra="$p1,$p2,,"; else extra=",,$p1,$p2"; fi
  echo "$suf,\"$lab\",$rn,$dur,$algn,$extra,$V_MAX,$W_MAX,$rep" >> "$OUT/runs.csv"
  echo "      OK  suffix=$suf  duración=${dur:-?} s  puntos de ruta=$npts"
  return 0
}

i=0; OK=0
for e in "${EXPS[@]}"; do
  i=$((i+1)); read -r alg p1 p2 rn gx gy gyaw rep <<<"$e"
  if [ "$alg" = pp ]; then id="pp_a${p1}_b${p2}_${rn}_r${rep}"; lab="alpha=$p1, beta=$p2"; algn=pure_pursuit
  else                      id="st_kd${p1}_ka${p2}_${rn}_r${rep}"; lab="Kd=$p1, Ka=$p2";      algn=stanley; fi
  echo; echo "[$i/$N] $id"

  attempt=1; success=0
  while :; do
    tag=$id; [ "$attempt" -gt 1 ] && tag="${id}_try${attempt}"
    run_one "$tag" && { success=1; break; }
    cleanup; wait_nodes_gone
    cause=$(diagnose "$CUR_SIMLOG" "$CUR_MPLOG")
    if [ -n "$cause" ] && [ "$attempt" -le "$RETRIES" ]; then
      echo "      intento $attempt falló: $FAIL"
      echo "      causa probable: $cause -> reintentando ($((attempt+1))/$((RETRIES+1)))"
      attempt=$((attempt+1)); continue
    fi
    msg="$FAIL"; [ -n "$cause" ] && msg="$FAIL — causa probable: $cause"
    echo "      FALLÓ: $msg"
    { echo "      FALLÓ: $msg"; echo "      ($id, $attempt intento(s); logs en $OUT/logs/)"; } >> "$OUT/fallos.txt"
    break
  done

  if [ "$success" -eq 1 ]; then
    OK=$((OK+1))
    cleanup
    wait_nodes_gone || { echo "Quedaron nodos ROS vivos; detengo para no mezclar corridas:"; ros2 node list --no-daemon; exit 1; }
  fi
done

# ---------------------------------- RESUMEN ----------------------------------
echo; echo "Listo: $OK/$N corridas buenas. Datos: $OUT/datos   runs.csv: $OUT/runs.csv"
[ -s "$OUT/fallos.txt" ] && echo "Corridas fallidas -> $OUT/fallos.txt"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/analyze_path_following.py" ] && python3 -c "import matplotlib, numpy" 2>/dev/null; then
  python3 "$HERE/analyze_path_following.py" "$OUT/datos" --out "$OUT/resultados" --meta "$OUT/runs.csv"
else
  echo "Para analizar:  python3 analyze_path_following.py $OUT/datos --out $OUT/resultados --meta $OUT/runs.csv"
fi
