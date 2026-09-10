#!/bin/bash
###############################################################################
# experimentos_practica1.sh
# Robots Moviles (TSM I, TSM II, TSCR), FI-UNAM, 2027-1 - Practica 01
#
# Corre A* (variando cost_radius y diagonals) y RRT (variando epsilon y N)
# sobre varios puntos meta, llamando /path_planning/plan_path directamente
# (sin GUI), y guarda todo en un CSV listo para las tablas del reporte.
#
# REQUISITO: house_simul YA debe estar corriendo (Terminal 1). No hace falta
# navig_utils (la GUI) porque llamamos al servicio directo.
###############################################################################

set -o pipefail

# ---------------------------------------------------------------------------
# CONFIG - ajusta estas rutas/valores si tu setup difiere
# ---------------------------------------------------------------------------
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="/root/ros2_ws/Addons/Ros2/Mobile-Robots-2027-1/ros2_ws/install/setup.bash"
OUT_CSV="$HOME/practica01_resultados.csv"
LOG_DIR="/tmp/p1_logs"
SERVICE_WAIT_TIMEOUT=20   # segundos maximos esperando que aparezca un servicio
CALL_TIMEOUT=90           # segundos maximos por llamada de planeacion

# Ejecutables reales (sin pasar por el wrapper 'ros2 run', que lanza un hijo
# y hace que $! capture el PID equivocado -> los kill/restart no mataban nada)
WS_INSTALL="$(dirname "$WS_SETUP")"
COST_MAP_BIN="${WS_INSTALL}/mapping/lib/mapping/cost_map"
A_STAR_BIN="${WS_INSTALL}/path_planner/lib/path_planner/a_star"
RRT_BIN="${WS_INSTALL}/path_planner/lib/path_planner/rrt"

INFLATION_RADIUS=0.2
COST_RADII=(0.05 0.2 0.5)          # sweep para A*
RRT_COST_RADIUS=0.2                # fijo para RRT (igual que uno de los de A*, para poder comparar)

DIAGONALS_VALUES=(false true)      # A*

EPSILONS=(0.3 0.5)                 # RRT: 2 x 4 = 8 combinaciones (igual que Tarea 4)
NS=(50 100 200 300)
RRT_REPEATS=2                      # RRT es aleatorio -> repetir cada combinacion

START="0.0 0.0"
GOALS=(                            # tentativos - edita si alguno resulta fuera del mapa
  "3.0 0.0"
  "8.0 0.0"
  "8.0 3.0"
  "-3.0 3.0"
  "5.0 -3.0"
)
# ---------------------------------------------------------------------------

mkdir -p "$LOG_DIR"

# Seguro: evita correr dos instancias a la vez (causó datos corruptos/
# contradictorios la vez pasada - dos copias peleando por los mismos nodos
# y escribiendo el mismo log/CSV al mismo tiempo).
LOCKFILE="$LOG_DIR/.script.lock"
if [ -e "$LOCKFILE" ] && kill -0 "$(cat "$LOCKFILE" 2>/dev/null)" 2>/dev/null; then
    echo "[!] Ya hay una instancia de este script corriendo (PID $(cat "$LOCKFILE"))."
    echo "    Mátala primero, o si estás seguro de que ya no existe, borra: $LOCKFILE"
    exit 1
fi
echo $$ > "$LOCKFILE"

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"

# Critico: sin esto, el log de los nodos se queda en el buffer y nunca
# aparece en el archivo hasta que el proceso muere (buffering de stdout
# cuando no hay una terminal (TTY) del otro lado).
export PYTHONUNBUFFERED=1

echo "algoritmo,inflation_radius,cost_radius,diagonals,epsilon,N,start_x,start_y,goal_x,goal_y,exito,tiempo_ms,num_puntos" > "$OUT_CSV"

COST_MAP_PID=""
PLANNER_PID=""

cleanup() {
    echo ""
    echo "Cerrando nodos..."
    [ -n "$PLANNER_PID" ]  && kill -INT "$PLANNER_PID"  2>/dev/null
    [ -n "$COST_MAP_PID" ] && kill -INT "$COST_MAP_PID" 2>/dev/null
    sleep 1
    [ -n "$PLANNER_PID" ]  && kill -9 "$PLANNER_PID"  2>/dev/null
    [ -n "$COST_MAP_PID" ] && kill -9 "$COST_MAP_PID" 2>/dev/null
    rm -f "$LOCKFILE"
}
trap cleanup EXIT INT TERM

wait_for_service () {
    local svc="$1"
    local t=0
    while ! ros2 service list 2>/dev/null | grep -qx "$svc"; do
        sleep 1
        t=$((t+1))
        if [ "$t" -ge "$SERVICE_WAIT_TIMEOUT" ]; then
            echo "  [!] Timeout esperando el servicio $svc"
            return 1
        fi
    done
    return 0
}

start_cost_map () {
    local cost_radius="$1"
    if [ -n "$COST_MAP_PID" ]; then
        kill -INT "$COST_MAP_PID" 2>/dev/null
        wait "$COST_MAP_PID" 2>/dev/null
    fi
    : > "$LOG_DIR/cost_map.log"
    "$COST_MAP_BIN" --ros-args -p inflation_radius:="${INFLATION_RADIUS}" -p cost_radius:="${cost_radius}" \
        > "$LOG_DIR/cost_map.log" 2>&1 &
    COST_MAP_PID=$!
    wait_for_service /get_inflated_map
    wait_for_service /get_cost_map
}

start_planner () {
    # uso: start_planner a_star|rrt  <log_name>  [-p param:=valor ...]
    local kind="$1"; local logname="$2"; shift 2
    if [ -n "$PLANNER_PID" ]; then
        kill -INT "$PLANNER_PID" 2>/dev/null
        wait "$PLANNER_PID" 2>/dev/null
    fi
    : > "$LOG_DIR/${logname}.log"
    local bin="$A_STAR_BIN"
    [ "$kind" = "rrt" ] && bin="$RRT_BIN"
    "$bin" --ros-args "$@" > "$LOG_DIR/${logname}.log" 2>&1 &
    PLANNER_PID=$!
    wait_for_service /path_planning/plan_path
}

call_plan () {
    # uso: call_plan sx sy gx gy logfile
    local sx="$1" sy="$2" gx="$3" gy="$4" logfile="$5"
    local before
    before=$(wc -l < "$logfile" 2>/dev/null || echo 0)
    timeout "$CALL_TIMEOUT" ros2 service call /path_planning/plan_path nav_msgs/srv/GetPlan \
      "{start: {pose: {position: {x: ${sx}, y: ${sy}, z: 0.0}}}, goal: {pose: {position: {x: ${gx}, y: ${gy}, z: 0.0}}}, tolerance: 0.0}" \
      > /dev/null 2>&1
    local new_line
    new_line=$(tail -n +"$((before+1))" "$logfile" 2>/dev/null | grep -E "Path planned after|Cannot plan path" | tail -n1)
    if echo "$new_line" | grep -q "Path planned after"; then
        local t n
        t=$(echo "$new_line" | grep -oE '[0-9.]+ ms' | grep -oE '[0-9.]+')
        n=$(echo "$new_line" | grep -oE 'with [0-9]+ points' | grep -oE '[0-9]+')
        echo "1,${t},${n}"
    else
        echo "0,,"
    fi
}

# Por si quedó algo huerfano de una corrida anterior (ver bug del PID de
# 'ros2 run' arriba) - limpieza defensiva antes de empezar.
pkill -9 -f "$COST_MAP_BIN" 2>/dev/null
pkill -9 -f "$A_STAR_BIN" 2>/dev/null
pkill -9 -f "$RRT_BIN" 2>/dev/null
sleep 1

read -r sx sy <<< "$START"
total=0

echo "=========================================================="
echo " A* : cost_radius x diagonals x $(( ${#GOALS[@]} )) puntos"
echo "=========================================================="
for cost_radius in "${COST_RADII[@]}"; do
    echo ">> [A*] cost_radius=${cost_radius}"
    start_cost_map "$cost_radius"
    start_planner a_star a_star -p diagonals:=false

    for diag in "${DIAGONALS_VALUES[@]}"; do
        # diagonals se lee en cada llamada -> no hace falta reiniciar el nodo
        ros2 param set /a_star_node diagonals "${diag}" > /dev/null 2>&1
        echo "   diagonals=${diag}"
        for goal in "${GOALS[@]}"; do
            read -r gx gy <<< "$goal"
            result=$(call_plan "$sx" "$sy" "$gx" "$gy" "$LOG_DIR/a_star.log")
            echo "      goal=(${gx},${gy}) -> ${result}"
            echo "A*,${INFLATION_RADIUS},${cost_radius},${diag},,,${sx},${sy},${gx},${gy},${result}" >> "$OUT_CSV"
            total=$((total+1))
        done
    done
done

echo "=========================================================="
echo " RRT : epsilon x N x $(( ${#GOALS[@]} )) puntos x ${RRT_REPEATS} repeticiones"
echo " (cost_radius fijo en ${RRT_COST_RADIUS} para poder comparar contra A*)"
echo "=========================================================="
start_cost_map "$RRT_COST_RADIUS"

for eps in "${EPSILONS[@]}"; do
    for n in "${NS[@]}"; do
        echo ">> [RRT] epsilon=${eps} N=${n}"
        start_planner rrt rrt -p epsilon:="${eps}" -p N:="${n}"
        for goal in "${GOALS[@]}"; do
            read -r gx gy <<< "$goal"
            for rep in $(seq 1 "$RRT_REPEATS"); do
                result=$(call_plan "$sx" "$sy" "$gx" "$gy" "$LOG_DIR/rrt.log")
                echo "   goal=(${gx},${gy}) rep=${rep} -> ${result}"
                echo "RRT,${INFLATION_RADIUS},${RRT_COST_RADIUS},,${eps},${n},${sx},${sy},${gx},${gy},${result}" >> "$OUT_CSV"
                total=$((total+1))
            done
        done
    done
done

echo "=========================================================="
echo " Listo: ${total} experimentos guardados en ${OUT_CSV}"
echo "=========================================================="
