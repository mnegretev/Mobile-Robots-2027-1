#!/usr/bin/env bash
# =============================================================================
# run_experiments.sh - Practica 3 (filtro de particulas), Robots Moviles FI-UNAM 2027-1
# Corre todos los experimentos del paso 13 de forma automatica (ver p3_runner.py).
#
# Antes, cada uno en su propia terminal BASH de la distrobox:
#   T1: ros2 launch house_simul house_simul.launch.py
#   T2: ros2 launch navig_utils localization.launch.py
# y sin ningun `pf` abierto. Luego, en una tercera terminal:
#   bash run_experiments.sh -n          # solo muestra el plan
#   bash run_experiments.sh -m 1        # prueba de humo (1 corrida)
#   bash run_experiments.sh             # todo: 14 configuraciones x 2 repeticiones
#   bash run_experiments.sh -r 3        # 3 repeticiones
# Los logs quedan en ../datos/ (un .log por corrida + runs.csv).
# =============================================================================
set -o pipefail      # sin -u: los setup.bash de ROS usan variables no definidas
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../../.." && pwd)"
# shellcheck disable=SC1091
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/install/local_setup.bash" ] && source "$WS/install/local_setup.bash"
exec python3 "$HERE/p3_runner.py" "$@"
