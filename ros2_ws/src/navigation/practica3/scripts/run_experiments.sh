#!/usr/bin/env bash
# =============================================================================
# run_experiments.sh - Practica 3 (filtro de particulas), Robots Moviles FI-UNAM 2027-1
# Corre todos los experimentos del paso 13 de forma automatica (ver p3_runner.py).
#
# Antes, cada uno en su propia terminal BASH de la distrobox (desde un workspace donde
# house_simul y navig_utils ya esten compilados):
#   T1: ros2 launch house_simul house_simul.launch.py
#   T2: ros2 launch navig_utils localization.launch.py
# y sin ningun `pf` abierto. Luego, en una tercera terminal (bash, no fish):
#   bash run_experiments.sh -n            # solo muestra el plan
#   bash run_experiments.sh -m 1          # prueba de humo (1 corrida)
#   bash run_experiments.sh               # todo: 14 configuraciones x 2 repeticiones
#   bash run_experiments.sh -r 3          # 3 repeticiones
#   bash run_experiments.sh --continuar   # reanuda el ultimo lote si se interrumpio
# Compila particle_filter (el de ESTE repo) y hace source de el, para que el `pf` que se corre
# sea este codigo. build/, install/ y log/ van en el disco de Linux (P3_BUILD_DIR, por defecto
# ~/.cache/practica3_ws): el disco /mnt/Datos no admite enlaces simbolicos y --symlink-install
# los necesita. Los logs quedan en ../datos/ (un .log por corrida + runs.csv).
# =============================================================================
set -o pipefail      # sin -u: los setup.bash de ROS usan variables no definidas
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../../.." && pwd)"
PKG="$WS/src/navigation/particle_filter"
BUILD_DIR="${P3_BUILD_DIR:-$HOME/.cache/practica3_ws}"
# shellcheck disable=SC1091
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
case " $* " in
  *" -n "*|*" -h "*|*" --help "*) ;;      # solo plan o ayuda: no hace falta compilar
  *)
    # Si ese build se genero desde otra copia del repo (p. ej. antes de moverlo), CMake no
    # reconfigura: se borra el build de particle_filter (es solo cache) y se compila de cero.
    CACHE="$BUILD_DIR/build/particle_filter/CMakeCache.txt"
    if [ -f "$CACHE" ]; then
      OLD_SRC="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$CACHE")"
      if [ "$(realpath -m "$OLD_SRC")" != "$(realpath -m "$PKG")" ]; then
        echo ">> El build en $BUILD_DIR viene de $OLD_SRC: lo borro y compilo de cero"
        rm -rf "$BUILD_DIR/build/particle_filter" "$BUILD_DIR/install/particle_filter"
      fi
    fi
    echo ">> Compilando $PKG"
    echo "   (build/install/log en $BUILD_DIR)"
    mkdir -p "$BUILD_DIR"
    ( cd "$BUILD_DIR" && colcon --log-base "$BUILD_DIR/log" build --symlink-install \
        --base-paths "$PKG" --packages-select particle_filter \
        --build-base "$BUILD_DIR/build" --install-base "$BUILD_DIR/install" ) \
      || { echo "!! Fallo la compilacion de particle_filter"; exit 1; }
    ;;
esac
# shellcheck disable=SC1091
[ -f "$BUILD_DIR/install/local_setup.bash" ] && source "$BUILD_DIR/install/local_setup.bash"
export P3_BUILD_DIR="$BUILD_DIR"      # p3_runner.py revisa que el pf venga de aqui
exec python3 "$HERE/p3_runner.py" "$@"
