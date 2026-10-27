#!/usr/bin/env bash
# =============================================================================
# run_experiments.sh - Tarea 10 (Canny, Hough y Hough probabilistica), Robots Moviles FI-UNAM 2027-1
# Corre todos los experimentos de forma automatica (ver t10_runner.py).
#
# Antes, en otra terminal BASH de la distrobox, lanza la simulacion y NO muevas el robot:
#   ros2 launch house_simul house_simul.launch.py realsense:=True
# Luego, en esta terminal (bash, no fish), sin ningun canny/hough/houghP abierto:
#   bash run_experiments.sh -n          # solo muestra el plan
#   bash run_experiments.sh -m 2        # prueba de humo (2 corridas)
#   bash run_experiments.sh             # todo
#   bash run_experiments.sh --continuar # reanuda el ultimo lote si se interrumpio
# Compila geometric_features (el de ESTE repo) y hace source de el, para que `ros2 run` use este
# codigo. build/, install/ y log/ van en el disco de Linux (T10_BUILD_DIR, por defecto
# ~/.cache/tarea10_ws): el disco /mnt/Datos no admite enlaces simbolicos y --symlink-install
# los necesita. Los datos quedan en ../datos/<lote>/.
# =============================================================================
set -o pipefail      # sin -u: los setup.bash de ROS usan variables no definidas
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../../.." && pwd)"
PKG="$WS/src/vision/geometric_features"
BUILD_DIR="${T10_BUILD_DIR:-$HOME/.cache/tarea10_ws}"
# shellcheck disable=SC1091
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
case " $* " in
  *" -n "*|*" -h "*|*" --help "*) ;;      # solo plan o ayuda: no hace falta compilar
  *)
    echo ">> Compilando $PKG"
    echo "   (build/install/log en $BUILD_DIR)"
    mkdir -p "$BUILD_DIR"
    ( cd "$BUILD_DIR" && colcon --log-base "$BUILD_DIR/log" build --symlink-install \
        --base-paths "$PKG" --packages-select geometric_features \
        --build-base "$BUILD_DIR/build" --install-base "$BUILD_DIR/install" ) \
      || { echo "!! Fallo la compilacion de geometric_features"; exit 1; }
    ;;
esac
# shellcheck disable=SC1091
[ -f "$BUILD_DIR/install/local_setup.bash" ] && source "$BUILD_DIR/install/local_setup.bash"
exec python3 "$HERE/t10_runner.py" "$@"
