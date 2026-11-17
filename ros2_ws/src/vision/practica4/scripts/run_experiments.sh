#!/usr/bin/env bash
#
# PRACTICA 4 - ENTRENAMIENTO DE UNA RED NEURONAL (Robots Moviles 2027-1, FI-UNAM)
# Omar Rosario Carrasco
#
# Corre con fc.py los experimentos de los puntos 4 y 5 de la practica: las 64 combinaciones de
# tasa de aprendizaje (0.5, 1, 3, 10), epocas (3, 10, 50, 100) y tamano de lote (5, 10, 30, 100)
# en cuatro barridos:
#   1) etiquetas one-hot, red 784-30-10        3) etiquetas one-hot, red 784-30-20-10
#   2) etiquetas binarias, red 784-30-4        4) etiquetas binarias, red 784-30-20-4
# Cada barrido guarda un renglon por combinacion en ../datos/<barrido>.csv. Si se interrumpe,
# al volver a correrlo se salta las combinaciones que ya estan en el CSV.
#
# Uso (no necesita ROS, solo Python 3 con NumPy, OpenCV y SymPy):
#   bash run_experiments.sh                       # los cuatro barridos, uno tras otro (aprox. 1.7 h)
#   BARRIDOS="1 2" bash run_experiments.sh        # solo los barridos indicados
#   bash run_experiments.sh --max-seconds 160     # solo las combinaciones que quepan en ese tiempo
#
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="$HERE/../datos"
BARRIDOS="${BARRIDOS:-1 2 3 4}"
mkdir -p "$DATA"

# fc.py busca el dataset en ../dataset, asi que se corre desde su carpeta, como indica la practica
cd "$HERE/../../handwritten_digits/handwritten_digits"

# Un solo hilo de BLAS, para que los tiempos de entrenamiento sean comparables entre corridas
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1

# Bitacora del equipo y de las versiones con las que se corrio (una linea por entorno distinto)
python3 - "$DATA/sistema.txt" <<PY
import datetime, os, platform, sys, numpy
try:
    import sympy; sp = sympy.__version__
except ImportError:
    sp = "no instalado"
cpu = [l.split(":")[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")]
env = "%s | %s x %d | Python %s | NumPy %s | SymPy %s" % (platform.platform(), cpu[0] if cpu else "?",
      len(cpu), platform.python_version(), numpy.__version__, sp)
log = open(sys.argv[1]).read() if os.path.exists(sys.argv[1]) else ""
if env not in log:
    open(sys.argv[1], "a").write("%s | %s\n" % (datetime.datetime.now().isoformat(timespec="seconds"), env))
PY

for b in $BARRIDOS; do
    case $b in
        1) python3 fc.py --experiments          --hidden 30    --csv "$DATA/onehot_784-30-10.csv"    "$@" ;;
        2) python3 fc.py --experiments --binary --hidden 30    --csv "$DATA/binario_784-30-4.csv"    "$@" ;;
        3) python3 fc.py --experiments          --hidden 30,20 --csv "$DATA/onehot_784-30-20-10.csv" "$@" ;;
        4) python3 fc.py --experiments --binary --hidden 30,20 --csv "$DATA/binario_784-30-20-4.csv" "$@" ;;
        *) echo "Barrido desconocido: $b (usa 1, 2, 3 o 4)"; exit 1 ;;
    esac
done
