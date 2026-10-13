# Práctica 2 – Seguimiento de rutas (pure pursuit vs Stanley)

Robots Móviles 2027-1, FI-UNAM. Omar Rosario Carrasco.

Comparación de pure pursuit y Stanley sobre una base diferencial (ROS 2 Jazzy + Gazebo): 27 corridas en dos rutas (R1, R2), variando un parámetro a la vez.

## Código evaluado
Los nodos de la práctica están en el paquete `path_follower` (carpeta hermana de ésta):

- `../path_follower/path_follower/pure_pursuit.py` (parámetros `alpha`, `beta`)
- `../path_follower/path_follower/stanley.py` (parámetros `Ka`, `Kd`)

Se lanzan con `motion_planning.launch.py` del paquete `navig_utils` (argumento `controller`); la meta se da con *2D Goal Pose* en RViz2. Cada nodo guarda `goal_*.txt` (x, y) y `robot_*.txt` (x, y, θ, v, ω) al terminar.

## Contenido

| Carpeta | Qué hay |
|---|---|
| `reporte/` | Reporte final (`.odt` editable y vista previa en PDF) y su fuente (`informe.md` + figuras) |
| `scripts/` | `run_experiments.sh` (barrido automático), análisis y figuras en Python, `build_informe.py` (genera el `.odt`) |
| `datos/` | Los `.txt` de las 27 corridas y `runs.csv` (parámetros y duración de cada una) |
| `resultados/` | Tablas (CSV/MD) y figuras: métricas por corrida, comparación, validación con modelo ideal, dinámica lineal, error de equilibrio |
| `HALLAZGOS.md` | Resumen de resultados |

## Reproducir el análisis (sin Gazebo)
Requiere Python 3 con NumPy y Matplotlib. Desde esta carpeta:

```bash
python3 scripts/analyze_path_following.py datos/ --meta datos/runs.csv --out resultados/
python3 scripts/kinematic_check.py datos/ --meta datos/runs.csv --out resultados/
python3 scripts/linealizacion_check.py --out resultados/
python3 scripts/steady_state_check.py datos/ --meta datos/runs.csv --metrics resultados/resumen_metricas.csv --out resultados/
python3 scripts/fig_ganancia_efectiva.py resultados/resumen_metricas.csv --out resultados/
python3 scripts/figuras_marco_teorico.py --out resultados/
```

## Repetir los experimentos (con Gazebo)
Dentro del contenedor `Ros2JJ`, con `xhost +local:docker` hecho en el host y el workspace compilado:

```bash
bash scripts/run_experiments.sh -n      # solo muestra el plan
bash scripts/run_experiments.sh         # corre las 28 combinaciones
bash scripts/run_experiments.sh -r 4    # 4 repeticiones por combinación
```

## Regenerar el reporte
Requiere `pandoc`, `lxml` y LibreOffice (con Math):

```bash
cd reporte/fuente
python3 ../../scripts/build_informe.py --md informe.md --fig fig --data ../../resultados --out ../Practica2_Seguimiento_de_rutas.odt
```
