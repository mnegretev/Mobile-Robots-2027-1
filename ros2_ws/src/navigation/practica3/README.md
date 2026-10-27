# Práctica 3 – Localización por filtros de partículas

Robots Móviles 2027-1, FI-UNAM. Omar Rosario Carrasco.

Evaluación del filtro de partículas del paquete `particle_filter` (ROS 2 Jazzy + Gazebo): 14 configuraciones de (*N*, σ²s, σ²m, σ²r), variando un parámetro a la vez respecto a una base, con repeticiones automáticas.

## Código evaluado

- `../particle_filter/src/ParticleFilter.h`: distribución inicial, movimiento de partículas, similitud, ruleta y remuestreo.
- Nodo: `ros2 run particle_filter pf --ros-args -p use_sim_time:=True -p N:=1000 -p s2s:=0.1 -p s2m:=0.01 -p s2r:=0.01`

## Contenido

| Carpeta | Qué hay |
|---|---|
| `scripts/` | `run_experiments.sh` / `p3_runner.py` (corridas automáticas en Gazebo), `analyze_pf.py` (métricas y figuras), `build_informe.py` (genera el `.odt`) |
| `datos/` | Un `.log` del nodo `pf` por corrida y `runs.csv` (parámetros, repetición, estado). Los lotes nuevos guardan además `<lote>_sistema.json` (equipo, commit, binario `pf` usado, factor de tiempo real y plan) y `<lote>_codigo/` (copia del código de `particle_filter` con el que se corrió). El lote `20261022_122126` es la prueba de humo (una corrida) y no entra en el análisis |
| `resultados/` | `iteraciones.csv`, `corridas.csv`, `configuraciones.csv` y figuras |
| `reporte/` | Fuente del reporte (`fuente/informe.md` + figuras); el `.odt` se genera con `build_informe.py` (ver abajo) |

## Repetir los experimentos (con Gazebo)

Cada uno en su propia terminal **bash** de la distrobox (`distrobox enter rosjj -- bash`), sin ningún `pf` abierto. T1 y T2 se lanzan desde un workspace donde `house_simul` y `navig_utils` ya estén compilados; T3, desde `ros2_ws/src/navigation/practica3` de **este** repo:

```bash
ros2 launch house_simul house_simul.launch.py          # T1
ros2 launch navig_utils localization.launch.py         # T2
bash scripts/run_experiments.sh -n                     # T3: muestra el plan
bash scripts/run_experiments.sh -m 1                   #     prueba de humo
bash scripts/run_experiments.sh                        #     14 configuraciones x 2 repeticiones (~50 min)
bash scripts/run_experiments.sh --continuar            #     reanuda el último lote si se interrumpió
```

`run_experiments.sh` compila el `particle_filter` de **este** repo y hace `source` de él. `build/`, `install/` y `log/` quedan en el disco de Linux (`~/.cache/practica3_ws`, o lo que diga `P3_BUILD_DIR`), porque `/mnt/Datos` no admite los enlaces simbólicos que usa `--symlink-install`. Antes de mover al robot, `p3_runner.py` revisa que el `pf` que va a correr se haya compilado desde este repo (y no desde otro workspace), que llegue la pose real, que esté el servicio del mapa y que el robot esté cerca del origen.

Ctrl+C detiene el filtro y el robot, y la corrida queda como `interrumpida`. `--continuar` reanuda el lote con el plan guardado en `<lote>_sistema.json`: salta las corridas que terminaron bien, repite las demás y no arranca si el código de `particle_filter` cambió desde que empezó el lote. Los logs de los intentos fallidos se conservan como `<run_id>.<estado>.log`; `analyze_pf.py` solo usa las filas `ok` de `runs.csv`.

## Reproducir el análisis y el reporte (sin Gazebo)

```bash
python3 scripts/analyze_pf.py datos/ --out resultados/ --map ../navig_utils/maps/appartment.pgm --batch 20261022_122331
cp resultados/fig_*.png reporte/fuente/fig/
cd reporte/fuente
python3 ../../scripts/build_informe.py --md informe.md --fig fig --data ../../resultados --out ../Practica3_Filtro_de_particulas.odt
```
`20261022_122331` es el lote de las 28 corridas que se reportan (14 configuraciones × 2 repeticiones). Requiere Python 3 con NumPy y Matplotlib; para el reporte, `pandoc` (≥ 3.1), `lxml` y LibreOffice.
