# Practica 01 - Planeacion de rutas (A* y RRT)

Automatizacion en ROS2 (sin bash ni grep de logs) para recolectar tiempos
de ejecucion de A* y RRT y generar la tabla comparativa que pide el punto
12 de la practica.

## Arquitectura

- `a_star.py` / `rrt.py` (en `path_planner`): ademas de resolver el
  servicio `/path_planning/plan_path`, ahora publican cada resultado
  (algoritmo, exito, tiempo en ms, parametros usados, punto meta) como
  JSON en el topico `/path_planning/plan_metrics`.
- `experiment_runner` (nodo nuevo en `path_planner`): llama al servicio
  `/path_planning/plan_path` para una lista de puntos meta, y reconfigura
  `diagonals` / `epsilon` / `N` del nodo planificador en caliente
  (`set_parameters`) sin reiniciarlo.
- `metrics_logger` (nodo nuevo en `path_planner`): escucha
  `/path_planning/plan_metrics` y va guardando cada fila en
  `datos/practica01_resultados.csv`. Expone el servicio
  `/path_planning/generate_table` para generar la tabla comparativa
  (`resultados/tabla_comparativa.md`) en cualquier momento.
- `scripts/generar_tabla.py`: la misma generacion de tabla, pero como
  script standalone (sin ROS2), por si quieres regenerarla mientras
  escribes el reporte.

## Launch universal (recomendado)

En vez de las 4-6 terminales manuales, un solo launch levanta todo
(simulacion, localizacion, cost_map, el planificador elegido y
metrics_logger):

```bash
ros2 launch navig_utils practica1.launch.py algorithm:=a_star diagonals:=false
ros2 launch navig_utils practica1.launch.py algorithm:=a_star diagonals:=true cost_radius:=0.5
ros2 launch navig_utils practica1.launch.py algorithm:=rrt epsilon:=0.5 N:=100
ros2 launch navig_utils practica1.launch.py algorithm:=a_star use_gui:=false   # headless, para barridos
```

Argumentos: `algorithm` (a_star|rrt), `diagonals` (a_star), `epsilon`/`N`
(rrt), `inflation_radius`, `cost_radius`, `use_gui` (false quita
lira_gui+RViz), `csv_path`/`table_path` (por defecto en tu home:
`~/practica01_resultados.csv` y `~/tabla_comparativa_practica01.md`).

Para cambiar de algoritmo o de `cost_radius`/`inflation_radius` hay que
volver a lanzar (Ctrl+C y repetir con otros argumentos) - esos siguen
siendo procesos distintos, el launch solo evita escribir cada comando
a mano. Una vez arriba y estable (dale ~5-10s a que `lifecycle_bringup`
active map_server/amcl), corre `experiment_runner` aparte:

```bash
ros2 run path_planner experiment_runner --ros-args -p phase:=a_star
```

## Como correrlo

Despues de `colcon build --symlink-install` y volver a sourcear el
workspace:

```bash
cd ~/Addons/ROS2JJ/Mobile-Robots-2027-1/ros2_ws/src/navigation
```

(Lanza todo lo siguiente desde aqui, para que las rutas relativas de
`metrics_logger` caigan dentro de `practica1/`.)

1. Terminal 1: `ros2 launch house_simul house_simul.launch.py`
2. Terminal 2: `ros2 launch navig_utils navig_utils.launch.py`
3. Terminal 3: `ros2 run mapping cost_map --ros-args -p inflation_radius:=0.2 -p cost_radius:=0.2`
4. Terminal 4: `ros2 run path_planner a_star --ros-args -p diagonals:=false`
5. Terminal 5 (dejala corriendo toda la sesion):
   `ros2 run path_planner metrics_logger`
6. Terminal 6 (automatiza la fase de A*):
   `ros2 run path_planner experiment_runner --ros-args -p phase:=a_star`

Cuando termine A*, en la terminal 4 detenla (Ctrl+C) y corre
`ros2 run path_planner rrt` en su lugar; la terminal 5 (`metrics_logger`)
NO la toques. Luego en la terminal 6:
`ros2 run path_planner experiment_runner --ros-args -p phase:=rrt`

Si quieres repetir el barrido de `cost_radius` (puntos 6-10 de la
practica), reinicia solo la terminal 3 (`cost_map`) con el nuevo valor;
las demas terminales no necesitan tocarse. El `cost_radius` activo no
queda registrado automaticamente en el CSV (no lo conoce el nodo
planificador) - si lo quieres anotar, pon una etiqueta antes de cada
tanda:
`ros2 param set /metrics_logger session_label "cost_radius=0.5"`

## Generar la tabla final

En cualquier momento, sin parar nada:

```bash
ros2 service call /path_planning/generate_table std_srvs/srv/Trigger
```

o, sin ROS2 corriendo, desde este folder:

```bash
python3 scripts/generar_tabla.py
```

Ambas opciones leen `datos/practica01_resultados.csv` y escriben
`resultados/tabla_comparativa.md`.

## Ajustar el experimento

Los puntos meta, valores de `diagonals`, y combinaciones de `epsilon`/`N`
estan en el bloque CONFIG al inicio de `experiment_runner.py` (mismos
valores que tenia `experimentos_practica1.sh`). Edita esa lista y
recompila (`colcon build --packages-select path_planner`) si quieres
otros puntos o parametros.
