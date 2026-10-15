# Practica 01 - Planeacion de rutas (A* y RRT)

Automatizacion en ROS2 (sin bash ni grep de logs) para recolectar tiempos
de ejecucion de A* y RRT en un solo CSV.

## Arquitectura

- `a_star.py` / `rrt.py` (en `path_planner`): ademas de resolver el
  servicio `/path_planning/plan_path`, publican cada resultado
  (algoritmo, exito, tiempo en ms, parametros usados, punto start/meta)
  como JSON en el topico `/path_planning/plan_metrics`.
- `experiment_runner` (nodo en `path_planner`): llama al servicio
  `/path_planning/plan_path` para `num_pruebas` pares (start, goal)
  tomados al azar dentro de un area fija, y reconfigura `diagonals` /
  `epsilon` / `N` del nodo planificador en caliente (`set_parameters`)
  sin reiniciarlo. Ver el bloque CONFIG al inicio del archivo
  (`AREA_X`/`AREA_Y`, `NUM_PRUEBAS_DEFAULT`).
- `metrics_logger` (nodo en `path_planner`): escucha
  `/path_planning/plan_metrics` y va guardando CADA resultado (de
  a_star o de rrt, los distingue la columna `algorithm`) en un solo CSV:
  `datos/resultados_practica01.csv`. Dejalo corriendo durante toda la
  sesion, cambies o no de algoritmo - no pierde nada entre medio.

Ruta por defecto (se calcula sola a partir de la ubicacion real del
archivo, funciona con `--symlink-install` sin importar desde donde
lances el nodo):

```
practica1/
  datos/
    resultados_practica01.csv
```

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
lira_gui+RViz), `csv_path` (por defecto `practica1/datos/resultados_practica01.csv`,
dentro de este repo).

Para cambiar de algoritmo o de `cost_radius`/`inflation_radius` hay que
volver a lanzar (Ctrl+C y repetir con otros argumentos) - esos siguen
siendo procesos distintos, el launch solo evita escribir cada comando
a mano. Una vez arriba y estable (dale ~5-10s a que `lifecycle_bringup`
active map_server/amcl), corre `experiment_runner` aparte:

```bash
ros2 run path_planner experiment_runner --ros-args -p phase:=a_star -p num_pruebas:=10
# ... Ctrl+C la terminal del launch, relanza con algorithm:=rrt ...
ros2 run path_planner experiment_runner --ros-args -p phase:=rrt -p num_pruebas:=10
```

Ambas fases escriben al MISMO `resultados_practica01.csv` (se distinguen
por la columna `algorithm`), asi que al final tienes un solo archivo con
todo.

## Como correrlo (manual, sin el launch universal)

Despues de `colcon build --symlink-install` y volver a sourcear el
workspace:

1. Terminal 1: `ros2 launch house_simul house_simul.launch.py`
2. Terminal 2: `ros2 launch navig_utils navig_utils.launch.py`
3. Terminal 3: `ros2 run mapping cost_map --ros-args -p inflation_radius:=0.2 -p cost_radius:=0.2`
4. Terminal 4: `ros2 run path_planner a_star --ros-args -p diagonals:=false`
5. Terminal 5 (dejala corriendo toda la sesion):
   `ros2 run path_planner metrics_logger`
6. Terminal 6 (automatiza la fase de A*):
   `ros2 run path_planner experiment_runner --ros-args -p phase:=a_star -p num_pruebas:=10`

Cuando termine A*, en la terminal 4 detenla (Ctrl+C) y corre
`ros2 run path_planner rrt` en su lugar; la terminal 5 (`metrics_logger`)
NO la toques - sigue escribiendo al mismo CSV. Luego en la terminal 6:
`ros2 run path_planner experiment_runner --ros-args -p phase:=rrt -p num_pruebas:=10`

Si quieres repetir el barrido de `cost_radius` (puntos 6-10 de la
practica), reinicia solo la terminal 3 (`cost_map`) con el nuevo valor;
las demas terminales no necesitan tocarse. El `cost_radius` activo no
queda registrado automaticamente en el CSV (no lo conoce el nodo
planificador) - si lo quieres anotar, pon una etiqueta antes de cada
tanda:
`ros2 param set /metrics_logger session_label "cost_radius=0.5"`

## Ajustar el experimento

`start` y `goal` se toman al azar en cada prueba, dentro de `AREA_X`/
`AREA_Y` (bloque CONFIG al inicio de `experiment_runner.py`), y
`num_pruebas` (parametro ROS2, default 5) dice cuantos pares al azar
probar por cada combinacion de `diagonals` (a_star) o `epsilon`/`N` (rrt):

```bash
ros2 run path_planner experiment_runner --ros-args -p phase:=rrt -p num_pruebas:=10 -p seed:=42
```

`seed` (opcional, entero >= 0) fija la semilla aleatoria para resultados
reproducibles. No hay garantia de que un punto al azar caiga en zona
libre del mapa - eso tambien queda como FALLO en el CSV, es informacion
real. Si ves demasiados FALLO "sospechosos" (exec_time_ms muy bajo,
como si ni buscara), achica `AREA_X`/`AREA_Y` y recompila
(`colcon build --packages-select path_planner`).

## La tabla comparativa

Este paquete ya no la genera automaticamente - `resultados_practica01.csv`
trae todo lo necesario (columna `algorithm`, `success`, `exec_time_ms`,
`num_points`, parametros usados) para armar la comparacion que pida la
practica directo en pandas/Excel al momento de escribir el reporte.
