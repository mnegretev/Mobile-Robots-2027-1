# Automatización Práctica 1 — Cómo funciona

**En una frase:** en vez de llamar el servicio de planeación a mano y copiar tiempos del log, ahora un nodo pide las rutas automáticamente, el planificador avisa el resultado de cada una por un tópico, y otro nodo escucha ese aviso y lo guarda en un CSV — sin reiniciar nada entre medio.

## El problema que reemplaza

Antes, para generar los datos de la práctica había que, a mano:
1. Lanzar A* o RRT.
2. Llamar el servicio de planeación por terminal (`ros2 service call ...`), uno por uno.
3. Copiar el tiempo de ejecución desde el log de consola.
4. Repetir todo esto para cada combinación de parámetros (diagonales, epsilon, N).
5. Armar la tabla comparativa a mano.

Ahora todo eso lo hacen nodos de ROS2 hablándose entre sí.

## Las piezas (y qué hace cada una)

| Nodo | Qué es | Qué hace |
|---|---|---|
| `a_star_node` / `rrt_node` | Ya existían (práctica original) | Resuelven el servicio `/path_planning/plan_path`: les mandas start+goal, regresan la ruta (o vacía si no encontraron). |
| `experiment_runner` | Nuevo | Cliente automático: genera pares start/goal al azar, le cambia los parámetros al planificador sin reiniciarlo, y llama al servicio muchas veces seguidas. |
| `metrics_logger` | Nuevo | Escucha en segundo plano y guarda cada resultado en un CSV. |

## El flujo, paso a paso

```
                 +---------------------------+
                 |     experiment_runner      |
                 | (genera start/goal al azar,|
                 |  cambia parametros)        |
                 +-------------+---------------+
                               | 1. llama al servicio
                               v
                 +---------------------------+
                 |  a_star_node / rrt_node    |
                 |  /path_planning/plan_path  |
                 +-------------+---------------+
                               | 2. responde la ruta
                               |    (de vuelta a experiment_runner)
                               |
                               | 3. ADEMAS publica un JSON
                               |    con el resumen del intento
                               v
                 +---------------------------+
                 |   /path_planning/          |
                 |      plan_metrics          |
                 |      (topico, JSON)        |
                 +-------------+---------------+
                               | 4. metrics_logger esta
                               |    escuchando ese topico
                               v
                 +---------------------------+
                 |      metrics_logger        |
                 |  guarda cada fila en:      |
                 |  resultados_practica01.csv |
                 +---------------------------+
```

**El punto clave:** `experiment_runner` y `metrics_logger` nunca se hablan directamente. `experiment_runner` solo le pide rutas al planificador; el planificador, como efecto secundario de cada petición, avisa por el tópico qué pasó; `metrics_logger` solo está escuchando ese aviso. Por eso aunque apagues y repitas `experiment_runner` varias veces, `metrics_logger` sigue vivo y no se pierde ningún dato.

## Por qué "sin reiniciar nada"

Cambiar `diagonals` (A*) o `epsilon`/`N` (RRT) a mano se vería así:

```bash
ros2 param set /a_star_node diagonals true
```

`experiment_runner` hace exactamente eso, pero desde código (usando el servicio `set_parameters` de ROS2) — así puede probar muchas combinaciones de parámetros sin que haya que matar y volver a lanzar el nodo planificador cada vez.

## Qué trae el JSON del tópico `plan_metrics`

Ejemplo real, un intento de A*:

```json
{
  "algorithm": "a_star",
  "success": true,
  "exec_time_ms": 125.19,
  "num_points": 178,
  "start_x": 0.0, "start_y": 0.0,
  "goal_x": 8.0, "goal_y": 0.0,
  "diagonals": false
}
```

Es la "ficha" de ese intento: qué algoritmo, si encontró ruta, cuánto tardó, con qué parámetros se corrió. `metrics_logger` convierte cada uno de estos JSON en una fila del CSV (le agrega dos columnas propias: cuándo lo recibió, y una etiqueta manual opcional).

## Resultado final

Un solo archivo, `practica1/datos/resultados_practica01.csv`, con los resultados de A* y RRT juntos (se distinguen por la columna `algorithm`), listo para analizar en el reporte.

## Cómo se corre (resumen)

```bash
ros2 launch navig_utils practica1.launch.py algorithm:=a_star
ros2 run path_planner experiment_runner --ros-args -p phase:=a_star -p num_pruebas:=10
```
