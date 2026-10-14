# Metodologia: automatizacion de Practica 01 con IA

El profesor pidio explicitamente documentar COMO se genero esta
automatizacion con ayuda de IA (que prompts, cuales funcionaron) y
dejar el esquema para que el resto del grupo lo pueda replicar - no
solo subir el codigo.

**Estado: borrador.** Falta confirmar que el codigo corre sin errores
(`colcon build` + prueba real con `house_simul`) antes de darlo por
cerrado - asi lo pidio el profesor: primero que funcione, luego se
documenta en limpio.

## Por que funciono (vs. un intento sin contexto)

El profesor senalo que un intento anterior con IA no funciono porque
la IA "no tenia contexto de la maquina". La diferencia en este caso:
antes de pedir el codigo, la IA tuvo acceso a:

1. El enunciado completo de la Practica 01 (el PDF del profesor).
2. El repositorio del curso (`github.com/mnegretev/Mobile-Robots-2027-1`)
   clonado para ver la plantilla real de `a_star.py` / `rrt.py`.
3. Mi clon local real (via el puente a mi computadora), en mi rama
   `rosario_carrasco`, donde A* y RRT ya estaban implementados y mi
   script `experimentos_practica1.sh` ya funcionaba - la IA leyo mi
   codigo real y mi propio enfoque en Bash antes de proponer el
   cambio a ROS2, en vez de adivinar sobre una plantilla vacia.
4. Una descripcion clara de que cambiar: el servicio
   `/path_planning/plan_path` ya existe y ya lo uso con la GUI y con
   Bash; lo que faltaba era que el tiempo de ejecucion saliera por
   ROS2 (topico) en vez de tener que leerlo de los logs con `grep`.

## Prompt 1 (genero la automatizacion completa)

```
Necesitamos hacer un arreglo al código final del siguiente Git.
https://github.com/mnegretev/Mobile-Robots-2027-1
Necesitamos hacer unos ajustes al código, para ejecutar pruebas automaticamente, usaremos ROS2 Jazzy Jay.
Te proveeré de las siguientes fuentes de información que te ayudaran a refinar el código.
Además de libros para que sepas como funciona la estructura de ROS2.
Con todo esto te dare las instrucciones del programa requerido.

Necesitamos que automatices la recolección de datos de las practica 1. La planeación de Rutas se hace llamar por un servicio, hice el código de Bash, pero ahora lo necesito internamente en ROS2, no necesitas que reiniciar nada. Mandas a llamar un servicio que publique el tiempo de ejecución internamente en ROS2.

Recolecta los datos de a_star y rrt, haz otro nodo de ROS2 que publiquen los tiempos de ejecución y finalmente generar una tabla comparativa según los aspectos vistos en la practica.

Te daré el contexto de la practica 1 mediante un documento PDF, solo centraté en la programación y los aspectos tecnicos para dar origen al código y la modificación del repositorio.
```

Adjunto: el PDF de la Practica 01 (los libros de referencia de ROS2 y
de buenas practicas de programacion se subieron como contexto general;
no hizo falta leerlos completos para esta parte puntual).

### Que genero, sin retrabajo:

- Modifico `a_star.py` y `rrt.py`: justo donde ya calculaban
  `delta_ms` (tiempo de ejecucion) y solo lo mandaban a
  `get_logger().info(...)`, ahora tambien lo publican como JSON
  (algoritmo, exito, tiempo, parametros, punto meta) en un topico
  nuevo `/path_planning/plan_metrics`. No toco la logica de los
  algoritmos.
- `experiment_runner` (nodo nuevo): cliente del servicio
  `/path_planning/plan_path`, reconfigura `diagonals` / `epsilon` /
  `N` del nodo planificador **en caliente** (`set_parameters`, el
  equivalente interno de `ros2 param set`) para no reiniciarlo entre
  combinaciones - y llama al servicio con `rclpy` en vez de invocar
  `ros2 service call` por CLI.
- `metrics_logger` (nodo nuevo): escucha `/path_planning/plan_metrics`
  y arma el CSV; expone `/path_planning/generate_table` para generar
  la tabla comparativa cuando se quiera, sin parar nada.

## Prompt 2 (launch universal, tras la explicacion del profesor)

```
Ok chum refina el código con la explicacion que me dio el profesor,
ademas genera un launch universal para la practica 1
```

Adjunto: nota de voz con la explicacion del profesor (resumen abajo).

### Que genero:

- Confirmo que la arquitectura ya coincidia con lo que pidio el
  profesor (no hizo falta cambiar la logica, solo verificar punto por
  punto contra la explicacion).
- `navig_utils/launch/practica1.launch.py`: un solo `ros2 launch` que
  levanta todo (simulacion, localizacion, mapa de costo, el
  planificador elegido, y `metrics_logger`), parametrizado por
  argumentos (`algorithm`, `diagonals`, `epsilon`, `N`,
  `inflation_radius`, `cost_radius`, `use_gui`).

## Resumen de lo que pidio el profesor (nota de voz)

- Modificar cada nodo planificador (A* y RRT) para que publique su
  propio tiempo de ejecucion en un topico de ROS2 - no usar Bash ni
  leer logs para medirlo.
- Que la recoleccion (barrer puntos meta y parametros) sea OTRO nodo
  de ROS2, no un script de Bash - para no tener que reiniciar nada
  entre pruebas.
- No basta con subir el codigo: documentar en un Markdown como se
  genero con IA, que prompts se usaron, cual funciono, y el esquema
  para que el resto del grupo lo pueda replicar.
- Primero debe quedar funcionando (prueba real); el documento se
  cierra/limpia despues de confirmar eso.

## Esquema para replicar el patron

```
house_simul + (map_server/amcl, localizacion)
        |
        v
    cost_map --(/get_inflated_map, /get_cost_map)--> a_star_node o rrt_node
                                                              |
                                   /path_planning/plan_path (servicio)
                                                              ^
                                                              |
                                          experiment_runner --+
                                          (barre puntos meta;
                                           set_parameters para
                                           diagonals/epsilon/N
                                           sin reiniciar nada)
                                                              |
                                /path_planning/plan_metrics (topico, JSON)
                                                              |
                                                              v
                                                      metrics_logger
                                                              |
                                                CSV + tabla comparativa (MD)
```

El patron general, para repetirlo en otra practica: **el nodo que
calcula publica su propio tiempo en un topico** + **un nodo cliente
aparte que automatiza las llamadas y reconfigura parametros sin
reiniciar** + **un nodo que solo escucha y acumula**.
