# Práctica 2 (v2) — recolección de datos minimalista

Herramienta **nueva y separada** de `practica2/` (que ya está entregada con su reporte y sus 27 corridas del 13-oct). Esto no toca `practica2/datos`, `practica2/resultados`, `practica2/reporte` ni `run_experiments.sh` — vive en su propia carpeta y usa su propio CSV.

## Qué cambia frente a `run_experiments.sh`

| | `run_experiments.sh` (v1, ya entregado) | `experiment_runner.py` (v2, esta carpeta) |
|---|---|---|
| Entre corridas | reinicia `house_simul` + `motion_planning` completos | nada se reinicia: el controlador vuelve solo a esperar la siguiente meta |
| Cambiar α/β o Kd/Ka | genera una copia del launch con los valores pegados | `set_parameters` en caliente (como `experiment_runner` de la Práctica 1) |
| Dar la meta | `ros2 topic pub` como subproceso | nodo rclpy publica directo en `/goal_pose` |
| Saber que terminó | sondea el folder esperando un archivo más nuevo | se suscribe a `/navigation/goal_reached` |
| Métricas | se calculan después, aparte, con `analyze_path_following.py` sobre los `.txt` | se calculan al vuelo (mismo cross-track RMS) y quedan ya en el CSV |
| Duración de la corrida | tomada del log o repartida a ojo entre muestras | medida en el momento (`goal_reached` − publicar meta) |
| Salida | 27+ `.txt` crudos + `runs.csv` sin métricas + 5 scripts de análisis aparte | un solo `datos/resultados_seguimiento.csv` con todo |

## Requisitos (no se modifican, ya existen)

`pure_pursuit.py` y `stanley.py` (paquete `path_follower`) ya declaran sus parámetros (`alpha`/`beta` o `Kd`/`Ka`, y `folder`) y ya publican `/navigation/goal_reached` al terminar — esta herramienta solo los usa, no los toca.

## Uso

Terminal 1 (queda corriendo toda la sesión):
```bash
ros2 launch navig_utils motion_planning_experimentos.launch.py controller:=pure_pursuit
```
Espera a ver en el log `"Ready to execute new path"`.

Terminal 2:
```bash
ros2 run path_follower experiment_runner --ros-args -p controlador:=pure_pursuit -p num_repeticiones:=3
```

Para Stanley: Ctrl+C a la terminal 1, relanzar con `controller:=stanley`, y `-p controlador:=stanley` en la terminal 2.

Por defecto corre 2 rutas (R1, R2) × 7 combinaciones de parámetros × `num_repeticiones` — con el default de 3 son 42 corridas por controlador. Edita `ROUTES`, `PP_ALPHAS`/`PP_BETAS` o `ST_KAS`/`ST_KDS` arriba del archivo si quieres otros valores.

## Salida

- `datos/resultados_seguimiento.csv`: una fila por corrida — parámetros, ruta, repetición, duración medida, error transversal RMS y máx., velocidad media, `w` máxima, éxito/fallo.
- `datos/goal_*.txt` / `robot_*.txt`: los crudos de cada corrida (los mismos que ya generaba el controlador, solo que aquí en vez de en el home).

Para graficar o comparar, es un CSV normal — se puede abrir con pandas igual que `runs.csv`, pero ya no hace falta pasar por `analyze_path_following.py` para tener las métricas.
