# Tarea 10 – Extracción de características geométricas

Robots Móviles 2027-1, FI-UNAM. Omar Rosario Carrasco.

Pruebas automáticas de los detectores del paquete `geometric_features` (ROS 2 Jazzy + Gazebo): detector de bordes de Canny, Transformada Hough y Transformada Hough probabilística. Se varía **un parámetro a la vez** respecto a una configuración base, con el robot quieto, para comparar la calidad de la detección y el tiempo de ejecución (paso 11 de la tarea).

## Código evaluado

`../geometric_features/geometric_features/`: `canny.py`, `hough.py` y `houghP.py`.

| Parámetro | Nodos | Qué hace |
|---|---|---|
| `canny_l`, `canny_u` | los tres | umbrales inferior y superior de Canny |
| `rho`, `theta`, `hough_th` | hough, houghP | resolución del acumulador (px y rad) y umbral de votos |
| `min_len`, `max_gap` | houghP | longitud mínima del segmento y hueco máximo para unir tramos |
| `blur` | los tres | kernel del filtro gaussiano previo a Canny (impar). Por defecto: 5 en canny, 0 (sin filtro) en hough y houghP |
| `n_avg` | hough, houghP | cuadros del promedio móvil del tiempo que se muestra en la imagen (20 por defecto) |
| `log_dir`, `n_frames` | los tres | modo experimento: guarda los datos de `n_frames` cuadros y las imágenes en `log_dir` (lo usa el script) |

El tiempo que se reporta es **solo el de la detección de líneas** (`cv2.HoughLines` / `cv2.HoughLinesP`), medido con `get_clock().now()` antes y después de la llamada.

## Contenido

| Carpeta | Qué hay |
|---|---|
| `scripts/` | `run_experiments.sh` / `t10_runner.py` (corridas automáticas en Gazebo), `t10_camara.py` (pone la cámara del brazo mirando al frente), `analyze_t10.py` (tablas, figuras y capturas) |
| `datos/<lote>/` | Una carpeta por corrida (`frames.csv` con tiempo, líneas y píxeles de borde por cuadro; `params.json`; `original.png`, `bordes.png`, `resultado.png`; `node.log`), más `corridas.csv`, `sistema.json` (incluye la pose del brazo), `vista_inicial.png` y `codigo/` (copia de los nodos usados) |
| `resultados/<lote>/` | `tablas.md`, `resumen_corridas.csv`, figuras `fig_*.png`, mosaicos y `capturas/` |

Las imágenes de `datos/`, las capturas y los mosaicos no se suben al repo (`.gitignore`) para no inflarlo; los CSV, JSON, tablas y figuras sí.

## Correr las pruebas (con Gazebo)

Cada uno en su propia terminal **bash** de la distrobox (`distrobox enter rosjj -- bash`):

```bash
# T1: simulación (desde un workspace donde house_simul ya esté compilado). No muevas el robot.
ros2 launch house_simul house_simul.launch.py realsense:=True

# T2: pruebas, sin ningún canny/hough/houghP abierto
cd ros2_ws/src/vision/tarea10/scripts
bash run_experiments.sh -n          # muestra el plan (36 corridas)
bash run_experiments.sh -m 2        # prueba de humo (2 corridas)
bash run_experiments.sh             # todo, ~10 min con la cámara a 10 Hz
bash run_experiments.sh --continuar # reanuda el último lote si se interrumpió
```

### La cámara del brazo

La RealSense va en la muñeca del xArm6 (`link6`) y mira a lo largo de su eje z: en la pose de arranque (todas las articulaciones en 0) apunta al piso. Antes de las pruebas, el script manda una trayectoria a `/xarm6_traj_controller/joint_trajectory` para que mire al frente y espera a que `/joint_states` llegue a la pose. Con `joint1 = joint4 = joint6 = 0`, la inclinación de la cámara solo depende de `q2 + q3 + q5` (inclinación = −(q2 + q3 + q5) − π/2), así que basta `q5 = −π/2 − (q2 + q3)`.

| Opción | Articulaciones (rad) | Cámara |
|---|---|---|
| `--pose frente` (por defecto) | q5 = −1.571 | horizontal, a ~0.72 m sobre `base_link` |
| `--pose alta` | q2 = −0.3, q3 = −1.2, q5 = −0.071 | horizontal, a ~1.12 m |
| `--pose inicial` | todas en 0 | apunta al piso (pose de arranque) |
| `--pose ninguna` | — | no mueve el brazo |
| `--inclinacion G` | ajusta q5 (y q3 si q5 llega a su límite) | G grados hacia arriba (negativo: hacia abajo) |

También se puede usar sola: `python3 scripts/t10_camara.py [--pose alta] [--inclinacion 5] [--foto vista.png]` (con la simulación corriendo y después de `source ~/.cache/tarea10_ws/install/local_setup.bash`).

`run_experiments.sh` compila el `geometric_features` de **este** repo y hace `source` de él. `build/`, `install/` y `log/` quedan en el disco de Linux (`~/.cache/tarea10_ws`, o lo que diga `T10_BUILD_DIR`), porque `/mnt/Datos` no admite los enlaces simbólicos que usa `--symlink-install`. Luego revisa que el código que va a correr `ros2 run` sea el de este repo, pone la cámara en la pose elegida (sección anterior), revisa que la cámara publique (`/camera/color/image_raw`) y que no haya detectores abiertos, y guarda `vista_inicial.png`. Después, por cada configuración, lanza el nodo, espera a que registre los cuadros, lo detiene con Ctrl+C y anota la corrida en `corridas.csv`. Las ventanas de OpenCV se abren y cierran solas: no las cierres a mano.

### Plan

| Grupo | Corridas | Qué se varía (todo lo demás en la base) |
|---|---|---|
| Canny (base 50/150, filtro 5×5) | C0–C7 | umbrales 10/20 (PDF), 30/90, 50/150, 100/200, 150/300; filtro sin, 3×3, 5×5, 9×9 |
| Hough (base: Canny 50/150, ρ=1, θ=1°, umbral 150) | H0–HB2 | comando del PDF; umbral 100–250; θ 0.5°, 2°, 5.7°; ρ 2 y 5; Canny 30/90 y 100/200; filtro 5×5; `use_sim_time:=true`; repetición de la base |
| Hough P (base: Canny 50/150, ρ=1, θ=1°, umbral 50, min_len 30, max_gap 5) | P0–PB2 | comando del PDF; min_len 10, 60, 100; max_gap 1, 20; umbral 20, 100; θ 5.7°; ρ 5; filtro 5×5; repetición de la base |

Canny registra 20 cuadros por corrida y Hough / Hough P 100 (`--frames N` lo cambia). La corrida `HS` comprueba qué mide `get_clock().now()` cuando el nodo usa el reloj de la simulación.

## Análisis (sin Gazebo)

```bash
python3 scripts/analyze_t10.py datos/<lote>          # resultados en resultados/<lote>/
```

Descarta los primeros 5 cuadros de cada corrida (`--warmup`) y calcula media, desviación estándar, mediana, p95, mínimo y máximo del tiempo; genera las tablas de parámetros y tiempos, las figuras (efecto de cada parámetro, tiempo vs. votos, distribución de tiempos), los mosaicos y una captura original | resultado por corrida. Requiere Python 3 con NumPy y Matplotlib.
