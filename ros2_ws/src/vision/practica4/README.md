# Práctica 4 – Entrenamiento de una red neuronal

Robots Móviles 2027-1, FI-UNAM. Omar Rosario Carrasco.

Entrenamiento de una red totalmente conectada que clasifica dígitos manuscritos (paquete `handwritten_digits`) y comparación de su desempeño con las 64 combinaciones de tasa de aprendizaje (0.5, 1, 3, 10), épocas (3, 10, 50, 100) y tamaño de lote (5, 10, 30, 100), con etiquetas one-hot y binarias y con una y dos capas ocultas: 256 entrenamientos.

## Código

`../handwritten_digits/handwritten_digits/fc.py`: `feedforward`, `backpropagate` (con las derivadas obtenidas con SymPy), etiquetas binarias y modo de experimentos. Se corre desde su carpeta, porque busca el dataset en `../dataset`:

| Comando | Qué hace |
|---|---|
| `python3 fc.py` | Entrena la red 784-30-10 (η = 3, 10 épocas, lotes de 5) y muestra imágenes de prueba con su clasificación. Cualquier tecla muestra otra; ESC termina |
| `python3 fc.py --binary` | Lo mismo con las etiquetas en código binario (red 784-30-4) |
| `python3 fc.py --hidden 30,20 --epochs 50 --batch-size 10 --learning-rate 3` | Otra arquitectura y otros parámetros de entrenamiento |
| `python3 fc.py --check` | Compara `backpropagate` con el gradiente que se obtiene derivando la función de costo con SymPy en una red 4-3-2 |
| `python3 fc.py --experiments --csv archivo.csv` | Entrena las 64 combinaciones y guarda un renglón por combinación |

Requiere Python 3 con NumPy y OpenCV. SymPy es opcional: si no está instalado, el programa usa las mismas derivadas escritas a mano y avisa (`--check` sí lo necesita).

## Contenido

| Carpeta | Qué hay |
|---|---|
| `scripts/` | `run_experiments.sh` (lanza los cuatro barridos) y `analyze.py` (figuras y resumen; usa Matplotlib) |
| `datos/` | Un CSV por barrido, con 64 renglones, y `sistema.txt` (equipo y versiones con los que se corrió) |
| `resultados/` | Las figuras del reporte (`fig_*.png`) y `resumen.txt` |

Columnas de los CSV: etiquetas, arquitectura, tasa de aprendizaje, épocas, lote, semilla, tiempo de `train_by_SGD` en segundos, aciertos (‖t − y‖ < 0.5) en 100 imágenes de prueba al azar, aciertos en las 5000 de prueba, dígito decodificado en prueba, costo promedio en prueba y las mismas tres medidas con las 5000 imágenes de entrenamiento.

## Repetir los experimentos y el análisis

```bash
bash scripts/run_experiments.sh                                    # los cuatro barridos, uno tras otro (aprox. 1.7 h)
BARRIDOS="1 2" bash scripts/run_experiments.sh                     # solo algunos (1 a 4)
python3 scripts/analyze.py --dataset ../handwritten_digits/dataset # figuras y resumen en resultados/
```

`run_experiments.sh` se salta las combinaciones que ya están en los CSV, así que se puede interrumpir y volver a lanzar. Para repetir todo desde cero hay que mover antes los CSV de `datos/`.

Los datos que están en `datos/` se corrieron en una máquina virtual con dos núcleos, dos barridos a la vez (uno por núcleo), por lo que los tiempos sirven para comparar combinaciones entre sí y no como tiempos absolutos.
