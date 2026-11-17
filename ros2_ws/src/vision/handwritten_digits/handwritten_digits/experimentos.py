import csv
import random
from datetime import datetime
from pathlib import Path
from time import perf_counter
import argparse
import time

import numpy as np
from fc import FCNeuralNetwork, load_dataset

train_x, train_t, test_x, test_t = load_dataset("../dataset", binario=True)

# Convertir las etiquetas de 4 bits a dígitos del 0 al 9.
digitos = np.array([t.ravel() @ [8, 4, 2, 1] for t in test_t])

# Elegir las mismas 100 imágenes para todas las configuraciones.
rng = np.random.default_rng(42)
indices = [
    i
    for d in range(10)
    for i in rng.choice(np.flatnonzero(digitos == d), size=10, replace=False)
]

# Cada configuración tiene el orden: (tasa, épocas, tamaño del lote).
pruebas = (
    [(tasa, 3, 50) for tasa in [0.5, 1.0, 3.0, 10.0]]
    + [(1.0, epocas, 50) for epocas in [10, 50, 100]]
    + [(1.0, 3, lote) for lote in [5, 10, 30, 100]]
)

marca = datetime.now().strftime("%Y%m%d_%H%M%S")
ruta = Path.home() / f"resultados_binarios_{marca}.csv"
resultados = []

with ruta.open("w", newline="") as archivo:
    tabla = csv.writer(archivo)
    tabla.writerow([
        "tasa", "epocas", "lote", "segundos", "aciertos", "total", "porcentaje"
    ])

    for numero, (tasa, epocas, lote) in enumerate(pruebas, start=1):
        print(f"\nPrueba {numero}/{len(pruebas)}: "
              f"tasa={tasa}, epocas={epocas}, lote={lote}")

        random.seed(42)
        np.random.seed(42)
        nn = FCNeuralNetwork([784, 30, 4])

        inicio = perf_counter()
        nn.train_by_SGD(train_x, train_t, epocas, lote, tasa)
        segundos = perf_counter() - inicio

        aciertos = sum(
            np.linalg.norm(test_t[i] - nn.feedforward(test_x[i])[-1]) < 0.5
            for i in indices
        )
        total = len(indices)
        porcentaje = 100 * aciertos / total

        fila = (tasa, epocas, lote, round(segundos, 2), aciertos, total, porcentaje)
        tabla.writerow(fila)
        archivo.flush()
        resultados.append(fila)

print("\nRESUMEN: RED BINARIA DE 4 SALIDAS")
for tasa, epocas, lote, segundos, aciertos, total, porcentaje in resultados:
    print(f"Tasa {tasa:>4.1f} | Epocas {epocas:>3} | Lote {lote:>3} | "
          f"{segundos:>7.2f} s | {aciertos}/{total} ({porcentaje:.1f} %)")

print(f"\nResultados guardados en: {ruta}")