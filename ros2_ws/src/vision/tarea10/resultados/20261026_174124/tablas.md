# Tarea 10 — tablas de resultados

Tiempos: solo la detección de líneas (`HoughLines` / `HoughLinesP`), medidos con `get_clock().now()`; media ± desviación estándar, mediana y percentiles sobre los cuadros de cada corrida (se descartan los primeros 5). El p10 es el tiempo que no superan el 10 % de los cuadros más rápidos. Líneas/segmentos: mediana por cuadro.

## Detección de bordes (Canny)

| Corrida | Grupo | Umbral inferior | Umbral superior | Filtro gaussiano | Píxeles de borde |
|---|---|---:|---:|---|---:|
| C0 | PDF | 10 | 20 | 5×5 | 14.61 % |
| C1 | umbrales | 30 | 90 | 5×5 | 7.54 % |
| C2 | base | 50 | 150 | 5×5 | 4.68 % |
| C3 | umbrales | 100 | 200 | 5×5 | 3.60 % |
| C4 | umbrales | 150 | 300 | 5×5 | 1.91 % |
| C5 | filtro | 50 | 150 | sin filtro | 10.27 % |
| C6 | filtro | 50 | 150 | 3×3 | 6.26 % |
| C7 | filtro | 50 | 150 | 9×9 | 3.53 % |

## Transformada Hough: parámetros

| Corrida | Grupo | Canny (inf/sup) | Filtro | ρ (px) | θ (rad) | Umbral | Líneas |
|---|---|---|---|---:|---|---:|---:|
| H0 | PDF | 10/20 | sin filtro | 5 | 0.1 (5.73°) | 50 | 356 |
| HB | base | 50/150 | sin filtro | 1 | 0.0175 (1°) | 150 | 274 |
| HT1 | umbral | 50/150 | sin filtro | 1 | 0.0175 (1°) | 100 | 1466 |
| HT2 | umbral | 50/150 | sin filtro | 1 | 0.0175 (1°) | 200 | 13 |
| HT3 | umbral | 50/150 | sin filtro | 1 | 0.0175 (1°) | 250 | 1 |
| HA1 | θ | 50/150 | sin filtro | 1 | 0.0087 (0.5°) | 150 | 509 |
| HA2 | θ | 50/150 | sin filtro | 1 | 0.0349 (2°) | 150 | 158 |
| HA3 | θ | 50/150 | sin filtro | 1 | 0.1 (5.73°) | 150 | 57 |
| HR1 | ρ | 50/150 | sin filtro | 2 | 0.0175 (1°) | 150 | 2189 |
| HR2 | ρ | 50/150 | sin filtro | 5 | 0.0175 (1°) | 150 | 2654 |
| HC1 | Canny | 30/90 | sin filtro | 1 | 0.0175 (1°) | 150 | 771 |
| HC2 | Canny | 100/200 | sin filtro | 1 | 0.0175 (1°) | 150 | 24 |
| HF | filtro | 50/150 | 5×5 | 1 | 0.0175 (1°) | 150 | 20 |
| HS | reloj sim. | 50/150 | sin filtro | 1 | 0.0175 (1°) | 150 | 274 |
| HB2 | repetición | 50/150 | sin filtro | 1 | 0.0175 (1°) | 150 | 274 |

## Transformada Hough: tiempos de ejecución

| Corrida | Grupo | Cuadros | Píxeles de borde | t medio ± σ (ms) | Mediana (ms) | p10 (ms) | p95 (ms) | Mín–máx (ms) |
|---|---|---:|---:|---:|---:|---:|---:|---|
| H0 † | PDF | 95 | 55729 | 5.844 ± 3.706 | 3.948 | 3.217 | 12.314 | 2.650–12.806 |
| HB | base | 95 | 31123 | 15.208 ± 1.780 | 15.007 | 13.266 | 18.143 | 11.176–19.697 |
| HT1 | umbral | 95 | 31123 | 16.475 ± 1.843 | 16.213 | 14.394 | 19.668 | 13.539–21.397 |
| HT2 | umbral | 95 | 31123 | 13.173 ± 2.037 | 12.499 | 11.651 | 17.141 | 8.494–18.577 |
| HT3 | umbral | 95 | 31124 | 10.609 ± 1.348 | 10.707 | 8.764 | 12.270 | 7.863–14.741 |
| HA1 † | θ | 95 | 31123 | 162.006 ± 108.886 | 221.820 | 21.182 | 283.728 | 18.718–370.871 |
| HA2 | θ | 95 | 31123 | 4.641 ± 0.479 | 4.582 | 4.212 | 5.283 | 3.603–6.554 |
| HA3 | θ | 95 | 31123 | 1.744 ± 0.252 | 1.671 | 1.534 | 2.274 | 1.325–2.382 |
| HR1 | ρ | 95 | 31123 | 9.164 ± 1.430 | 8.823 | 7.809 | 11.914 | 7.415–14.087 |
| HR2 † | ρ | 95 | 31123 | 27.298 ± 31.485 | 8.229 | 6.922 | 83.428 | 6.679–92.331 |
| HC1 | Canny | 95 | 40805 | 14.187 ± 1.088 | 14.004 | 13.038 | 16.330 | 12.466–18.965 |
| HC2 | Canny | 95 | 20369 | 6.514 ± 0.750 | 6.301 | 6.002 | 8.145 | 5.549–9.772 |
| HF | filtro | 95 | 14765 | 6.099 ± 1.510 | 5.881 | 5.320 | 6.946 | 4.818–19.741 |
| HS ¹ | reloj sim. | 95 | 31123 | 0.000 ± 0.000 | 0.000 | 0.000 | 0.000 | 0.000–0.000 |
| HB2 † | repetición | 95 | 31123 | 66.152 ± 50.480 | 85.444 | 8.654 | 113.874 | 8.110–337.650 |

¹ Con `use_sim_time:=true` el reloj del nodo es el de la simulación (`/clock`).

† Corrida perturbada (p90/p10 > 2): tuvo bloques de cuadros mucho más lentos por la carga del sistema. Su media no representa al algoritmo; para comparar se usa el p10.

## Transformada Hough probabilística: parámetros

| Corrida | Grupo | Canny (inf/sup) | Filtro | ρ (px) | θ (rad) | Umbral | min_len | max_gap | Segmentos |
|---|---|---|---|---:|---|---:|---:|---:|---:|
| P0 | PDF | 10/20 | sin filtro | 5 | 0.1 (5.73°) | 50 | 1 | 1 | 10000 |
| PB | base | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 30 | 5 | 259 |
| PL1 | min_len | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 10 | 5 | 851 |
| PL2 | min_len | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 60 | 5 | 79 |
| PL3 | min_len | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 100 | 5 | 23 |
| PG1 | max_gap | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 30 | 1 | 76 |
| PG2 | max_gap | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 30 | 20 | 385 |
| PT1 | umbral | 50/150 | sin filtro | 1 | 0.0175 (1°) | 20 | 30 | 5 | 257 |
| PT2 | umbral | 50/150 | sin filtro | 1 | 0.0175 (1°) | 100 | 30 | 5 | 212 |
| PA1 | θ | 50/150 | sin filtro | 1 | 0.1 (5.73°) | 50 | 30 | 5 | 218 |
| PR1 | ρ | 50/150 | sin filtro | 5 | 0.0175 (1°) | 50 | 30 | 5 | 177 |
| PF | filtro | 50/150 | 5×5 | 1 | 0.0175 (1°) | 50 | 30 | 5 | 115 |
| PB2 | repetición | 50/150 | sin filtro | 1 | 0.0175 (1°) | 50 | 30 | 5 | 262 |

## Transformada Hough probabilística: tiempos de ejecución

| Corrida | Grupo | Cuadros | Píxeles de borde | t medio ± σ (ms) | Mediana (ms) | p10 (ms) | p95 (ms) | Mín–máx (ms) |
|---|---|---:|---:|---:|---:|---:|---:|---|
| P0 | PDF | 95 | 55680 | 9.178 ± 0.690 | 8.948 | 8.518 | 10.350 | 7.968–11.758 |
| PB | base | 95 | 31123 | 18.473 ± 2.320 | 18.046 | 16.336 | 22.423 | 15.418–31.386 |
| PL1 | min_len | 95 | 31123 | 22.633 ± 2.165 | 21.951 | 20.631 | 26.989 | 19.974–31.208 |
| PL2 | min_len | 95 | 31123 | 15.295 ± 1.612 | 14.654 | 13.853 | 18.127 | 13.396–21.989 |
| PL3 | min_len | 95 | 31123 | 14.043 ± 1.831 | 13.326 | 12.678 | 17.748 | 11.694–21.355 |
| PG1 | max_gap | 95 | 31123 | 16.143 ± 1.836 | 15.473 | 14.572 | 19.492 | 13.848–24.025 |
| PG2 | max_gap | 95 | 31123 | 23.773 ± 3.388 | 22.896 | 20.511 | 30.051 | 19.857–39.326 |
| PT1 | umbral | 95 | 31123 | 13.581 ± 1.593 | 13.187 | 12.183 | 16.704 | 11.727–20.447 |
| PT2 | umbral | 95 | 31123 | 22.669 ± 2.303 | 22.064 | 20.590 | 26.772 | 19.677–31.424 |
| PA1 | θ | 95 | 31123 | 4.838 ± 0.380 | 4.786 | 4.413 | 5.423 | 3.923–5.786 |
| PR1 | ρ | 95 | 31123 | 10.280 ± 0.897 | 9.935 | 9.514 | 12.129 | 9.218–13.560 |
| PF | filtro | 95 | 14765 | 9.940 ± 1.002 | 9.632 | 9.085 | 11.479 | 8.805–14.618 |
| PB2 | repetición | 95 | 31123 | 18.497 ± 1.871 | 18.081 | 16.729 | 21.231 | 15.785–26.082 |
