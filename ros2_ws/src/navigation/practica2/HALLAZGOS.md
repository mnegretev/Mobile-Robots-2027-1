# Práctica 2 – Resultados de los experimentos (27 corridas + verificación cinemática)

Carpeta analizada: `~/practica2_exp/20261013_121056`. v_max = 0.5 m/s, w_max = 1.0 rad/s en todas las corridas.
Rutas planeadas con A* + suavizado: **R1** meta (4.47, −0.76), 4.53 m, casi recta con una curva suave al final; **R2** meta (0, 5), 5.74 m, forma de "S" con ~1 m de excursión lateral.
Estado: 27 de 28 combinaciones completas. Falta sólo Stanley Kd=0.5, Ka=1 en R2, que **no termina** (ver §3).

## 1. Qué se midió (definiciones para el marco teórico)

| Métrica | Definición | Comparable PP vs Stanley |
|---|---|---|
| Error transversal RMS (m) | distancia de cada pose a la polilínea de la ruta, raíz de la media cuadrática, sólo en la fase de seguimiento | sí (métrica principal) |
| Error transversal máx. (m) | máximo después de 0.3 m recorridos (quita el desfase de arranque, ~0.045 m) | sí |
| Tiempo de seguimiento (s) | desde que arranca hasta la última muestra con v > 0 (no incluye el giro final en sitio) | sí |
| Variación total de ω a 5 Hz (rad/s) | Σ\|Δω\| de ω promediada en ventanas de 0.2 s | sí (agitación del control) |
| ω RMS, cruces de ω por cero | sobre las muestras originales | **no**: dependen de la frecuencia de muestreo |
| Error de posición final (m) | distancia del último punto a la meta | **no discrimina**: el lazo termina al estar a `tol_dist = 0.3 m` |

## 2. Resultados principales (1 corrida por celda)

**Efecto de cada parámetro** (figuras `fig_barrido_pure_pursuit.png`, `fig_barrido_stanley.png`):

| Algoritmo / parámetro | R1: error RMS | R2: error RMS | Tendencia |
|---|---|---|---|
| PP, β = 0.1 / 0.4 / 1.0 / 4.0 (α = 1) | 0.010 / 0.016 / 0.059 / 0.108 m | 0.042 / 0.067 / 0.132 / 0.260 m | β grande: ω suave pero se corta camino; β pequeño: preciso pero ω agitada (var. ω en R2: 6.4 → 3.1 → 1.7 → 1.1) |
| PP, α = 0.1 / 0.5 / 1 / 4 (β = 0.4) | 0.017 / 0.019 / 0.016 / 0.021 m | 0.032 / 0.055 / 0.067 / 0.089 m | α pequeño frena más al desalinearse: más preciso y más lento (R2: 19.3 s → 14.5 → 14.5 → 13.3 s). En R1 apenas importa |
| Stanley, Ka = 0.5 / 1 / 2 / 5 (Kd = 1) | 0.063 / 0.042 / 0.032 / 0.029 m | 0.197 / 0.168 / 0.128 / 0.135 m | mejora hasta Ka ≈ 2; después la ω se agita mucho (var. ω en R2: 2.6 → 4.0 → 5.6 → 11.3) |
| Stanley, Kd = 0.5 / 1 / 2 / 5 (Ka = 1) | 0.061 / 0.042 / 0.047 / 0.031 m | no termina / 0.168 / 0.112 / 0.062 m | Kd reduce el error más que Ka en R2 (−63 % de 1 a 5) con menos costo en agitación |

Caso extremo, PP β = 4 en R2: error RMS 0.26 m, máx 0.44 m, 37 s de seguimiento y v media 0.18 m/s. Con β = 4 la ganancia efectiva es 0.125 rad/s por rad: ω se queda en ≈ 0.2 rad/s, el error angular sigue grande y, como v = v_max·exp(−e²/α), la velocidad cae a ≈ 0.05 m/s durante ~25 s. Además la rotación final del PP usa β: dura 28 s (la duración total de esa corrida es 64.7 s). Por eso el reporte debe usar el **tiempo de seguimiento**, no la duración total.

**Comparación entre algoritmos** (`fig_algoritmos.png`, `comparacion_algoritmos.md`, `fig_cmp_base_R1.png`, `fig_cmp_base_R2.png`):

| Ruta | Config. | Error RMS (m) | Tiempo seg. (s) | Var. ω 5 Hz |
|---|---|---|---|---|
| R1 | PP base (α=1, β=0.4) | 0.016 | 10.1 | 0.92 |
| R1 | Stanley base (Kd=1, Ka=1) | 0.042 | 9.6 | 1.10 |
| R1 | PP mejor RMS (α=1, β=0.1) | 0.010 | 13.1 | 2.99 |
| R1 | Stanley mejor RMS (Kd=1, Ka=5) | 0.029 | 9.6 | 4.01 |
| R2 | PP base | 0.067 | 14.5 | 3.12 |
| R2 | Stanley base | 0.168 | 18.2 | 3.98 |
| R2 | PP mejor RMS (α=0.1, β=0.4) | 0.032 | 19.3 | 2.91 |
| R2 | Stanley mejor RMS (Kd=5, Ka=1) | 0.062 | 15.1 | 5.70 |

Lectura: con estos ajustes **pure pursuit sigue mejor la ruta que Stanley en las dos rutas** (entre 1.9× y 2.9× menos error RMS según la pareja de configuraciones), pero cada mejora de precisión cuesta tiempo (α pequeño) o agitación de ω (β pequeño, Ka/Kd grandes). Stanley base en R2 se queda hasta ~0.3 m fuera de la ruta en la segunda mitad y tarda más; subir Kd lo corrige.

## 3. Verificación con un modelo cinemático ideal (`kinematic_check.py`)

Se simuló, sin Gazebo, cada corrida con las mismas leyes de control, la misma ruta y la misma pose inicial (robot diferencial ideal, sin aceleración finita ni retardos). Resultado (`fig_validacion_cinematica.png`, `validacion_cinematica.csv`):

- 27/27 corridas reproducidas; correlación 0.98 entre el error RMS simulado y el medido; error relativo mediano 8 % (pure pursuit 6 %, Stanley 11 %). La mayor diferencia es Stanley Ka=5 en R2 (medido 0.135 m, simulado 0.076 m): en el robot real la ω agitada y los retardos empeoran el seguimiento, que el modelo ideal no ve.
- **El error depende de la ley de control, no del ruido de Gazebo.** Esto respalda las comparaciones de §2 aunque haya una sola corrida por celda.
- **Frecuencia de control:** repitiendo Stanley en el modelo a 57 Hz en lugar de 5.7 Hz, el error RMS casi no cambia (p. ej. base R2: 0.150 → 0.149 m; Kd=5: 0.060 → 0.061 m). Por tanto la ventaja de pure pursuit no se debe a que Stanley corra más lento. (Se reproduce con `--st-dt 0.0175`.)
- **Stanley Kd=0.5, Ka=1 en R2 (no termina en 150 s ni en 300 s):** el modelo reproduce el fallo. El robot llega a 0.31 m de la meta a los ~18 s, justo fuera de la tolerancia (0.3 m); después el control lo aleja de la ruta y se estaciona a ≈1 m con v ≈ 0, porque v = v_max·exp(−5(e_t² + α²)) se anula y Ka·α + Kd·e_t ≈ 0 (un equilibrio espurio). Es una hipótesis coherente con el modelo; no se confirmó en Gazebo porque los nodos sólo guardan la pose al terminar.

## 4. Interpretación (para Discusión)

- En pure pursuit, para errores pequeños ω ≈ (w_max / 2β)·e_a, es decir, un controlador proporcional con ganancia w_max/(2β) (β=0.1 → 5, 0.4 → 1.25, 1 → 0.5, 4 → 0.125 rad/s por rad). Es la misma idea que Ka en Stanley, por eso ambos muestran el mismo compromiso precisión ↔ agitación.
- Stanley (versión del curso) es un P sobre θ y e_t **sin término de curvatura** ni manejo del final de la ruta. En un tramo curvo, para producir la ω que exige la curva debe existir un error de equilibrio ≈ κ·v / Kd; por eso en R2 el error baja mucho con Kd y poco con Ka. (Hipótesis consistente con los datos y con el modelo.)
- α no actúa sobre la dirección sino sobre v: sólo importa cuando el robot está desalineado (R2); en R1 los ángulos de error son pequeños y α casi no cambia el error.

## 5. Limitaciones que hay que declarar en el reporte

1. **Una sola corrida por celda.** Del par de réplicas que existe (PP α=0.1, β=0.4, R1: error RMS 0.0135 vs 0.0166 m; tiempo 10.1 vs 11.2 s) el ruido es ≈ 0.003 m en RMS y ≈ 10 % en tiempo. Diferencias menores a eso (p. ej. PP R1 α=0.1 vs 1) no son concluyentes. El acuerdo con el modelo cinemático (§3) indica que diferencias mayores a ~15 % sí son robustas.
2. **Los tiempos son estimados:** los .txt no traen marca de tiempo; se repartió la duración (tomada del log) uniformemente sobre las muestras.
3. **Los barridos de parámetros no son equivalentes** entre algoritmos (α, β vs Kd, Ka tienen unidades y efectos distintos), así que "mejor RMS" es la mejor del barrido, no un óptimo global.
4. **Una combinación sin resultado** (Stanley Kd=0.5, Ka=1, R2): se reporta como "no converge" con la explicación de §3.
5. Las dos rutas son de un solo mapa y una sola pose inicial; no se probaron rutas con giros cerrados ni obstáculos más cercanos.

## 6. Qué sigue

- Si el profesor pidió "4 iteraciones" como **repeticiones**: `-r 4` sobre las configuraciones que se vayan a reportar (base y mejor de cada algoritmo, ambas rutas).
- Marco teórico: derivar ω ≈ (w_max/2β)·e_a, el compromiso precisión-agitación, la ley de Stanley adaptada y la métrica de error transversal; citar fuentes arbitradas para Stanley (Hoffmann et al., ACC 2007) y distinguir las que no lo son (Snider 2009 y Coulter 1992 son reportes técnicos de CMU); verificar datos bibliográficos en tu biblioteca antes de citar.
