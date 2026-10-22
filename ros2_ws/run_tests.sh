#!/bin/bash

# Define los arreglos con las combinaciones de parámetros para cada prueba
# Prueba:        1 (Base)  2 (N Alto)  3 (N Bajo)  4 (s2s Bajo)  5 (s2s Alto)
N_vals=(         100       500         50          100           100)
s2s_vals=(       1.0       1.0         1.0         0.1           5.0)
s2m_vals=(       1.0       1.0         1.0         1.0           1.0)
s2r_vals=(       1.0       1.0         1.0         1.0           1.0)

# Tiempo de duración de cada prueba en segundos (ajústalo según cuánto tardas en mover el robot)
DURATION=45

for i in ${!N_vals[@]}; do
    N=${N_vals[$i]}
    s2s=${s2s_vals[$i]}
    s2m=${s2m_vals[$i]}
    s2r=${s2r_vals[$i]}
    
    LOG_FILE="log_N${N}_s2s${s2s}_s2m${s2m}_s2r${s2r}.txt"
    echo "--------------------------------------------------------"
    echo "Iniciando prueba $((i+1)) de ${#N_vals[@]}: N=$N, s2s=$s2s, s2m=$s2m, s2r=$s2r"
    echo "Guardando en: $LOG_FILE"
    echo "¡Mueve el robot en la GUI ahora!"
    
    # Ejecuta el nodo por el tiempo definido y redirige la salida estándar al log
    timeout ${DURATION}s ros2 run particle_filter pf --ros-args -p use_sim_time:=True -p N:=$N -p s2s:=$s2s -p s2m:=$s2m -p s2r:=$s2r > "$LOG_FILE"
    
    echo "Prueba finalizada. Deteniendo filtro para reiniciar las partículas en la siguiente iteración..."
    sleep 3
done

echo "Todas las pruebas automatizadas han concluido."
