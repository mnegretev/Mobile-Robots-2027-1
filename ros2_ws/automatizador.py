import subprocess
import math
import re
import time
import csv

# Diseño de 20 experimentos para evaluar el desempeño
pruebas = [
    # --- GRUPO 0: Prueba Base ---
    {"N": 100, "s2s": 0.1,  "s2m": 0.05, "s2r": 0.05}, 

    # --- GRUPO 1: Variando el número de partículas (N) ---
    {"N": 10,  "s2s": 0.1,  "s2m": 0.05, "s2r": 0.05},
    {"N": 50,  "s2s": 0.1,  "s2m": 0.05, "s2r": 0.05},
    {"N": 250, "s2s": 0.1,  "s2m": 0.05, "s2r": 0.05},
    {"N": 500, "s2s": 0.1,  "s2m": 0.05, "s2r": 0.05},

    # --- GRUPO 2: Variando la varianza del sensor (s2s) ---
    {"N": 100, "s2s": 0.01, "s2m": 0.05, "s2r": 0.05},
    {"N": 100, "s2s": 0.5,  "s2m": 0.05, "s2r": 0.05},
    {"N": 100, "s2s": 1.0,  "s2m": 0.05, "s2r": 0.05},
    {"N": 100, "s2s": 5.0,  "s2m": 0.05, "s2r": 0.05},

    # --- GRUPO 3: Variando el ruido del modelo de movimiento (s2m) ---
    {"N": 100, "s2s": 0.1,  "s2m": 0.01, "s2r": 0.05},
    {"N": 100, "s2s": 0.1,  "s2m": 0.1,  "s2r": 0.05},
    {"N": 100, "s2s": 0.1,  "s2m": 0.5,  "s2r": 0.05},
    {"N": 100, "s2s": 0.1,  "s2m": 1.0,  "s2r": 0.05},

    # --- GRUPO 4: Variando la dispersión del remuestreo (s2r) ---
    {"N": 100, "s2s": 0.1,  "s2m": 0.05, "s2r": 0.01},
    {"N": 100, "s2s": 0.1,  "s2m": 0.05, "s2r": 0.1},
    {"N": 100, "s2s": 0.1,  "s2m": 0.05, "s2r": 0.5},
    {"N": 100, "s2s": 0.1,  "s2m": 0.05, "s2r": 1.0},

    # --- GRUPO 5: Casos combinados y valores por defecto ---
    {"N": 500, "s2s": 0.5,  "s2m": 0.1,  "s2r": 0.1}, 
    {"N": 50,  "s2s": 0.05, "s2m": 0.01, "s2r": 0.01},
    {"N": 100, "s2s": 1.0,  "s2m": 1.0,  "s2r": 1.0}   
]

MAX_ITERACIONES = 30
archivo_csv = "resultados_practica3.csv"

# Inicializar el archivo CSV con los encabezados
with open(archivo_csv, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Prueba', 'N', 's2s', 's2m', 's2r', 'Iteraciones', 'Estado', 'Error_Dist_m', 'Error_Ang_rad'])

for i, p in enumerate(pruebas):
    cmd = [
        "stdbuf", "-oL",
        "ros2", "run", "particle_filter", "pf",
        "--ros-args", "-p", "use_sim_time:=True",
        "-p", f"N:={p['N']}", "-p", f"s2s:={p['s2s']}",
        "-p", f"s2m:={p['s2m']}", "-p", f"s2r:={p['s2r']}"
    ]
    
    print(f"\n========================================================")
    print(f"--- Iniciando Prueba {i+1}/{len(pruebas)} ---")
    print(f"Parámetros: N={p['N']}, s2s={p['s2s']}, s2m={p['s2m']}, s2r={p['s2r']}")
    
    proceso = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    
    iteracion = 0
    pose_real = None
    pose_est = None
    estado_final = "No Convergió"
    error_dist = 0.0
    error_ang = 0.0
    
    for linea in proceso.stdout:
        linea_min = linea.lower()
        numeros = [float(n) for n in re.findall(r'[-+]?\d*\.\d+|\d+', linea)]
        
        if len(numeros) >= 3:
            if "groundtruth" in linea_min:
                pose_real = numeros[-3:]
            elif "estimated" in linea_min:
                pose_est = numeros[-3:]
        
        if pose_real and pose_est:
            iteracion += 1
            
            error_dist = math.sqrt((pose_est[0] - pose_real[0])**2 + (pose_est[1] - pose_real[1])**2)
            d_theta = pose_est[2] - pose_real[2]
            error_ang = abs(math.atan2(math.sin(d_theta), math.cos(d_theta)))
            
            print(f"  ---> Iteración {iteracion}/{MAX_ITERACIONES} | Error Dist: {error_dist:.3f}m | Error Ang: {error_ang:.3f}rad")
            
            if error_dist < 0.20 and error_ang < 0.20:
                print(f"✅ Localizado exitosamente en iteración {iteracion}.")
                estado_final = "Localizado"
                proceso.terminate()
                break
                
            if iteracion >= MAX_ITERACIONES:
                print(f"❌ Abortando: No convergió en {MAX_ITERACIONES} iteraciones.")
                proceso.terminate()
                break
                
            pose_real = None
            pose_est = None
            
    proceso.wait()
    
    # Guardar los resultados de esta prueba en el CSV
    with open(archivo_csv, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([i+1, p['N'], p['s2s'], p['s2m'], p['s2r'], iteracion, estado_final, round(error_dist, 3), round(error_ang, 3)])
        
    print("Guardado en CSV. Reiniciando entorno en 3 segundos...")
    time.sleep(3)