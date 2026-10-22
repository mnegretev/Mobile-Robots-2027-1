import math
import re
import glob

def analizar_resultados():
    archivos_log = glob.glob("log_*.txt")
    
    if not archivos_log:
        print("No se encontraron archivos de log (log_*.txt) en este directorio.")
        return
        
    for archivo in sorted(archivos_log):
        print(f"--- {archivo} ---")
        
        with open(archivo, 'r') as f:
            lineas = f.readlines()
            
        iteracion = 0
        localizado = False
        pose_real = None
        pose_est = None
        
        for linea in lineas:
            linea_min = linea.lower()
            
            # Buscar todos los números (enteros o decimales, positivos o negativos) en la línea
            numeros = [float(n) for n in re.findall(r'[-+]?\d*\.\d+|\d+', linea)]
            
            if len(numeros) >= 3:
                # Asumimos que los últimos 3 números corresponden a (x, y, theta)
                if "real" in linea_min:
                    pose_real = numeros[-3:]
                elif "estim" in linea_min:
                    pose_est = numeros[-3:]
                    
            # Si ya leímos ambas poses en un ciclo, evaluamos el error
            if pose_real and pose_est:
                iteracion += 1
                
                # Cálculo de error de distancia Euclidiana
                error_dist = math.sqrt((pose_est[0] - pose_real[0])**2 + (pose_est[1] - pose_real[1])**2)
                
                # Cálculo de error angular normalizado entre -PI y PI
                d_theta = pose_est[2] - pose_real[2]
                error_ang = abs(math.atan2(math.sin(d_theta), math.cos(d_theta)))
                
                # Umbrales: 15 cm de posición y ~5.7 grados (0.1 rad) de orientación
                if error_dist < 0.15 and error_ang < 0.1:
                    print(f"✅ Localizado en iteración: {iteracion}")
                    print(f"   Posición Real: x={pose_real[0]:.3f}, y={pose_real[1]:.3f}, theta={pose_real[2]:.3f}")
                    print(f"   Pos. Estimada: x={pose_est[0]:.3f}, y={pose_est[1]:.3f}, theta={pose_est[2]:.3f}")
                    print(f"   Error final: {error_dist:.3f} m, {error_ang:.3f} rad\n")
                    localizado = True
                    break
                
                # Reiniciar para la siguiente lectura
                pose_real = None
                pose_est = None
                
        if not localizado:
            print(f"❌ El robot NO logró localizarse dentro del umbral en {iteracion} iteraciones registradas.\n")

if __name__ == "__main__":
    analizar_resultados()
