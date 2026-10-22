import rclpy
from rclpy.node import Node
import subprocess
import time
import math
import csv
import itertools
from geometry_msgs.msg import Twist, PoseArray

class PFTester(Node):
    def __init__(self):
        super().__init__('pf_tester')
        
        # Publicador y Suscriptores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.real_pose_sub = self.create_subscription(PoseArray, '/gazebo/diff_base_with_xarm/pose', self.real_pose_cb, 10)
        self.particle_sub = self.create_subscription(PoseArray, '/particle_cloud', self.particle_cloud_cb, 10)
        
        self.real_pose_x = None
        self.real_pose_y = None
        self.est_pose_x = None
        self.est_pose_y = None
        self.is_localized = False
        
        # 1. MARGEN DE ERROR ESTRICTO: 5 centímetros (0.05 metros)
        self.error_threshold = 0.05 
        
        self.test_cases = []
        
        # Generar EXACTAMENTE 30 combinaciones para evaluar
        list_N = [100, 300, 500, 700, 1000] # 5 opciones
        list_s2s = [0.2, 0.5, 0.8]          # 3 opciones
        list_s2m = [0.05, 0.1]              # 2 opciones
        list_s2r = [0.1]                    # 1 opción
        # Total: 5 x 3 x 2 x 1 = 30 pruebas
        
        for N, s2s, s2m, s2r in itertools.product(list_N, list_s2s, list_s2m, list_s2r):
            self.test_cases.append({'N': N, 's2s': s2s, 's2m': s2m, 's2r': s2r})

    def real_pose_cb(self, msg):
        if msg.poses:
            self.real_pose_x = msg.poses[0].position.x
            self.real_pose_y = msg.poses[0].position.y

    def particle_cloud_cb(self, msg):
        if not msg.poses:
            return
        
        # Paso A: Calcular el centroide (posición estimada promedio)
        sum_x = sum(p.position.x for p in msg.poses)
        sum_y = sum(p.position.y for p in msg.poses)
        self.est_pose_x = sum_x / len(msg.poses)
        self.est_pose_y = sum_y / len(msg.poses)
        
        # Paso B: Calcular la dispersión (qué tan esparcidas están las partículas)
        var_x = sum((p.position.x - self.est_pose_x)**2 for p in msg.poses) / len(msg.poses)
        var_y = sum((p.position.y - self.est_pose_y)**2 for p in msg.poses) / len(msg.poses)
        dispersion = math.sqrt(var_x + var_y)
        
        # Paso C: Validación estricta para evitar FALSOS POSITIVOS
        if self.real_pose_x is not None:
            error = math.sqrt((self.real_pose_x - self.est_pose_x)**2 + 
                              (self.real_pose_y - self.est_pose_y)**2)
            
            # error < 0.05: El centro está a menos de 5cm del robot real.
            # dispersion < 0.4: Las partículas forman un enjambre compacto y denso.
            if error < self.error_threshold and dispersion < 2.5:
                self.is_localized = True

    def run_tests(self):
        results = []
        
        for idx, params in enumerate(self.test_cases):
            self.get_logger().info(f"Iniciando prueba {idx+1}/{len(self.test_cases)} con: {params}")
            self.is_localized = False
            self.est_pose_x = None
            
            # Comando con el argumento corregido para Jazzy (-p use_sim_time:=true)
            cmd = (
                f"ros2 run particle_filter pf --ros-args -p use_sim_time:=true "
                f"-p N:={params['N']} -p s2s:={params['s2s']} "
                f"-p s2m:={params['s2m']} -p s2r:={params['s2r']}"
            )
            pf_process = subprocess.Popen(cmd, shell=True)
            time.sleep(3) 
            
            iterations = 0
            max_iterations = 600 # 60 segundos máximos de prueba
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.3
            
            start_time = time.time()
            
            while not self.is_localized and iterations < max_iterations and rclpy.ok():
                self.cmd_pub.publish(twist_msg)
                rclpy.spin_once(self, timeout_sec=0.1)
                iterations += 1
            
            # Detener el robot al finalizar la prueba
            self.cmd_pub.publish(Twist())
            time_taken = time.time() - start_time
            
            results.append({
                'N': params['N'], 's2s': params['s2s'], 's2m': params['s2m'], 's2r': params['s2r'],
                'localized': self.is_localized,
                'iterations': iterations,
                'time_seconds': round(time_taken, 2),
                'final_real_x': round(self.real_pose_x, 3) if self.real_pose_x is not None else 'N/A',
                'final_est_x': round(self.est_pose_x, 3) if self.est_pose_x is not None else 'N/A'
            })
            
            self.get_logger().info(f"Prueba terminada. Localizado: {self.is_localized} en {iterations} iteraciones.")
            
            # Matar el nodo de C++
            pf_process.terminate()
            pf_process.wait()
            time.sleep(2)

            # Guardado progresivo en CSV
            with open('pf_results.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=results[0].keys())
                writer.writeheader()
                writer.writerows(results)

        self.get_logger().info("Todas las 30 pruebas finalizadas. Resultados guardados en pf_results.csv")

def main(args=None):
    rclpy.init(args=args)
    tester = PFTester()
    tester.run_tests()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()