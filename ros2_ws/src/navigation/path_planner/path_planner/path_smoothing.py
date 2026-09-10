#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from navig_msgs.srv import ProcessPath
import numpy

NAME = "Leonardo Santos Vicente"

class PathSmoothingNode(Node):
    def smooth_path(self, Q, w1, w2, max_steps):
        P = numpy.copy(Q)
        tol     = 0.00001                   
        nabla   = numpy.full(Q.shape, float("inf"))
        epsilon = 0.1                       
        #
        # TODO:
        # Write the code to smooth the path Q, using the gradient descend algorithm,
        # and return a new smoothed path P.
        # Path is composed of a set of points [x,y] as follows:
        # [[x0,y0], [x1,y1], ..., [xn,ym]].
        # The smoothed path must have the same shape.
        # Return the smoothed path.

        # J es un paraboloide, por lo que tiene un mínimo global.
        # El objetivo del algoritmo es encontrar ese mínimo mediante descenso
        # del gradiente. Para llegar al mínimo se avanza en dirección contraria
        # al gradiente, hasta que su magnitud sea suficientemente pequeña.
        #
        # La primera y la última coordenada no se modifican porque representan
        # los extremos de la ruta. Por lo tanto, sus gradientes se mantienen
        # en cero.
        nabla[0] = 0
        nabla[-1] = 0

        steps = 0

        # Se repite el proceso mientras la magnitud del gradiente sea mayor
        # que la tolerancia y no se haya alcanzado el número máximo de pasos.
        #
        # Los puntos interiores de nabla fueron inicializados con infinito,
        # por lo que el algoritmo entra al while en la primera iteración.
        while numpy.linalg.norm(nabla) > tol and steps < max_steps:

            # Se recorren únicamente los puntos interiores de la ruta:
            # desde el segundo punto hasta el penúltimo.
            #
            # El primer y último punto NO se incluyen en este algoritmo.
            for i in range(1, len(Q) - 1):

                # Q representa la ruta original.
                # P representa la ruta que se va modificando hasta obtener
                # la ruta suavizada.

                # El gradiente se obtiene al derivar la función de costo J.
                # Al derivar la sumatoria, por regla de la cadena, aparecen
                # el punto anterior, el punto actual y el punto siguiente.
                #
                # w1 determina cuánto se desea suavizar la trayectoria.
                # w2 determina cuánto se desea conservar la trayectoria original.
                #
                # El primer término busca suavizar la trayectoria haciendo
                # que el punto actual tenga una posición más cercana a sus
                # puntos vecinos.
                #
                # El segundo término evita que la trayectoria se aleje
                # demasiado de la ruta original.
                nabla[i] = (
                    w1 * (2 * P[i] - P[i - 1] - P[i + 1])
                    + w2 * (P[i] - Q[i])
                )

            # Descenso del gradiente:
            # P se mueve en la dirección contraria al gradiente.
            #
            # epsilon representa la distancia pequeña que se avanza
            # en cada iteración.
            #
            # Como nabla[0] y nabla[-1] son cero, los puntos inicial
            # y final permanecen sin cambios.
            P = P - epsilon * nabla

            # Se incrementa el contador de pasos.
            steps += 1
        #
        # END OF TODO
        #
        return P

    def callback_smooth_path(self, request, response):
        w1  = self.get_parameter('w1').get_parameter_value().double_value
        w2  = self.get_parameter('w2').get_parameter_value().double_value
        steps  = self.get_parameter('steps').get_parameter_value().integer_value
        self.get_logger().info("Smoothing path with params: " + str([w1, w2, steps]))
        start_time = self.get_clock().now()
        Q = numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in request.path.poses])
        P = self.smooth_path(Q, w1, w2, steps)
        end_time = self.get_clock().now()
        delta_ms = (end_time.nanoseconds - start_time.nanoseconds)/1e6
        self.get_logger().info("Path smoothed after " + str(delta_ms) + " ms")
        self.msg_smooth_path.header.frame_id = request.path.header.frame_id
        self.msg_smooth_path.header.stamp = self.get_clock().now().to_msg()
        self.msg_smooth_path.poses = []
        for i in range(len(request.path.poses)):
            p = PoseStamped()
            p.pose.position.x = P[i,0]
            p.pose.position.y = P[i,1]
            self.msg_smooth_path.poses.append(p)
        self.pub_smooth_path.publish(self.msg_smooth_path)
        response.processed_path = self.msg_smooth_path
        return response
            
    def __init__(self):
        super().__init__("path_smoothing_node")
        self.get_logger().info("INITIALIZING PATH SMOOTHING NODE - " + NAME)
        self.declare_parameter('w1', 0.9)
        self.declare_parameter('w2', 0.1)
        self.declare_parameter('steps', 10000)
        self.srv_smooth_path = self.create_service(ProcessPath, '/path_planning/smooth_path', self.callback_smooth_path)
        self.pub_smooth_path = self.create_publisher(Path, '/path_planning/smoothed_path', 10)
        self.msg_smooth_path = Path()
            
def main(args=None):
    rclpy.init(args=args)
    path_smoothing_node = PathSmoothingNode()
    rclpy.spin(path_smoothing_node)
    path_smoothing_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
