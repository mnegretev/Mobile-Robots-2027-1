#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# INVERSE KINEMATICS BY NEWTON-RAPHSON
#
# Instructions:
# Write the code necessary to solve the inverse kinematics problem
# using the Newton-Raphson method.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#
import rclpy
from rclpy.node import Node
from manip_msgs.srv import *
import numpy
import math

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"

H0 = [[1.0, 0.0, 0.0, 0.000], # link1 to link_base, joint rotates on Z
      [0.0, 1.0, 0.0, 0.000],
      [0.0, 0.0, 1.0, 0.267],
      [0.0, 0.0, 0.0, 1.000]]

H1 = [[1.0,  0.0, 0.0, 0.0], #link2 to link1, joint rotates on Z
      [0.0,  0.0, 1.0, 0.0],
      [0.0, -1.0, 0.0, 0.0],
      [0.0,  0.0, 0.0, 1.0]]

H2 = [[1.0, 0.0, 0.0,  0.0535], #link3 to link2, joint rotates on Z
      [0.0, 1.0, 0.0, -0.2845],
      [0.0, 0.0, 1.0,  0.000],
      [0.0, 0.0, 0.0,  1.000]]

H3 = [[1.0,  0.0, 0.0, 0.0775], #link4 to link3, joint rotates on Z
      [0.0,  0.0, 1.0, 0.3425],
      [0.0, -1.0, 0.0, 0.000],
      [0.0,  0.0, 0.0, 1.000]]

H4 = [[1.0, 0.0,  0.0, 0.000], #link5 to link4, joint rotates on Z
      [0.0, 0.0, -1.0, 0.000],
      [0.0, 1.0,  0.0, 0.000],
      [0.0, 0.0,  0.0, 1.000]]

H5 = [[1.0,  0.0, 0.0, 0.076], #link6 to link5, joint rotates on Z
      [0.0,  0.0, 1.0, 0.097],
      [0.0, -1.0, 0.0, 0.000],
      [0.0,  0.0, 0.0, 1.000]]

H6 = [[1.0, 0.0, 0.0, 0.000], #link6 to link_tcp (final effector), fixed joint
      [0.0, 1.0, 0.0, 0.000],
      [0.0, 0.0, 1.0, 0.172],
      [0.0, 0.0, 0.0, 1.000]]

Hs = [numpy.asarray(H0), numpy.asarray(H1), numpy.asarray(H2),
     numpy.asarray(H3), numpy.asarray(H4), numpy.asarray(H5),
     numpy.asarray(H6)]

class IKNewtonRaphsonNode(Node):
    def matrix_to_euler_xyz(self, R):
        # Calculate pitch (sy)
        sy = numpy.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]) 
        singular = sy < 1e-6 # Check for gimbal lock
        if not singular:
            x = numpy.arctan2(R[2, 1], R[2, 2])
            y = numpy.arctan2(-R[2, 0], sy)
            z = numpy.arctan2(R[1, 0], R[0, 0])
        else:
            # Gimbal lock case
            x = numpy.arctan2(-R[1, 2], R[1, 1])
            y = numpy.arctan2(-R[2, 0], sy)
            z = 0
            
        return x,y,z

    def forward_kinematics(self, Q):
        q_vec = numpy.zeros(6, dtype=float)
        q_arr = numpy.asarray(Q, dtype=float)
        n_copy = min(len(q_arr), 6)
        q_vec[:n_copy] = q_arr[:n_copy]

        H = numpy.identity(4, dtype=float)
        for i in range(6):
            c = math.cos(q_vec[i])
            s = math.sin(q_vec[i])
            Rz = numpy.asarray([
                [c, -s, 0.0, 0.0],
                [s,  c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=float)
            H = H @ Hs[i] @ Rz
        H = H @ Hs[6]
        R, P, Y = self.matrix_to_euler_xyz(H[0:3, 0:3])
        return numpy.asarray([H[0, 3], H[1, 3], H[2, 3], R, P, Y], dtype=float)

    def jacobian(self, Q):
        delta_q = 1e-6
        q_base = numpy.zeros(6, dtype=float)
        q_arr = numpy.asarray(Q, dtype=float)
        n_copy = min(len(q_arr), 6)
        q_base[:n_copy] = q_arr[:n_copy]

        J = numpy.zeros((6, 6), dtype=float)
        for i in range(6):
            q_next = q_base.copy()
            q_prev = q_base.copy()
            q_next[i] += delta_q
            q_prev[i] -= delta_q
            diff = self.forward_kinematics(q_next) - self.forward_kinematics(q_prev)
            diff[3:] = (diff[3:] + math.pi) % (2.0 * math.pi) - math.pi
            J[:, i] = diff / (2.0 * delta_q)
        return J

    def _solve_ik_from_seed(self, Xd, seed_q, max_iter, tol=1e-3):
        Q = numpy.asarray(seed_q, dtype=float).copy()
        X = self.forward_kinematics(Q)
        error = X - Xd
        error[3:] = (error[3:] + math.pi) % (2.0 * math.pi) - math.pi
        iterations = 0
        while numpy.linalg.norm(error) > tol and iterations < max_iter:
            J = self.jacobian(Q)
            dq = numpy.linalg.pinv(J, rcond=1e-3) @ error
            step_norm = numpy.linalg.norm(dq)
            if step_norm > 0.3:
                dq = dq * (0.3 / step_norm)
            Q = (Q - dq + math.pi) % (2.0 * math.pi) - math.pi
            X = self.forward_kinematics(Q)
            error = X - Xd
            error[3:] = (error[3:] + math.pi) % (2.0 * math.pi) - math.pi
            iterations += 1
        success = bool(numpy.linalg.norm(error) <= tol and numpy.all(numpy.isfinite(Q)))
        return success, Q, iterations

    def inverse_kinematics(self, Xd, init_guess=numpy.zeros(6), max_iter=2000):
        Xd = numpy.asarray(Xd, dtype=float)
        seeds = []
        if init_guess is not None and len(init_guess) == 6:
            guess = numpy.asarray(init_guess, dtype=float)
            if numpy.linalg.norm(guess) < 1e-5:
                # Perturb all-zero configuration slightly to avoid kinematic singularity
                seeds.append(numpy.asarray([0.0, -0.15, -0.15, 0.0, 0.15, 0.0], dtype=float))
            else:
                seeds.append(guess)
        else:
            seeds.append(numpy.asarray([0.0, -0.5, -0.5, 0.0, 0.5, 0.0], dtype=float))
        seeds.append(numpy.asarray([0.0, -0.8, -0.7, 0.0, 1.2, 0.0], dtype=float))

        Q = seeds[0]
        iterations = 0
        success = False
        for seed in seeds:
            success, Q, iterations = self._solve_ik_from_seed(Xd, seed, max_iter)
            if success:
                break

        if success:
            self.get_logger().info("IK solved after " + str(iterations) + " steps. Q=" + str(Q))
        else:
            self.get_logger().info("Cannot solve IK")
        return success, Q

    def callback_ik_pose2pose(self, req, resp):
        N = self.get_parameter('N').get_parameter_value().integer_value
        Xd = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
        self.get_logger().info("Calculating IK for " + str(Xd) + " with max " + str(N) + " iterations and Qo=" + str(req.initial_guess))
        success, Q = self.inverse_kinematics(Xd, req.initial_guess, N)
        resp.q = [float(v) for v in Q] if success else []
        return resp

    def callback_forward_kinematics(self, req, resp):
        X = self.forward_kinematics(req.q)
        resp.x, resp.y, resp.z = float(X[0]), float(X[1]), float(X[2])
        resp.roll, resp.pitch, resp.yaw = float(X[3]), float(X[4]), float(X[5])
        return resp
    
    def __init__(self):
        super().__init__("inverse_kinematics")
        self.get_logger().info("INITIALIZING INVERSE KINEMATICS BY NEWTON-RAPHSON NODE - " + NAME)
        self.declare_parameter('N', 100)
        self.srv_ik = self.create_service(InverseKinematicsPose2Pose, '/manipulation/ik_pose2pose', self.callback_ik_pose2pose)
        self.srv_fk = self.create_service(ForwardKinematics, '/manipulation/forward_kinematics', self.callback_forward_kinematics)

def main(args=None):
    rclpy.init(args=args)
    ik_node = IKNewtonRaphsonNode()
    rclpy.spin(ik_node)
    ik_node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
