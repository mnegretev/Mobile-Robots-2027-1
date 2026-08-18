#
# MOBILE ROBOTS - FI-UNAM, 2026-2
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

NAME = "FULL NAME"

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
        #
        # TODO:
        # Calculate the forward kinematics given the set of six angles 'q'
        # You can use the following steps:
        #     H = I   # Assing to H a 4x4 identity matrix
        #     for all q in Q:
        #         R = Homogeneous transformation with zero translation and rotated q[rad] over z-axis
        #         R should be a numpy matrix
        #         H = H * Hs[i] * R
        #     H = H * Hs[6]
        #     Get RPY from the resulting H
        #     Get xyz from the resulting H
        #
        H =  numpy.identity(4)
        #print(Q)
        for i in range(6):
            R = numpy.asarray([[numpy.cos(Q[i]), -numpy.sin(Q[i]), 0, 0],
                               [numpy.sin(Q[i]),  numpy.cos(Q[i]), 0, 0],
                               [0,0,1,0],
                               [0,0,0,1]])
            #print(R)
            H = H @ Hs[i] @ R
        H = H @ Hs[6]
        R,P,Y = self.matrix_to_euler_xyz(H)
        x,y,z = H[0,3], H[1,3], H[2,3]
        return numpy.asarray([x, y, z, R, P, Y])

    def jacobian(self, Q):
        delta_q = 0.000001
        J = numpy.asarray([[0.0 for q in Q] for i in range(6)])
        #
        # TODO:
        # Calculate the Jacobian evaluated in the point Q
        # Use the numeric approximation:   f'(x) = (f(x+delta) - f(x-delta))/(2*delta)
        #
        # You can do the following steps:
        #     J = matrix of 6x6 full of zeros
        #     q_next = [q1+delta       q2        q3   ....     q7
        #                  q1       q2+delta     q3   ....     q7
        #                              ....
        #                  q1          q2        q3   ....   q7+delta]
        #     q_prev = [q1-delta       q2        q3   ....     q7
        #                  q1       q2-delta     q3   ....     q7
        #                              ....
        #                  q1          q2        q3   ....   q7-delta]
        #     FOR i = 0,..,5:
        #           i-th column of J = ( FK(i-th row of q_next) - FK(i-th row of q_prev) ) / (2*delta_q)
        #     RETURN J
        #
        qn = numpy.asarray([Q,]*len(Q)) + numpy.identity(len(Q))*delta_q
        qp = numpy.asarray([Q,]*len(Q)) - numpy.identity(len(Q))*delta_q
        for i in range(6):
            J[:,i] = (self.forward_kinematics(qn[i]) - self.forward_kinematics(qp[i]))/(2.0*delta_q)
        return J
        
    def inverse_kinematics(self, Xd, init_guess=numpy.zeros(7), max_iter=2000):
        Xd= numpy.asarray(Xd)
        Q = init_guess
        iterations = 0

        #
        # TODO:
        # Solve the IK problem given a desired configuration.
        # Use the Newton-Raphson method for root finding. (Find the roots of equation FK(q) - Xd = 0)
        # You can do the following steps:
        #
        #    Set an initial guess for joints 'Q'
        #    Calculate Forward Kinematics 'X' by calling the corresponding function
        #    Calcualte error = X - Xd
        #    Ensure orientation angles of error are in (-pi,pi]
        #    WHILE |error| > TOL and iterations < maximum iterations:
        #        Calculate Jacobian
        #        Update estimated Q with Q = Q - pseudo_inverse(J)*error
        #        Ensure all angles q are in [-pi,pi]
        #        Recalculate forward kinematics X
        #        Recalculate error and ensure angles are in (-pi,pi]
        #        Increment iterations
        #    Set success if maximum iterations were not exceeded
        #    Return success and calculated Q
        #
        tol = 0.000001
        X = self.forward_kinematics(Q)
        e = X - Xd
        e[3:6] = (e[3:6] + math.pi)%(2*math.pi) - math.pi
        while numpy.linalg.norm(e)  > tol and iterations < max_iter:
            J = self.jacobian(Q)
            Q = (Q - numpy.linalg.pinv(J) @ e + math.pi)%(2*math.pi) - math.pi
            X = self.forward_kinematics(Q)
            e = X - Xd
            e[3:6] = (e[3:6] + math.pi)%(2*math.pi) - math.pi
            iterations += 1
        success = iterations < max_iter
        if success:
            self.get_logger().info("IK solved after " + str(iterations) + " steps. Q=" + str(Q))
        else:
            self.get_logger().info("Cannot solve IK")
        return success, Q

    def callback_ik_pose2pose(self, req, resp):
        N = self.get_parameter('N').get_parameter_value().integer_value
        Xd = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
        self.get_logger().info("Calculating IK for " +  str(Xd) + "with max " + str(N) + " iterations and Qo=" + str(req.initial_guess))
        success, Q = self.inverse_kinematics(Xd, req.initial_guess, N)
        resp.q = Q if success else []
        return resp
    
    def __init__(self):
        super().__init__("inverse_kinematics")
        self.get_logger().info("INITIALIZING INVERSE KINEMATICS BY NEWTON-RAPHSON NODE - " + NAME)
        self.declare_parameter('N', 100)
        self.srv_smooth_path = self.create_service(InverseKinematicsPose2Pose, '/manipulation/ik_pose2pose', self.callback_ik_pose2pose)

def main(args=None):
    rclpy.init(args=args)
    ik_node = IKNewtonRaphsonNode()
    print(ik_node.forward_kinematics([0, -0.5, -1.4, 0,0.3,0]))
    rclpy.spin(ik_node)
    ik_node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
