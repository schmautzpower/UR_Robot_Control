import numpy as np
from numpy import cos, sin, tan, pi, sqrt
from math import atan2


class Roboterarm:
    def __init__(self):
        self.dh_params = [
            [0, 0, 0.1807, pi/2],
            [0, -0.6127, 0, 0],
            [0, -0.57155, 0, 0],
            [0, 0, 0.17415, pi/2],
            [0, 0, 0.11985, -pi/2],
            [0, 0, 0.11655, 0]
        ]

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            alpha, a, d, theta = self.dh_params[i]
            cos_theta = cos(joint_angles[i] + theta)
            sin_theta = sin(joint_angles[i] + theta)
            cos_alpha = cos(alpha)
            sin_alpha = sin(alpha)
            T_i = np.array([
                [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
                [sin_theta, cos_theta*cos_alpha, -
                    cos_theta*sin_alpha, a*sin_theta],
                [0, sin_alpha, cos_alpha, d],
                [0, 0, 0, 1]
            ])
            T = np.dot(T, T_i)
        return T[:3, 3]

    # def inverse_kinematics(self, target_position):
    #     x, y, z = target_position
    #     d6 = self.dh_params[5][2]
    #     theta1 = atan2(y, x)
    #     r = sqrt(x*x + y*y)
    #     s = z - d6
    #     D = (r*r + s*s - self.dh_params[0][1]*self.dh_params[0][1] - self.dh_params[3]
    #          [1]*self.dh_params[3][1]) / (2*self.dh_params[0][1]*self.dh_params[3][1])
    #     theta3 = atan2(-sqrt(1 - D*D), D)
    #     K = self.dh_params[0][1] + self.dh_params[3][1]*cos(theta3)
    #     L = self.dh_params[3][1]*sin(theta3)
    #     theta2 = atan2(s, r) - atan2(L, K)
    #     theta5 = atan2(sqrt(1 - (cos(theta2)*cos(theta3))**2),
    #                    cos(theta2)*cos(theta3))
    #     theta4 = atan2((self.dh_params[3][1]*sin(theta3))/(self.dh_params[0][1] + self.dh_params[3][1]*cos(
    #         theta3)), (self.dh_params[2][1]*sin(theta2))/(self.dh_params[1][1] + self.dh_params[4][1]*cos(theta5)))
    #     theta6 = atan2(sin(theta2)*sin(theta3), cos(theta2))
    #     return [theta1, theta2, theta3, theta4, theta5, theta6]

    def inverse_kinematics(self, target_position):
        x, y, z = target_position
        d6 = self.dh_params[5][2]
        theta1 = atan2(y, x)
        r = sqrt(x*x + y*y)
        s = z - d6

        # Calculate the denominator
        denominator = 2 * self.dh_params[0][2] * self.dh_params[3][2]

        # Check if the denominator is not zero
        if denominator != 0:
            D = (r*r + s*s - self.dh_params[0][1]*self.dh_params[0][1] -
                 self.dh_params[3][1]*self.dh_params[3][1]) / denominator

            # Check if D is within the valid range [-1, 1] for acos function
            if -1 <= D <= 1:
                theta3 = atan2(-sqrt(1 - D*D), D)
                K = self.dh_params[0][1] + self.dh_params[3][1]*cos(theta3)
                L = self.dh_params[3][1]*sin(theta3)
                theta2 = atan2(s, r) - atan2(L, K)
                theta5 = atan2(sqrt(1 - (cos(theta2)*cos(theta3))
                               ** 2), cos(theta2)*cos(theta3))
                theta4 = atan2((self.dh_params[3][1]*sin(theta3))/(self.dh_params[0][1] + self.dh_params[3][1]*cos(
                    theta3)), (self.dh_params[2][1]*sin(theta2))/(self.dh_params[1][1] + self.dh_params[4][1]*cos(theta5)))
                theta6 = atan2(sin(theta2)*sin(theta3), cos(theta2))
                return [theta1, theta2, theta3, theta4, theta5, theta6]
            else:
                # Handle the case where D is outside the valid range [-1, 1]
                # This might involve setting theta3 to a default value or handling it differently based on your application logic.
                pass
        else:
            # Handle the case where the denominator is zero
            # This might involve setting theta3 to a default value or handling it differently based on your application logic.
            pass


if __name__ == "__main__":
    # Denavit-Hartenberg Parameter
    robot = Roboterarm()

    # Test Forward Kinematics
    joint_angles = [1.5708, -1.7453, 1.2217, 5.236, -1.5708, 1.5708]
    end_effector_position = robot.forward_kinematics(joint_angles)
    print("End Effector Position (Forward Kinematics):", end_effector_position)

    # Test Inverse Kinematics
    target_position = [0.38860315, -0.889169, 0.59125]
    joint_angles = robot.inverse_kinematics(target_position)
    print("Joint Angles (Inverse Kinematics):", joint_angles)
