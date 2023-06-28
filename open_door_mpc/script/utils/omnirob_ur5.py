from abc import ABC, abstractmethod

import numpy as np
import modern_robotics as mr


class OmniRobUr5(ABC):
    # base
    a, b, wheel_R = 0.24, 0.35, 0.108
    # arm
    L1, L2, W1, W2, H1, H2 = 0.425, 0.392, 0.109, 0.082, 0.089, 0.095
    # W2 += 0.158  #
    W2 += 0.108  #
    M6 = np.array([[-1, 0, 0, L1 + L2],
                   [0, 0, 1, W1 + W2],
                   [0, 1, 0, H1 - H2],
                   [0, 0, 0, 1]], dtype=float)
    Slist = np.array([[0, 0, 1, 0, 0, 0],
                      [0, 1, 0, -H1, 0, 0],
                      [0, 1, 0, -H1, 0, L1],
                      [0, 1, 0, -H1, 0, L1 + L2],
                      [0, 0, -1, -W1, L1 + L2, 0],
                      [0, 1, 0, H2 - H1, 0, L1 + L2]], dtype=float).T
    Blist = np.array([[0., 1., 0., 0.191, -0., 0.817],
                      [0., 0., 1., 0.095, -0.817, -0.],
                      [0., 0., 1., 0.095, -0.392, -0.],
                      [0., 0., 1., 0.095, -0., -0.],
                      [0, -1., 0., -0.082, -0., -0.],
                      [0., 0., 1., -0., -0., -0.]], dtype=float).T
    # V_b = F*d_theta -- MR:p351
    F = wheel_R / 4 * np.array([[-1 / (a + b), 1 / (a + b), 1 / (a + b), -1 / (a + b)],
                                [1, 1, 1, 1],
                                [-1, 1, -1, 1]])

    F6 = np.zeros(shape=(6, 4), dtype=float)
    F6[2:5, :] = F
    # phi x  y  J1 J2 J3 J4 J5 J6 w1 w2 w3 w4 gripper
    config = np.zeros(14, dtype=np.float64).reshape(1, 14)

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def world_frame_set(self, desired_world_positions):
        pass

    @abstractmethod
    def get_world_frame(self):
        pass

    @abstractmethod
    def get_amr_pos(self):
        pass

    @abstractmethod
    def get_tip_pos(self):
        pass

    # Get amrRob arm joint angles
    @abstractmethod
    def get_arm_agl(self):
        pass

    @abstractmethod
    def arm_ctrl(self, desired_arm_joint_angles):
        pass

    @abstractmethod
    def gripper_ctrl(self, state):
        pass

    @abstractmethod
    def wheel_pos_ctrl(self, desired_wheel_positions):
        pass

    def rob_move2(self, vy, vx, omega):
        a, b, wheel_R = self.a, self.b, self.wheel_R
        omega_1 = (vy - vx + (a + b) * omega) / wheel_R
        omega_2 = (vy + vx + (a + b) * omega) / wheel_R
        omega_3 = (vy - vx - (a + b) * omega) / wheel_R
        omega_4 = (vy + vx - (a + b) * omega) / wheel_R
        v_wheel_sim = [omega_1, omega_2, -omega_3, -omega_4]
        v_wheel_act = [-omega_1, -omega_2, -omega_3, -omega_4]
        return v_wheel_sim, v_wheel_act

    def rob_move(self, vx, vy, omega):  # follow MR book
        a, b, wheel_R = self.a, self.b, self.wheel_R
        omega_1 = (-vy + vx - (a + b) * omega) / wheel_R
        omega_2 = (vy + vx + (a + b) * omega) / wheel_R
        omega_3 = (-vy + vx + (a + b) * omega) / wheel_R
        omega_4 = (vy + vx - (a + b) * omega) / wheel_R
        v_wheel_sim = [omega_1, omega_2, -omega_3, -omega_4]
        v_wheel_act = [omega_1, -omega_2, -omega_3, omega_4]
        return v_wheel_sim, v_wheel_act

    def ur5_ik(self, target, now_theta):
        return mr.IKinSpace(self.Slist, self.M6, target, now_theta, eomg=0.01, ev=0.001)

    def ur5_fk(self, theta_list):
        T = mr.FKinSpace(self.M6, self.Slist, theta_list)
        return T

