import torch
import yaml
import numpy as np
# from dwa.dwa_core import RobotType
from .envs.navigation_2d import Navigation2DEnv
import matplotlib.pyplot as plt
import math
class Config:
    """
    simulation parameter class
    """

    def __init__(self, yaml_file):
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check\

        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        # self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.rectangle

        # obstacles [x(m) y(m), ....]
        with open(yaml_file) as file:
            param = yaml.safe_load(file)

        self.ob = np.array(param['obstacles'])
        self.ob_len = param['obstacle_length']
        self.start_loc = param['start_loc']
        self.goal_loc = param['goal_loc']
        self.num_samples = param['num_samples']
        self.horizon = param['horizon']
        self.map_size = param['map_size']
        self.map_origin = param['map_origin']

        self.robot_width = 1.2 + self.ob_len   # [m] for collision check
        self.robot_length = 1.2 + self.ob_len  # [m] for collision check


def computeTrajLen(trajectory):
    N = len(trajectory)
    total_len  = 0.0
    for i in range(1, N):
        A = trajectory[i]
        B = trajectory[i - 1]
        delta = np.sqrt( (A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2 )
        total_len += delta
    return total_len


class Interface:
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    def __init__(self, cfg):
        self.env = Navigation2DEnv(cfg.test)
        self.state = self.env.reset()
        self.traj = []

    def render(self, state_seq):
        self.env.render(
            predicted_trajectory=state_seq,
            is_collisions=None,
            top_samples=None,
            mode="human",
        )
        ntraj = np.array(self.traj)
        plt.plot(ntraj[:, 0], ntraj[:, 1], "-r")

    def trajLength(self):
        return computeTrajLen(self.traj)
