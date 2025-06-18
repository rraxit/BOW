import numpy as np
import matplotlib.pyplot as plt
from cbf_toolbox.geometry import Sphere, Ellipsoid, HalfPlane
from cbf_toolbox.dynamics import Dynamics, SingleIntegrator2d
from cbf_toolbox.vertex import Agent, Obstacle, Goal
from cbf_toolbox.safety import Simulation
import hydra
from omegaconf import DictConfig, OmegaConf



class CBF:
    def __init__(self, cfg: DictConfig):

        start = np.array(cfg.test.start_loc)
        goal = np.array(cfg.test.goal_loc)
        self.a1 = Agent(state=start, shape=Sphere(cfg.planner.robot_radius), dynamics=SingleIntegrator2d())
        g1 = Goal(goal)
        self.s = Simulation()
        self.s.add_agent(agent=self.a1, control=g1)
        self.cfg = cfg
        self.traj = []

        for obs in cfg.test.obstacles:
            o1 = Obstacle(state=np.array(obs), shape=Sphere(cfg.planner.obstacle_length), dynamics=SingleIntegrator2d())
            self.s.add_obstacle(o1)
    def __call__(self, *args, **kwargs):
        self.s.simulate(num_steps=self.cfg.test.num_steps, dt=0.1,  plotting=self.cfg.visualize)

        self.traj = self.a1.trajectory

    def trajLength(self):
        N = len(self.traj)
        total_len = 0.0
        for i in range(1, N):
            A = self.traj[i]
            B = self.traj[i - 1]
            delta = np.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)
            total_len += delta
        return total_len



