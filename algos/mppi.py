import torch
import numpy as np
from .common import Interface
from .mppi_core import MPPI as MPPICONTROL
class MPPI(Interface):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.mppi = MPPICONTROL(
        horizon=cfg.planner.horizon,
        num_samples=cfg.planner.num_samples, # 10000
        dim_state=3,
        dim_control=2,
        dynamics=self.env.dynamics,
        stage_cost=self.env.stage_cost,
        terminal_cost=self.env.terminal_cost,
        u_min=self.env.u_min,
        u_max=self.env.u_max,
        sigmas=torch.tensor([0.5, 0.5]),
        lambda_=1.0,
    )

    def __call__(self, *args, **kwargs):

        with torch.no_grad():
            action_seq, state_seq = self.mppi.forward(state=self.state)
        self.state, is_goal_reached = self.env.step(action_seq[0, :])
        return self.state, state_seq, is_goal_reached