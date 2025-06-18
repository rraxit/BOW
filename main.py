import numpy as np
import hydra
from omegaconf import DictConfig, OmegaConf
from algos import CBF,MPPI
import time
import os


def get_planner(cfg: DictConfig):
    if cfg.planner.name == "cbf":
        return CBF(cfg)
    if cfg.planner.name == "mppi":
        return MPPI(cfg)

    raise NotImplementedError

def calculate_trajectory_metrics(trajectory, dt=None):
    """
    Calculate average velocity and jerk from trajectory data
    
    Args:
        trajectory: numpy array of shape (n_steps, n_dims) containing positions
        dt: time step (if None, assumes unit time steps)
    
    Returns:
        avg_velocity: average velocity magnitude
        avg_jerk: average jerk magnitude
    """
    if len(trajectory) < 3:

        return 0.0, 0.0
    
    trajectory = np.array(trajectory)[:, :2]  # Ensure trajectory is 2D (x, y)
    print(f"trajectory shape: {trajectory.shape}")
    # If dt is not provided, estimate from elapsed time and steps
    if dt is None:
        dt = 1.0  # Default unit time step
    
    # Calculate velocity (first derivative of position)
    velocity = np.diff(trajectory, axis=0) / dt
    
    # Calculate acceleration (second derivative of position)
    acceleration = np.diff(velocity, axis=0) / dt
    
    # Calculate jerk (third derivative of position)
    jerk = np.diff(acceleration, axis=0) / dt
    
    # # Calculate magnitudes
    # velocity_magnitudes = np.linalg.norm(velocity, axis=1)
    # jerk_magnitudes = np.linalg.norm(jerk, axis=1)
    
    # Calculate averages
    avg_velocity = np.mean(velocity)
    avg_jerk = np.mean(jerk) 
    
    return avg_velocity, avg_jerk

@hydra.main(version_base=None, config_path="config", config_name="main")
def my_app(cfg: DictConfig) -> None:
    # print(OmegaConf.to_yaml(cfg))
    planner = get_planner(cfg)
    start = time.time()
    print(f"running {cfg.planner.name}")
    step = 0
    
    if cfg.planner.name == "cbf":
        planner()
        step = cfg.test.num_steps
    else:
        while True:
            try:
                state, state_seq, is_goal_reached = planner()
                x = state.cpu().numpy()
                planner.traj.append(x)
                step += 1
                print(f"step: {step}", end="\r", flush=True)
                if cfg.visualize:
                    planner.render(state_seq)
                if is_goal_reached:
                    break
            except KeyboardInterrupt:
                break
    
    elapsed_time = time.time() - start
    
    # Calculate trajectory metrics
    dt = elapsed_time / step if step > 0 else 1.0  # Estimate time step
    avg_velocity, avg_jerk = calculate_trajectory_metrics(planner.traj, dt)
    
    # Print results
    print(f"traj_length = {planner.trajLength():.3f}")
    print(f"traj_duration = {elapsed_time:.3f} sec")
    print(f"avg_velocity = {avg_velocity:.3f}")
    print(f"avg_jerk = {avg_jerk:.3f}")
    
    if cfg.save_results:
        outputs = os.path.join(cfg.output_folder, f"{cfg.seed}", cfg.test.name, cfg.planner.name)
        # print(f"saving results to {outputs}")
        os.makedirs(outputs, exist_ok=True)
        traj_file = os.path.join(outputs, "traj.npz")
        log_file = os.path.join(outputs, "log.text")
        np.save(traj_file, planner.traj)
        
        with open(log_file, "w") as f:
            f.write("[DEFAULT]\n")
            f.write(f"traj_length = {planner.trajLength():.3f} \n")
            f.write(f"traj_duration = {elapsed_time:.3f} \n")
            f.write(f"num-steps = {step} \n")
            f.write(f"avg_velocity = {avg_velocity:.3f} \n")
            f.write(f"avg_jerk = {avg_jerk:.3f} \n")
    
    if cfg.planner.name == "bow":
        os.remove("temp.yaml")

if __name__ == "__main__":
    my_app()
