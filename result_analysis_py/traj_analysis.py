import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Rectangle, Circle

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",  # or "sans-serif" depending on your document's font
    "font.serif": ["Palatino"],  # or any other font available in your LaTeX distribution
    "pgf.rcfonts": False,  # to prevent pgf from overriding rc settings
})

def main():
    env2_config = yaml.safe_load(open('../config/test/env2.yaml'))
    #plot_obstacles_using Rectangle with gray color
    #obstacles are given in terms of center points
    #matplotlib rectangles are defined with Origin, Width and Height
    width = height = 1.0
    offset = np.array([width, height]) / 2
    fig, ax = plt.subplots(figsize=(16, 16))

    for obs in env2_config['obstacles']:

        obs = np.array(obs)
        origin = obs - offset
        obs_patch = Rectangle((origin[0], origin[1]), width, height, linewidth=1, edgecolor='k', facecolor='grey')
        ax.add_patch(obs_patch)

    # add start and goal locations
    start = env2_config['start_loc']
    goal = env2_config['goal_loc']

    ax.add_patch(Circle(start, 0.50, color='g', alpha=0.4))
    ax.add_patch(Circle(goal, 0.50, color='r', alpha=0.4))

    METHODS = ("dwa", "mppi", "hrvo", "cbf", "bow")
    NAMES = {"dwa": "DWA", "mppi": "MPPI", "hrvo": "HRVO", "cbf": "CBF", "bow": "BOW"}
    seed = 2
    env = "env2"
    for method in METHODS:
        filepath = f"../results/benchmark/{seed}/{env}/{method}/traj.npz.npy"
        data = np.load(filepath)
        ax.plot(data[:, 0], data[:, 1], label=NAMES[method], linewidth=2)

    plt.axis([-10,10,-10,10])
    plt.legend(fontsize=30)
    plt.grid()
    plt.tight_layout()
    # plt.show()
    plt.savefig('env2_paths.pgf')

if __name__ == '__main__':
    main()