import os
import numpy as np
from configparser import ConfigParser
from collections import defaultdict
import matplotlib.pyplot as plt
import csv
from math import log

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",  # or "sans-serif" depending on your document's font
    "font.serif": ["Palatino"],  # or any other font available in your LaTeX distribution
    "pgf.rcfonts": False,  # to prevent pgf from overriding rc settings
})

def plot_results(results, name):
    # Step 2: Plotting
    fig, ax = plt.subplots(figsize=(8, 6))
    categories = list(results.keys())
    values = [np.mean(value) for value in results.values()]
    errors = [np.std(value) for value in results.values()]
    # Creating the bar chart with error bars
    ax.bar(categories, values, yerr=errors, capsize=5, color='skyblue', edgecolor='black', alpha=0.7)

    # Step 3: Customization
    # ax.set_title(name)
    ax.set_xlabel('Planners')
    ax.set_ylabel(name)
    ax.set_ylim(0, max(values) + max(errors) + 5)

    # Adding grid for better readability
    ax.yaxis.grid(True)

    # Show plot
    plt.show()

def save_results(results, name):
    # Specify the CSV file name
    csv_file_name = f'{name}.csv'

    # Write the default dict to a CSV file
    with open(csv_file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write header
        writer.writerow(['Planner', 'Mean', "Error"])
        # Write data
        for category, values in results.items():
            writer.writerow([category, f"{np.mean(values):.4f}", f"{np.std(values):.4f}"])



def box_plots(results, name, unit):
    
    values = list(results.values())
    if unit == "seconds":
        for i, val in enumerate(values):
            for j, item in enumerate(val):
                values[i][j] = log(item)
        
        unit = "log seconds"

    labels = [key.upper() for key in results.keys()]
    colors = ['peachpuff', 'orange', 'tomato', "deepskyblue", "lime", "magenta"]

    fig, ax = plt.subplots()
    ax.set_ylabel(unit)

    bplot = ax.boxplot(values,
                    patch_artist=True,  # fill with color
                    labels=labels)  # will be used to label x-ticks
    # bplot = ax.boxplot(values,
    #                 patch_artist=True)  # will be used to label x-ticks
    # fill with colors
    for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
    
    plt.tight_layout()
    plt.savefig(f"figs/{name}.pgf")


def main(name, unit):

    ENVS = ("env1_f", "env2_f", "env3_f", "env4_f", "env5_f", "env6_f",)
    METHODS = ("mppi","cbf")
    results = defaultdict(list)
    for seed in range(1, 11):
        for env in ENVS:
            for method in METHODS:
                filepath = f"../results/benchmark/{seed}/{env}/{method}/verbose.txt"
                if os.path.exists(filepath):
                    config = ConfigParser()
                    config.read(filepath)
                    duration = float(config["DEFAULT"][name])
                    results[method].append(duration)
                else:
                    print(filepath)
    box_plots(results, name, unit)


if __name__ == '__main__':
    main("traj_length", "meter")
    main("num-steps", "iterations")
    main("traj_duration", "seconds")
