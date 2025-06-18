import os
import numpy as np
from collections import defaultdict
import re
import json

def parse_verbose_file(filepath):
    """
    Parses a single verbose.txt file and extracts relevant metrics.
    """
    metrics = {}
    try:
        with open(filepath, 'r') as f:
            content = f.read()
            
            # Extract trajectory shape
            traj_shape_match = re.search(r"trajectory shape: \((\d+), (\d+)\)", content)
            if traj_shape_match:
                metrics['num_steps'] = int(traj_shape_match.group(1)) # First number in shape is num_steps

            # Extract other metrics
            traj_length_match = re.search(r"traj_length = ([\d.]+)", content)
            if traj_length_match:
                metrics['traj_length'] = float(traj_length_match.group(1))

            traj_duration_match = re.search(r"traj_duration = ([\d.]+) sec", content)
            if traj_duration_match:
                metrics['traj_duration'] = float(traj_duration_match.group(1))

            avg_velocity_match = re.search(r"avg_velocity = ([\d.]+)", content)
            if avg_velocity_match:
                metrics['avg_velocity'] = float(avg_velocity_match.group(1))

            avg_jerk_match = re.search(r"avg_jerk = (-?[\d.]+)", content)

            if avg_jerk_match:
                metrics['avg_jerk'] = float(avg_jerk_match.group(1))

    except FileNotFoundError:
        print(f"File not found: {filepath}")
    except Exception as e:
        print(f"Error reading file {filepath}: {e}")
    return metrics

def main():
    ENVS = ("env1_f", "env2_f", "env3_f", "env4_f", "env5_f", "env6_f")
    # Using only 'mppi' and 'cbf' as per your initial script, but you can add more if needed
    METHODS = ("mppi", "cbf") 
    
    all_results = defaultdict(lambda: defaultdict(list))

    # Iterate through seeds, environments, and methods to collect data
    for seed in range(1, 11):  # 10 seeds
        for env_full_name in ENVS:
            # Extract environment number from name (e.g., "env1_f" -> "1")
            env_number = re.search(r"env(\d+)_f", env_full_name).group(1)
            for method in METHODS:
                filepath = f"../results/benchmark/{seed}/{env_full_name}/{method}/verbose.txt"
                metrics = parse_verbose_file(filepath)
                
                if metrics:
                    all_results[method][env_number].append(metrics)

    # Process collected data and calculate averages and standard deviations
    final_output = []
    for planner, env_data in all_results.items():
        for environment, runs_data in env_data.items():
            if not runs_data:
                continue

            # Initialize lists for all metrics
            path_lengths = []
            durations = []
            velocities = []
            jerks = []
            num_steps_list = []

            for run in runs_data:
                if 'traj_length' in run:
                    path_lengths.append(run['traj_length'])
                if 'traj_duration' in run:
                    durations.append(run['traj_duration'])
                if 'avg_velocity' in run:
                    velocities.append(run['avg_velocity'])
                if 'avg_jerk' in run:
                    jerks.append(run['avg_jerk'])
                if 'num_steps' in run:
                    num_steps_list.append(run['num_steps'])

            # Calculate averages and standard deviations
            avg_path_length = np.mean(path_lengths) if path_lengths else None
            stddev_path_length = np.std(path_lengths) if path_lengths else None
            
            avg_time_ms = np.mean(durations) * 1000 if durations else None # Convert seconds to milliseconds
            stddev_time_ms = np.std(durations) * 1000 if durations else None

            avg_velocity = np.mean(velocities) if velocities else None
            stddev_velocity = np.std(velocities) if velocities else None

            avg_jerk = np.mean(jerks) if jerks else None
            stddev_jerk = np.std(jerks) if jerks else None
            
            avg_num_steps = np.mean(num_steps_list) if num_steps_list else None
            stddev_num_steps = np.std(num_steps_list) if num_steps_list else None


            final_output.append({
                "planner": planner.upper(),
                "environment": environment,
                "avg_path_length": avg_path_length,
                "stddev_path_length": stddev_path_length,
                "avg_time_ms": avg_time_ms,
                "stddev_time_ms": stddev_time_ms,
                "avg_velocity": avg_velocity,
                "stddev_velocity": stddev_velocity,
                "avg_jerk": avg_jerk,
                "stddev_jerk": stddev_jerk,
                "num_nodes": None, # This metric isn't available in your verbose.txt
                "avg_num_nodes": avg_num_steps, # Renaming "num_steps" to "num_nodes" to match your desired output
                "stddev_num_nodes": stddev_num_steps
            })
    
    # Sort the output for consistent order (optional)
    final_output_sorted = sorted(final_output, key=lambda x: (x['planner'], x['environment']))

    # Print the output in JSON format
    print(json.dumps(final_output_sorted, indent=2))
    with open('result_mppi_cbf.txt', 'w') as f:
        json.dump(final_output_sorted, f, indent=2)


if __name__ == '__main__':
    main()