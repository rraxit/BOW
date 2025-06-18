import json

def get_benchmark_metrics(file_path, planner_name=None, environment_name=None):
    """
    Extracts average and standard deviation of path length, real time,
    average velocity, and average jerk for specified planner and environment
    from a JSON benchmark file.

    Args:
        file_path (str): The path to the JSON benchmark file.
        planner_name (str, optional): The name of the planner to filter by (e.g., "DWA", "RRT", "BOW").
                                      If None, data for all planners will be included.
        environment_name (str, optional): The environment identifier to filter by (e.g., "1", "4").
                                         If None, data for all environments will be included.

    Returns:
        list: A list of dictionaries, where each dictionary contains the
              extracted average and standard deviation metrics for a matching planner/environment.
              Returns an empty list if the file is not found or no matching data.
    """
    results = {}
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
        return []
    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from {file_path}")
        return []

    benchmarks = data.get('benchmarks', [])

    for entry in benchmarks:
        if entry.get('run_type') == 'aggregate':
            name_parts = entry.get('name', '').split('/')
            if len(name_parts) == 2:
                current_planner = name_parts[0]
                env_part_full = name_parts[1]

                aggregate_name = entry.get('aggregate_name')

                # Determine environment name based on aggregate_name
                if aggregate_name:
                    current_environment = env_part_full.replace(f'_{aggregate_name}', '')
                else:
                    current_environment = env_part_full # Fallback if aggregate_name is missing

                planner_match = (planner_name is None) or (current_planner == planner_name)
                environment_match = (environment_name is None) or (current_environment == environment_name)

                if planner_match and environment_match:
                    key = (current_planner, current_environment)
                    if key not in results:
                        results[key] = {
                            'planner': current_planner,
                            'environment': current_environment,
                            'avg_path_length': None,
                            'stddev_path_length': None,
                            'avg_time_ms': None,
                            'stddev_time_ms': None,
                            'avg_velocity': None,
                            'stddev_velocity': None,
                            'avg_jerk': None,
                            'stddev_jerk': None,
                            'num_nodes': None
                        }

                    if aggregate_name == 'mean':
                        results[key]['avg_path_length'] = entry.get('path_length')
                        results[key]['avg_time_ms'] = entry.get('real_time')
                        results[key]['avg_velocity'] = entry.get('avg_velocity')
                        results[key]['avg_jerk'] = entry.get('avg_jerk')
                        results[key]['avg_num_nodes'] = entry.get('num_nodes')
                    elif aggregate_name == 'stddev':
                        results[key]['stddev_path_length'] = entry.get('path_length')
                        results[key]['stddev_time_ms'] = entry.get('real_time')
                        results[key]['stddev_velocity'] = entry.get('avg_velocity')
                        results[key]['stddev_jerk'] = entry.get('avg_jerk')
                        results[key]['stddev_num_nodes'] = entry.get('num_nodes')
    return list(results.values())

# Example Usage:

# To get metrics for a specific planner and environment (e.g., DWA in environment 1)
# planner = "DWA"
# environment = "1"
# dwa_env1_metrics = get_benchmark_metrics('result.json', planner, environment)
# print(f"Metrics for Planner '{planner}' in Environment '{environment}':")
# for item in dwa_env1_metrics:
#     print(json.dumps(item, indent=2))

# print("\n" + "="*50 + "\n")

# To get metrics for all planners and environments
all_metrics = get_benchmark_metrics('/home/airlab/Cppdev/bow_benchmark/build/benchmark/result_nonconvex.json')
print("Metrics for All Planners and Environments:")
for item in all_metrics:
    print(json.dumps(item, indent=2))
    open('/home/airlab/Cppdev/bow_benchmark/build/benchmark/result_nonconvex.txt', 'w').write(json.dumps(all_metrics, indent=2))