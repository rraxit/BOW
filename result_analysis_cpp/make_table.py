import json
import numpy as np
from collections import defaultdict

def load_json_data(filename):
    """Load JSON data from file"""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def get_lib_lang(planner):
    """Get the library and language for each planner"""
    lib_lang_map = {
        'BOW': 'FCL + C++',
        'DWA': 'FCL + C++', 
        'HRVO': 'C++',
        'RRT': 'FCL + C++',
        'MPPI': 'CUDA + Py',
        'CBF': 'Gurobi + Py',
    }
    return lib_lang_map.get(planner, 'Unknown')

def organize_data_by_environment(data):
    """Organize data by environment and planner"""
    env_data = defaultdict(dict)
    
    for entry in data:
        env = entry['environment']
        planner = entry['planner']
        env_data[env][planner] = entry
    
    return env_data

def format_value_with_std(avg, std, decimal_places=2):
    """Format average ± standard deviation"""
    if std == 0 or std is None or np.isnan(std):
        return f"{avg:.{decimal_places}f} $\\pm$ 0.00"
    else:
        return f"{avg:.{decimal_places}f} $\\pm$ {std:.{decimal_places}f}"

def generate_latex_table(data, output_file=None):
    """Generate LaTeX table from JSON data"""
    
    # Organize data by environment
    env_data = organize_data_by_environment(data)
    
    # Define planner order
    planner_order = ['BOW', 'DWA', 'HRVO', 'RRT','MPPI', 'CBF']
    
    # Start building LaTeX table
    latex_lines = [
        "\\begin{table*}[t]",
        "    \\centering",
        "    \\setlength{\\tabcolsep}{5pt}",
        "    \\begin{tabular}{@{}llcrrrrrr@{}}",
        "        \\toprule",
        "        \\textbf{Env} & \\textbf{Method} & \\textbf{Lib + Lang} & \\textbf{Steps} & \\textbf{Traj. Length (m)} & \\textbf{Total Time (ms)} & \\textbf{Time / Step (ms)} & \\textbf{Avg Velocity} & \\textbf{Avg Jerk} \\\\",
        "        \\midrule"
    ]
    
    # Sort environments
    sorted_envs = sorted(env_data.keys())
    
    for i, env in enumerate(sorted_envs):
        planners_in_env = [p for p in planner_order if p in env_data[env]]
        num_planners = len(planners_in_env)
        
        if num_planners == 0:
            continue
            
        # Add multirow for environment
        latex_lines.append(f"        \\multirow{{{num_planners}}}{{*}}{{{env}}}")
        
        for j, planner in enumerate(planners_in_env):
            entry = env_data[env][planner]
            
            # Extract values
            path_length = entry['avg_path_length']
            path_std = entry['stddev_path_length']
            time_ms = entry['avg_time_ms']
            time_std = entry['stddev_time_ms']
            velocity = entry['avg_velocity']
            velocity_std = entry['stddev_velocity']
            jerk = entry['avg_jerk']
            jerk_std = entry['stddev_jerk']
            steps = entry.get('avg_num_nodes')
            steps_std = entry.get('stddev_num_nodes')
            
            # Calculate time per step
            if steps > 0:
                time_per_step = time_ms / steps
                time_per_step_str = f"{time_per_step:.2f}"
            else:
                time_per_step_str = "—"
            
            # Get library and language
            lib_lang = get_lib_lang(planner)
            
            # Format values
            path_str = format_value_with_std(path_length, path_std)
            time_str = format_value_with_std(time_ms, time_std)
            velocity_str = "—" if np.isnan(velocity) or np.isinf(velocity) else format_value_with_std(velocity, velocity_std, 3)
            jerk_str = format_value_with_std(jerk, jerk_std, 3)
            steps_str = format_value_with_std(steps, steps_std, 1) if steps > 0 else "0.0 $\\pm$ 0.0"
            
            # Build row
            if j == 0:
                row = f"            & {planner} & {lib_lang} & {steps_str} & {path_str} & {time_str} & {time_per_step_str} & {velocity_str} & {jerk_str} \\\\"
            else:
                row = f"            & {planner} & {lib_lang} & {steps_str} & {path_str} & {time_str} & {time_per_step_str} & {velocity_str} & {jerk_str} \\\\"
            
            latex_lines.append(row)
        
        # Add midrule between environments (except last)
        if i < len(sorted_envs) - 1:
            latex_lines.append("        \\midrule")
    
    # Close table
    latex_lines.extend([
        "        \\bottomrule",
        "    \\end{tabular}",
        "    \\caption{Comparison of path planning methods across six environments. Missing entries indicate that the planner was not tested or failed in that environment.}",
        "    \\label{tab:comparison}",
        "    \\vspace{-10pt}",
        "\\end{table*}"
    ])
    
    latex_table = '\n'.join(latex_lines)
    
    # Save to file if specified
    if output_file:
        with open(output_file, 'w') as f:
            f.write(latex_table)
        print(f"LaTeX table saved to {output_file}")
    
    return latex_table

def main():
    # Load data from JSON file
    try:
        data = load_json_data('/home/airlab/Cppdev/bow_benchmark/build/benchmark/result_all.txt')
    except FileNotFoundError:
        print("Error: result.txt not found. Please make sure the file exists.")
        return
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON: {e}")
        return
    
    # Generate LaTeX table
    latex_table = generate_latex_table(data, 'planning_comparison_table_all.tex')
    
    # Print to console
    print("\nGenerated LaTeX table:")
    print("=" * 80)
    print(latex_table)

if __name__ == "__main__":
    main()