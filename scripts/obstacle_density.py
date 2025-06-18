import yaml
from argparse import ArgumentParser
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from shapely.geometry import Polygon, box, Point
from scipy.ndimage import gaussian_filter


def compute_obstacle_dispersity(obstacles):
    """Calculate the dispersity of obstacles based on their spatial spread."""
    if len(obstacles) < 2:
        return 0  # No dispersity if there's only one or zero obstacles

    # Extract x and y coordinates
    x_coords = np.array([obs[0] for obs in obstacles])
    y_coords = np.array([obs[1] for obs in obstacles])

    # Compute standard deviation as a measure of dispersity
    dispersity = np.sqrt(np.std(x_coords) ** 2 + np.std(y_coords) ** 2)

    return dispersity

def calculate_obstacle_density(width, height, obstacles, obs_len):
    """
    Calculate obstacle density with two methods:
    1. Simple area ratio calculation
    2. Accurate calculation accounting for overlaps using Shapely
    """
    total_area = width * height

    # Method 1: Simple calculation without accounting for overlaps
    obstacle_area = (obs_len * obs_len) * len(obstacles)
    dispersity =  compute_obstacle_dispersity(obstacles)

    # Method 2: Account for overlaps using Shapely
    polygons = []
    for obs in obstacles:
        # Create a box for each obstacle
        polygons.append(box(obs[0], obs[1], obs[0] + obs_len, obs[1] + obs_len))

    # Handle the case with no obstacles
    if not polygons:
        return 0.0, 0.0, 0.0

    # Calculate actual area considering overlaps
    union_area = calculate_union_area(polygons)
    accurate_density = union_area / total_area


    # Calculate overlap percentage
    overlap_percentage = 0
    if obstacle_area > 0:
        overlap_percentage = (obstacle_area - union_area) / obstacle_area * 100

    return dispersity, accurate_density, overlap_percentage

def calculate_union_area(polygons):
    """Calculate the union area of multiple polygons accounting for overlaps."""
    if not polygons:
        return 0.0

    # Start with the first polygon
    union = polygons[0]

    # Union with each subsequent polygon
    for polygon in polygons[1:]:
        union = union.union(polygon)

    return union.area

def readConfig(filename):
    """Read environment configuration from YAML file."""
    with open(filename) as file:
        data = yaml.safe_load(file)

    obstacles = data['obstacles']
    robot_radius = data['robot_radius']
    start = data['start']
    goal = data['goal']
    boundary = data['boundary']

    # Calculate environment dimensions
    width = abs(boundary[1] - boundary[0])
    height = abs(boundary[3] - boundary[2])
    obs_len = 2 * data['obstacle_length'] + robot_radius

    # Plot environment
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot start and goal
    ax.scatter(start[0], start[1], color='green', s=100, label='Start')
    ax.scatter(goal[0], goal[1], color='red', s=100, label='Goal')

    # Plot obstacles
    for obs in obstacles:
        ax.add_patch(Rectangle((obs[0], obs[1]), obs_len, obs_len,
                               fill=True, color='blue', alpha=0.7))

    # Set plot boundaries
    ax.set_xlim(boundary[0], boundary[1])
    ax.set_ylim(boundary[2], boundary[3])

    # Generate heatmap to visualize obstacle density
    resolution = 100
    x = np.linspace(boundary[0], boundary[1], resolution)
    y = np.linspace(boundary[2], boundary[3], resolution)
    X, Y = np.meshgrid(x, y)

    # Create second subplot for heatmap
    ax.set_title("Environment")
    ax.legend()

    return width, height, obstacles, obs_len, boundary, fig, ax


def visualize_density_heatmap(obstacles, obs_len, boundary, resolution=50):
    """Create a heatmap visualization of obstacle density."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Generate grid
    x = np.linspace(boundary[0], boundary[1], resolution)
    y = np.linspace(boundary[2], boundary[3], resolution)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros((resolution, resolution))

    # Generate polygons for obstacles
    polygons = [box(obs[0], obs[1], obs[0] + obs_len, obs[1] + obs_len) for obs in obstacles]

    # Calculate density at each grid point
    for i in range(resolution):
        for j in range(resolution):
            point = Point(X[i, j], Y[i, j])
            if any(polygon.contains(point) for polygon in polygons):
                Z[i, j] = 1  # Mark grid cell as occupied

    # Apply Gaussian filter for smoother visualization
    Z = gaussian_filter(Z, sigma=1)

    # Plot heatmap
    c = ax.pcolormesh(X, Y, Z, cmap='hot', shading='auto', alpha=0.7)
    fig.colorbar(c, ax=ax, label='Obstacle Density')

    ax.set_title("Obstacle Density Heatmap")
    ax.set_xlabel("X coordinate")
    ax.set_ylabel("Y coordinate")

    return fig, ax

if __name__ == '__main__':
    parser = ArgumentParser(description='Calculate obstacle density')
    parser.add_argument('--config', default="env1.yaml", type=str, help='Path to environment config file')
    parser.add_argument('--plot', action='store_true', help='plot obstacle map')
    parser.add_argument('--heatmap', action='store_true', help='Generate density heatmap')
    args = parser.parse_args()

    # Read configuration and create environment plot
    width, height, obstacles, obs_len, boundary, fig, ax = readConfig(args.config)

    # Calculate density
    dispersity, accurate_density, overlap_percentage = calculate_obstacle_density(
        width, height, obstacles, obs_len
    )

    # Print results
    print(f"Environment dimensions: {width} x {height}")
    print(f"Number of obstacles: {len(obstacles)}")
    print(f"Obstacle size: {obs_len} x {obs_len}")
    print(f"Dispersity calculation: {dispersity:.2f}%")
    print(f"Accurate density (accounting for overlaps): {accurate_density * 100:.2f}%")
    print(f"Overlap percentage: {overlap_percentage:.2f}%")

    # Optional heatmap visualization
    if args.heatmap:
        heatmap_fig, heatmap_ax = visualize_density_heatmap(obstacles, obs_len, boundary)
        plt.figure(heatmap_fig.number)
        plt.savefig('density_heatmap.png', dpi=300, bbox_inches='tight')

    # Finalize and show main plot
    if args.plot:
        plt.figure(fig.number)
        plt.axis('equal')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.savefig('environment.png', dpi=300, bbox_inches='tight')
        plt.show()