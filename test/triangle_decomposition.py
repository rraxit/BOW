import yaml
import matplotlib.pyplot as plt
import numpy as np

def load_triangles_from_yaml(filepath):
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    return data['triangles']

def interpolate_edge(p1, p2, step=0.1):
    """Generate points between p1 and p2 at roughly `step` intervals."""
    p1 = np.array(p1)
    p2 = np.array(p2)
    dist = np.linalg.norm(p2 - p1)
    num_points = max(int(dist / step), 1)
    return [p1 + (p2 - p1) * t for t in np.linspace(0, 1, num_points + 1)]

def draw_squares_on_edges(triangles, step=0.1, square_size=0.1):
    fig, ax = plt.subplots()
    for tri in triangles:
        for i in range(3):
            p1 = tri[i]
            p2 = tri[(i + 1) % 3]
            points = interpolate_edge(p1, p2, step=step)
            
            for pt in points:
                with open("points.txt", "a") as f:
                    f.write(f"[{pt[0]:.4f}, {pt[1]:.4f}],\n")
                print(f"[{pt[0]:.4f}, {pt[1]:.4f}],")
                square = plt.Rectangle((pt[0] - square_size/2, pt[1] - square_size/2),
                                       square_size, square_size,
                                       edgecolor='black', facecolor='gray')
                ax.add_patch(square)
    ax.set_aspect('equal')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Approximated Triangle Edges with Squares")
    plt.axis([0.0, 20.0, 0.0, 20.0])
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    triangles = load_triangles_from_yaml("env_nonconvex.yaml")
    draw_squares_on_edges(triangles, step=0.5, square_size=0.25)
