import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import yaml



def plot_nonconvex_polygon(vertices, color='blue', alpha=0.5):
    """
    Plots a non-convex polygon defined by its vertices.

    Parameters:
    - vertices: A list or array of (x, y) tuples defining the polygon vertices.
    - color: Color of the polygon.
    - alpha: Transparency level of the polygon.
    """


    # Create a Polygon patch
    polygon = Polygon(vertices, closed=True, color=color, alpha=alpha)

    # Add the polygon to the axes
    ax.add_patch(polygon)




if __name__ == '__main__':
    # Define vertices of a non-convex polygon

    # vertices = (
    #     np.array([[180.0, 274.0, 225.0, 212, 148], [23.0, 46.0, 97.0, 61.0, 83.0]]).T,
    #     np.array([[33.0, 67.0, 75.0, 128.0, 84.0, 44.0], [40.0, 55.0, 109.0, 97.0, 149.0, 94.0]]).T,
    #     np.array([[189.0, 250.0, 158.0], [102.0, 165.0, 132.0]]).T
    # )

    with open('../test/env_nonconvex.yaml', 'r') as file:
        data = yaml.safe_load(file)['triangles']
        vertices = np.array(data).reshape((-1, 3, 2))

    fig, ax = plt.subplots()

    # Plot the non-convex polygon
    for poly in vertices:
        poly = poly
        plot_nonconvex_polygon(poly, color='green', alpha=0.6)

    # Set limits and aspect
    # ax.set_xlim(min(vertices, key=lambda v: v[0])[0] - 1, max(vertices, key=lambda v: v[0])[0] + 1)
    # ax.set_ylim(min(vertices, key=lambda v: v[1])[1] - 1, max(vertices, key=lambda v: v[1])[1] + 1)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)

    plt.title('Non-Convex Polygon')
    plt.grid()
    plt.show()