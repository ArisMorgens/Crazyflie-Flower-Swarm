import os
import json
import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def generate_petal(a=0.5, b=0.8, num_points=20):
    """Generate a single teardrop-shaped petal in the x-z plane. Tip starts at origin."""
    t_vals = np.linspace(0, np.pi, num_points)
    points = []
    for t in t_vals:
        x = a * np.sin(t)
        y = 0.0
        z = b * (1 - np.cos(t)) * np.sin(t)
        points.append(np.array([x, y, z]))
    return np.array(points)


def rotate_points(points, angle):
    """Rotate points around the Z-axis."""
    R = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,              0,             1]
    ])
    return points @ R.T


def generate_flower(num_petals=5, a=0.5, b=0.8, num_points=20, center=(0, 0, 0)):
    """Generate multiple petals evenly rotated around a center point."""
    base_petal = generate_petal(a, b, num_points)
    center = np.array(center)
    all_petals = []
    for k in range(num_petals):
        angle = 2 * np.pi * k / num_petals
        rotated = rotate_points(base_petal, angle)
        translated = rotated + center
        # Close the loop: last point = first point
        closed_petal = np.vstack([translated, translated[0]])
        all_petals.append(closed_petal)
    return all_petals


def generate_stem(center=(0, 0, 1), num_points=10, curvature=0.2):
    """Generate a stem that starts straight and bends near the top, constrained to the plane defined by the origin and center."""
    center = np.array(center)
    t_vals = np.linspace(0, 1, num_points)

    d = center
    d_hat = d / np.linalg.norm(d)

    # Build an in-plane bend direction
    arbitrary = np.array([0, 0, 1]) if abs(d_hat[2]) < 0.9 else np.array([1, 0, 0])
    n = np.cross(d_hat, arbitrary)
    n /= np.linalg.norm(n)
    u = np.cross(n, d_hat)
    u /= np.linalg.norm(u)

    points = []
    for t in t_vals:
        main = d * t
        bend_mag = curvature * np.sin(np.pi * t**2)  # delayed bend near the top
        points.append(main + bend_mag * u)
    return np.array(points)


def save_trajectories(flowers, save_dir):
    """Save petals and stem of each flower as JSON files for the Rust swarm script."""
    os.makedirs(save_dir, exist_ok=True)
    for idx, flower in enumerate(flowers):
        with open(os.path.join(save_dir, f'petals{idx+1}.json'), 'w') as f:
            json.dump([petal.tolist() for petal in flower['petals']], f)
        with open(os.path.join(save_dir, f'stem{idx+1}.json'), 'w') as f:
            json.dump(flower['stem'].tolist(), f)


def plot_flowers(flowers):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for flower in flowers:
        for petal in flower['petals']:
            ax.plot(petal[:, 0], petal[:, 1], petal[:, 2], color=flower['petal_color'])
        ax.plot(flower['stem'][:, 0], flower['stem'][:, 1], flower['stem'][:, 2],
                linewidth=3, color='#006400')
        c = flower['center']
        ax.scatter([c[0]], [c[1]], [c[2]], s=80)

    ax.scatter([0], [0], [0], s=50)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Flower Swarm Trajectories")

    # Equal aspect ratio
    ranges = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()])
    max_range = (ranges[:, 1] - ranges[:, 0]).max() / 2.0
    mid = [np.mean(ax.get_xlim()), np.mean(ax.get_ylim()), np.mean(ax.get_zlim())]
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    plt.show()


if __name__ == "__main__":
    # (num_petals, a, b, center, stem_points, stem_curvature, petal_points, petal_color)
    flower_params = [
        (5, 0.4, 0.4, (0.0,  0.0,  0.45), 15, 0.06, 20, '#FF69B4'),  # hot pink
        (5, 0.4, 0.4, (0.4,  0.2,  0.25), 15, 0.06, 20, '#FFD700'),  # gold
        (5, 0.4, 0.4, (0.4, -0.2,  0.65), 15, 0.06, 20, '#00BFFF'),  # sky blue
        (5, 0.4, 0.4, (-0.4, 0.2,  0.25), 15, 0.06, 20, '#FF4500'),  # orange red
        (5, 0.4, 0.4, (-0.4, -0.2, 0.65), 15, 0.06, 20, '#32CD32'),  # lime green
    ]

    flowers = [
        {
            'petals': generate_flower(num_petals=p, a=a, b=b, num_points=pp, center=center),
            'stem':   generate_stem(center=center, num_points=sp, curvature=sc),
            'center': center,
            'petal_color': color,
        }
        for p, a, b, center, sp, sc, pp, color in flower_params
    ]

    save_trajectories(flowers, os.path.join(os.path.dirname(__file__), 'flower_trajectories'))
    plot_flowers(flowers)
