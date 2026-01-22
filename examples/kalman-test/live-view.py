#!/usr/bin/env -S uv run

# live view of Sirin's rotation, single-threaded, with logging

# uv: numpy
# uv: matplotlib

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Regex to extract quaternion
quat_re = re.compile(
    r"Quaternion\s*\{\s*r:\s*([-\d\.eE]+),\s*x:\s*([-\d\.eE]+),\s*y:\s*([-\d\.eE]+),\s*z:\s*([-\d\.eE]+)"
)
mag_re = re.compile(
    r"Magnetometer\s*\{\s*x:\s*([-\d\.eE]+),\s*y:\s*([-\d\.eE]+),\s*z:\s*([-\d\.eE]+)"
)

def quat_to_matrix(r, x, y, z):
    q = np.array([r, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])

# Shared vector basis
basis = {
    "x": np.array([1, 0, 0], dtype=float),
    "y": np.array([0, 1, 0], dtype=float),
    "z": np.array([0, 0, 1], dtype=float),
}
rotated = basis.copy()

# Matplotlib interactive plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_proj_type('persp')
ax.set_box_aspect((1, 1, 1))
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Live Quaternion Orientation — Basis Vectors")

# Create 3 plotted vectors (x=red, y=green, z=blue)
line_x, = ax.plot([0, 1], [0, 0], [0, 0], color="red", linewidth=3)
line_y, = ax.plot([0, 0], [0, 1], [0, 0], color="green", linewidth=3)
line_z, = ax.plot([0, 0], [0, 0], [0, 1], color="blue", linewidth=3)

# Read stdin line by line
for line in sys.stdin:
    m = quat_re.search(line)
    if not m:
        continue

    #print(f"Newline: {line}")
    r, x, y, z = map(float, m.groups())
    print(f"Received quaternion: r={r}, x={x}, y={y}, z={z}")  # log

    R = quat_to_matrix(r, x, y, z)
    rotated = {
        "x": R @ basis["x"],
        "y": R @ basis["y"],
        "z": R @ basis["z"],
    }

    # Update plot even if no new quaternion
    rx = rotated["x"]
    ry = rotated["y"]
    rz = rotated["z"]

    line_x.set_data([0, rx[0]], [0, rx[1]])
    line_x.set_3d_properties([0, rx[2]])

    line_y.set_data([0, ry[0]], [0, ry[1]])
    line_y.set_3d_properties([0, ry[2]])

    line_z.set_data([0, rz[0]], [0, rz[1]])
    line_z.set_3d_properties([0, rz[2]])

    plt.pause(0.001)
