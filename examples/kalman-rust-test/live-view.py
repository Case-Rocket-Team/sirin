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
    r"rot_quaternion:\s*\[\s*([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*\]"
)

accel_re = re.compile(
    r"accel:\s*\[\[\s*([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*\]\]"
)

# Regex to extract magnetometer
mag_re = re.compile(
    r"Magnetometer:\s*\(\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)\s*\)"
)

grav_re = re.compile(
    r"gravity:\s*\[\[(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?),\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?),\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\]\]"
)

debug_re = re.compile(
    r"debug:\s*\[\[(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?),\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?),\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\]\]"
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
def normalize(v):
    n = np.linalg.norm(v)
    if n > 1e-6:
        return v / n
    return v

# Shared vector basis
basis = {
    "x": np.array([1, 0, 0], dtype=float),
    "y": np.array([0, 1, 0], dtype=float),
    "z": np.array([0, 0, 1], dtype=float),
}

last_mag = None
R = np.eye(3)

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
ax.set_title("Live Quaternion Orientation — Basis + Magnetic Field")

# Basis vectors
line_x, = ax.plot([0, 1], [0, 0], [0, 0], color="red", linewidth=3, label="X axis")
line_y, = ax.plot([0, 0], [0, 1], [0, 0], color="green", linewidth=3, label="Y axis")
line_z, = ax.plot([0, 0], [0, 0], [0, 1], color="blue", linewidth=3, label="Z axis")
# Acceleration vector
line_accel, = ax.plot(
    [0, 0], [0, 0], [0, 0],
    color="cyan", linewidth=3, label="Acceleration"
)
line_debug, = ax.plot(
    [0, 0], [0, 0], [0, 0],
    color="purple", linewidth=3, label="Debug"
)
# Magnetic field line
line_mag, = ax.plot([0, 0], [0, 0], [0, 0],
                    color="gold", linewidth=2, linestyle="--", label="Mag Field")

ax.legend()

for line in sys.stdin:
    dirty = False
    am = accel_re.search(line)
    if am:
        ax_, ay_, az_ = map(float, am.groups())
        accel = normalize(np.array([ax_, ay_, az_], dtype=float))

        print(f"Received accel: x={ax_}, y={ay_}, z={az_}")

        line_accel.set_data([0, accel[0]], [0, accel[1]])
        line_accel.set_3d_properties([0, accel[2]])
        dirty = True
    dm = debug_re.search(line)
    if dm:
        ax_, ay_, az_ = map(float, dm.groups())
        debug = normalize(np.array([ax_, ay_, az_], dtype=float))

        print(f"Received grav: x={ax_}, y={ay_}, z={az_}")

        line_debug.set_data([0, debug[0]], [0, debug[1]])
        line_debug.set_3d_properties([0, debug[2]])
        dirty = True
    # Quaternion update
    qm = quat_re.search(line)
    if qm:
        x, y, z, r = map(float, qm.groups())
        print(f"Received quaternion: r={r}, x={x}, y={y}, z={z}")
        R = quat_to_matrix(r, x, y, z)

        rx = R @ basis["x"]
        ry = R @ basis["y"]
        rz = R @ basis["z"]

        line_x.set_data([0, rx[0]], [0, rx[1]])
        line_x.set_3d_properties([0, rx[2]])

        line_y.set_data([0, ry[0]], [0, ry[1]])
        line_y.set_3d_properties([0, ry[2]])

        line_z.set_data([0, rz[0]], [0, rz[1]])
        line_z.set_3d_properties([0, rz[2]])
        dirty=True

    # Magnetometer update
    mm = mag_re.search(line)
    if mm:
        mx, my, mz = map(float, mm.groups())
        print(f"Received mag: x={mx}, y={my}, z={mz}")
        mag_body = np.array([mx, my, mz], dtype=float)

        # Rotate magnetometer into world frame
        # mag_world = R @ mag_body

        # Normalize for visualization
        mag_world = normalize(mag_body)

        line_mag.set_data([0, mag_world[0]], [0, mag_world[1]])
        line_mag.set_3d_properties([0, mag_world[2]])
        dirty = True
    
    if dirty:
        plt.pause(0.01)

