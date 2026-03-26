#!/usr/bin/env -S uv run

# live view of acceleration + raw magnetometer (no rotation)

# uv: numpy
# uv: matplotlib

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Regex patterns

quat_re = re.compile(
    r"Quaternion\s*\{\s*r:\s*([-\d\.eE]+),\s*x:\s*([-\d\.eE]+),\s*y:\s*([-\d\.eE]+),\s*z:\s*([-\d\.eE]+)"
)
accel_re = re.compile(
    r"accel:\s*Accel\s*\{\s*x:\s*([-\d\.eE]+),\s*y:\s*([-\d\.eE]+),\s*z:\s*([-\d\.eE]+)\s*\}"
)

mag_re = re.compile(
    r"Magnetometer:\s*\[\s*([-+\d\.eE]+)\s*,\s*([-+\d\.eE]+)\s*,\s*([-+\d\.eE]+)\s*\]"
)

def normalize(v):
    n = np.linalg.norm(v)
    if n > 1e-6:
        return v / n
    return v

# Matplotlib interactive plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_proj_type("persp")
ax.set_box_aspect((1, 1, 1))

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Live Acceleration & Raw Magnetometer")

# Acceleration vector
line_accel, = ax.plot(
    [0, 0], [0, 0], [0, 0],
    color="cyan", linewidth=3, label="Acceleration"
)

# Magnetometer vector (raw, unrotated)
line_mag, = ax.plot(
    [0, 0], [0, 0], [0, 0],
    color="gold", linewidth=2, linestyle="--", label="Magnetometer"
)


ax.legend()

for line in sys.stdin:
    # Accelerometer update
    dirty = False
    # am = accel_re.search(line)
    am = False
    if am:
        ax_, ay_, az_ = map(float, am.groups())
        accel = normalize(np.array([ax_, ay_, az_], dtype=float))

        print(f"Received accel: x={ax_}, y={ay_}, z={az_}")

        line_accel.set_data([0, accel[0]], [0, accel[1]])
        line_accel.set_3d_properties([0, accel[2]])
        dirty = True

    # Magnetometer update (NO rotation)
    mm = mag_re.search(line)
    if mm:
        mx, my, mz = map(float, mm.groups())
        mag = normalize(np.array([mx, my, mz], dtype=float))

        print(f"Received mag: x={mx}, y={my}, z={mz}")

        line_mag.set_data([0, mag[0]], [0, mag[1]])
        line_mag.set_3d_properties([0, mag[2]])
        dirty = True
    
    
    if dirty:
        plt.pause(0.01)