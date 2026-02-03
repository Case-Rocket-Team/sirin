import sys
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np

# Regex for magnetometer lines
mag_re = re.compile(
    r"Magnetometer:\s*\[\s*([-+\d\.eE]+)\s*,\s*([-+\d\.eE]+)\s*,\s*([-+\d\.eE]+)\s*\]"
)

xs, ys, zs = [], [], []

# Read from stdin
for line in sys.stdin:
    match = mag_re.search(line)
    if match:
        x, y, z = map(float, match.groups())
        xs.append(x)
        ys.append(y)
        zs.append(z)

if not xs:
    print("No magnetometer data found.")
    sys.exit(1)

xs = np.array(xs)
ys = np.array(ys)
zs = np.array(zs)

# 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.scatter(xs, ys, zs, s=5, c="blue")

ax.set_xlabel("Mag X")
ax.set_ylabel("Mag Y")
ax.set_zlabel("Mag Z")
ax.set_title("Magnetometer Calibration Plot")

# Equal aspect ratio (important for calibration)
max_range = np.array([
    xs.max() - xs.min(),
    ys.max() - ys.min(),
    zs.max() - zs.min()
]).max() / 2.0

mid_x = (xs.max() + xs.min()) * 0.5
mid_y = (ys.max() + ys.min()) * 0.5
mid_z = (zs.max() + zs.min()) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()
