import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import sys

# --- regex patterns ---
quat_pattern = r'rot_quaternion:\s*\[([^\]]+)\]'
accel_pattern = r'accel:\s*\[\[([^\]]+)\]\]'
mag_pattern = r'debug:\s*\[\[([^\]]+)\]\]'

# --- storage ---
latest_quat = None
latest_accel = None
latest_mag = None

# --- parsing function ---
def parse_line(line):
    global latest_quat, latest_accel, latest_mag

    q_match = re.search(quat_pattern, line)
    a_match = re.search(accel_pattern, line)
    m_match = re.search(mag_pattern, line)

    if q_match:
        latest_quat = np.fromstring(q_match.group(1), sep=',')
        print("recieved quaternion")

    if a_match:
        latest_accel = np.fromstring(a_match.group(1), sep=',')

    if m_match:
        latest_mag = np.fromstring(m_match.group(1), sep=',')

# --- setup plot ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def update(frame):
    ax.clear()

    # read one line from stdin (non-blocking-ish)
    line = sys.stdin.readline()
    if line:
        parse_line(line)

    # base axes
    origin = np.array([0, 0, 0])

    # plot rotated frame if quaternion exists
    if latest_quat is not None:
        try:
            # scipy uses [x, y, z, w]
            r = R.from_quat(latest_quat)

            axes = np.eye(3)  # unit x,y,z
            rotated_axes = r.apply(axes)

            colors = ['r', 'g', 'b']
            labels = ['X', 'Y', 'Z']

            for i in range(3):
                ax.quiver(*origin, *rotated_axes[i], color=colors[i], length=1.0)

        except Exception as e:
            pass

    # plot accel
    if latest_accel is not None:
        ax.quiver(*origin, *latest_accel, color='black', label='Accel')

    # plot magnetic field
    if latest_mag is not None:
        ax.quiver(*origin, *latest_mag, color='purple', label='Mag')

    # formatting
    ax.set_xlim([-15, 15])
    ax.set_ylim([-15, 15])
    ax.set_zlim([-15, 15])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_title("IMU Visualization")

# animate
ani = FuncAnimation(fig, update, interval=100)
plt.show()