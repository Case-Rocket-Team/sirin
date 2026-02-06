import numpy as np
data = [-0.07777706, -0.9895428, -0.12147297, 0.15793993, -0.13253294, 0.9785143, -0.98438084, 0.056920532, 0.16659516]
rot_mat = np.array(data).reshape(3,3)

[print(np.linalg.norm(rot_mat[:, i])) for i in range(0, 3)]

print(np.linalg.det(rot_mat))
