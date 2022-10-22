from re import X
import numpy as np

from scipy.spatial.transform import Rotation
from tqdm import tqdm

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

def scipy_yaw_from_quat(quat):
    R = Rotation.from_quat(np.array([quat.x, quat.y, quat.z, quat.w]))
    r, p, y = R.as_euler('xyz', degrees=False)
    return y

def yaw_from_quat(quat):
    return np.arctan2(2*(quat.w*quat.z + quat.x*quat.y), 1-2*(quat.y**2+quat.z**2))

if __name__ == '__main__':
    for q in tqdm(np.linspace(0, 360, 3600)):
        R = Rotation.from_euler('xyz', np.array([0, 0, q]))
        q = Quaternion(*R.as_quat())
        assert np.round(scipy_yaw_from_quat(q), 9) == np.round(yaw_from_quat(q), 9), (scipy_yaw_from_quat(q), yaw_from_quat(q))
