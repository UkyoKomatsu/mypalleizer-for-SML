"""Inspect Cartesian IK solutions for the configured search pose."""
import numpy as np
import math
from scipy.optimize import least_squares

from socket_bridge import fk, LIMITS


def solve(goal, seed):
    lower, upper = np.array(LIMITS).T

    def error(candidate):
        pose = fk(candidate)
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        return [*((pose[:3, 3] - goal[:3]) * 10),
                math.atan2(math.sin(yaw - goal[3]), math.cos(yaw - goal[3]))]

    result = least_squares(error, np.clip(seed, lower, upper), bounds=(lower, upper), max_nfev=100)
    pose = fk(result.x)
    return np.degrees(result.x), pose[:3, 3], np.linalg.norm(pose[:3, 3] - goal[:3])


def main():
    goal = np.array([0.15, 0, 0.2, 0])
    for seed in (
        [0, 0, 0, 0],
        [0, -0.03, 0.4, 0],
        [0, 0.2, -0.4, 0],
        [0, 0.5, 0.5, 0],
    ):
        print('seed:', seed, 'result:', solve(goal, seed))


if __name__ == '__main__':
    main()
