import numpy as np
import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as controller

h1 = 0.1
h2 = 0.1
h3 = 0.1
D = 0.1
W1 = 0.1
W2 = 0.1

x0 = 0.1
y0 = 0.2
z0 = 0.3
roll0 = 0
pitch0 = 0
yaw0 = 0

depth_1 = 0.01
depth_2 = 0.02
depth_3 = 0.03
depth_4 = 0.04

theta = 0  # rad

init_pose_0 = np.array([x0, y0, z0, roll0, pitch0, yaw0])
init_pose_1 = np.array([W2, 0, h3, roll0, pitch0, yaw0])

P = init_pose_0 - init_pose_1

robot = UR10eController(
    '.DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')


def cal_move_in_pose_0(side, depth, init_pose_0_):
    if side == 0:
        move_in_pose = [init_pose_0_[0], init_pose_0_[1] - depth * np.cos(theta), init_pose_0_[2] - depth * np.sin(theta), roll0, pitch0, yaw0]
    elif side == 1:
        move_in_pose = [init_pose_0_[0], init_pose_0_[1] + depth * np.cos(theta), init_pose_0_[2] - depth * np.sin(theta), roll0, pitch0, yaw0]

    return move_in_pose


if __name__ == "__main__":
    # move to initial state
    robot.move_p(init_pose_0, 0.8, 1.2)
