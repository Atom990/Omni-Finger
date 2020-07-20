import numpy as np
import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as Controller
import DataAcquisition.Finger as Finger

depth_1 = 0
depth_2 = 0
depth_3 = 0
depth_4 = 0

finger = Finger.Finger(0, 0, [1, 2, 3], [4, 5], 0, 0)

init_pose_base = np.array([0, 0, 0, 0, 0, 0])
init_pose_finger = np.array([finger.a / 2, 0, finger.h_3, 0, 0, 0])


def move_to_next_point(init_pose_, move_direction, stride):
    if move_direction == 'horizontal':
        next_pose = init_pose_ + np.array([
            -stride, 0, 0, 0, 0, 0])
    elif move_direction == 'vertical':
        next_pose = init_pose_ + np.array([
            -stride * np.cos(finger.beta), -stride * np.cos(finger.alpha),
            stride * np.sin(finger.alpha) * np.sin(finger.theta_2), 0, 0, 0])
    else:
        next_pose = init_pose_
        print('Wrong parameters!')
    return next_pose


def press_finger(init_pose_, depth):
    next_pose = init_pose_ + np.array([0, -depth * np.sin(finger.theta_1), -depth * np.cos(finger.theta_1), 0, 0, 0])
    return next_pose


if __name__ == '__main__':
    robot = Controller.UR10eController('../DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')
    # move to the initial point
    robot.move_p(init_pose_base, 0.5, 1)
    # get the transformation vector: pose_base = pose_finger + T
    T = init_pose_base - init_pose_finger

    # TODO
    # path 1: from init point to move downwards

    # path 2: go back to the init point, move horizontally

    # path 3: move downwards

    # path 4: move horizontal

    # path 5: move horizontal

    # save data
