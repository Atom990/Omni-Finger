import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as dc
import numpy as np
import Finger


def press_finger(init_pose_, finger_, depth, move=True):
    next_pose_ = init_pose_ + np.array([0, -depth * np.sin(finger_.theta_1), -depth * np.cos(finger_.theta_1), 0, 0, 0])
    if move == True:
        robot.move_p(next_pose_, 0.1, 0.1)
    return next_pose_


def move_to_next_point(init_pose_, finger, move_direction, stride, move=True):
    if move_direction == 'horizontal':
        next_pose_ = init_pose_ + np.array([-stride, 0, 0, 0, 0, 0])
        if move == True:
            robot.move_p(next_pose_, 0.1, 0.1)
    elif move_direction == 'vertical_col_1':
        next_pose_ = init_pose_ + np.array([stride * np.cos(finger.beta), stride * np.cos(finger.alpha),
                                            -stride * np.sin(finger.alpha) * np.sin(finger.theta_2), 0, 0, 0])
        if move == True:
            robot.move_p(next_pose_, 0.1, 0.1)
    elif move_direction == 'vertical_col_2':
        next_pose_ = init_pose_ + np.array([-stride * np.cos(finger.beta), stride * np.cos(finger.alpha),
                                            -stride * np.sin(finger.alpha) * np.sin(finger.theta_2), 0, 0, 0])
        if move == True:
            robot.move_p(next_pose_, 0.1, 0.1)
    else:
        next_pose_ = init_pose_
        print('Wrong parameters!')
    return next_pose_


robot = dc.UR10eController('./DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')
small_finger = Finger.Finger([78 * np.pi / 180, 84.29 * np.pi / 180], 78.06 * np.pi / 180, 84.41 * np.pi / 180)

calibration_point_pose_base = np.array([0.374560, -0.174580, -0.015080, 2.221, 2.221, 0])
calibration_point_pose_finger = np.array([-0.250, 0.350, -0.077, 2.221, 2.221, 0])

# transform vector between the two frames
T = calibration_point_pose_base - calibration_point_pose_finger

init_pose_col_1_base = press_finger(np.array([0.0090, 0.00196, 0.08542, 1.782, 0, 0]), small_finger, -0.00296, False) + T
robot.move_p(init_pose_col_1_base, 0.1, 0.1)
