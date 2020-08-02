import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as Controller
import numpy as np
import cv2
import Finger
import FTReading

big_finger = Finger.Finger(0, 0, [0.03042, 0.05542, 0.08601], [78 * np.pi / 180, 84.29 * np.pi / 180],
                           78.06 * np.pi / 180, 84.41 * np.pi / 180)

depth_list = [0.008, 0.010, 0.012, 0.014, 0.016, 0.018, 0.020]  # press depth
D = 0.015  # diameter


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


if __name__ == '__main__':
    robot = Controller.UR10eController('./DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')

    # set the very first initial pose
    #             _row 1_
    #            /       \
    # column 1  /__row 2__\  column 2
    #          /           \
    #         /____row 3____\

    calibration_point_pose_base = np.array([0.478590, -0.417430, 0.152090, 1.362, 1.362, -1.103])
    calibration_point_pose_finger = np.array([0.00428, 0.01189, 0.0791, 1.362, 1.362, -1.103])

    T = calibration_point_pose_base - calibration_point_pose_finger

    init_pose_col_1_base = np.array([0.01285, 0.0028, 0.12191, 1.362, 1.362, -1.103]) + T
    init_pose_col_2_base = np.array([-0.01285, 0.0028, 0.12191, 1.362, 1.362, -1.103]) + T

    init_pose_row_1_base = np.array([0.010, 0.00322, 0.12264, 1.362, 1.362, -1.103]) + T
    init_pose_row_2_base = np.array([0.010, 0.01189, 0.0791, 1.362, 1.362, -1.103]) + T
    init_pose_row_3_base = np.array([0.0125, 0.01948, 0.04342, 1.362, 1.362, -1.103]) + T

    pose = []

    col_1_point = 1
    col_2_point = 1
    row_1_point = 1
    row_2_point = 1
    row_3_point = 1

    col_1_index = range(0, col_1_point)
    col_2_index = range(col_1_index.stop, col_1_index.stop + col_2_point)
    row_1_index = range(col_2_index.stop, col_2_index.stop + row_1_point)
    row_2_index = range(row_1_index.stop, row_1_index.stop + row_2_point)
    row_3_index = range(row_2_index.stop, row_2_index.stop + row_3_point)

    robot.move_p(calibration_point_pose_base, 0.1, 0.1)

    for d in depth_list:
        # start
        # column 1
        for i in col_1_index:
            if i == col_1_index[0]:
                pose.append(init_pose_col_1_base)
                robot.move_p(pose[i])
            else:
                pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                               finger=big_finger,
                                               move_direction='vertical_col_1',
                                               stride=0.005,
                                               move=True))
            press_finger(pose[i], big_finger, d, move=True)

            pose_finger = pose[i] - T

            robot.move_p(pose[i])

        # column 2
        for i in col_2_index:
            if i == col_2_index[0]:
                pose.append(init_pose_col_2_base)
                robot.move_p(pose[i])
            else:
                pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                               finger=big_finger,
                                               move_direction='vertical_col_2',
                                               stride=0.005,
                                               move=True))
            press_finger(pose[i], big_finger, d, move=True)

            pose_finger = pose[i] - T

            robot.move_p(pose[i])

        # row 1
        for i in row_1_index:
            if i == row_1_index[0]:
                pose.append(init_pose_row_1_base)
                robot.move_p(pose[i])
            else:
                pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                               finger=big_finger,
                                               move_direction='horizontal',
                                               stride=0.005,
                                               move=True))
            press_finger(pose[i], big_finger, d, move=True)

            pose_finger = pose[i] - T

            robot.move_p(pose[i])

        # row 2
        for i in row_2_index:
            if i == row_2_index[0]:
                pose.append(init_pose_row_2_base)
                robot.move_p(pose[i])
            else:
                pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                               finger=big_finger,
                                               move_direction='horizontal',
                                               stride=0.005,
                                               move=True))
            press_finger(pose[i], big_finger, d, move=True)

            pose_finger = pose[i] - T

            robot.move_p(pose[i])

        # row 3
        for i in row_3_index:
            if i == row_3_index[0]:
                pose.append(init_pose_row_3_base)
                robot.move_p(pose[i])
            else:
                pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                               finger=big_finger,
                                               move_direction='horizontal',
                                               stride=0.005,
                                               move=True))
            press_finger(pose[i], big_finger, d, move=True)

            pose_finger = pose[i] - T

            robot.move_p(pose[i])
