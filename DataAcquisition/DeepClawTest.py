import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as Controller
import numpy as np
import cv2
import Finger
import FTReading

big_finger = Finger.Finger(0, 0, [0.03042, 0.05542, 0.08601], [78 * np.pi / 180, 84.29 * np.pi / 180],
                           78.06 * np.pi / 180, 84.41 * np.pi / 180)
cam = cv2.VideoCapture(2)
FTSensor = FTReading.FTReading("192.168.1.1")
FTSensor.InitFT()
d = 0.004  # press depth


def press_finger(init_pose_, finger_, depth, move=True):
    next_pose_ = init_pose_ + np.array([0, -depth * np.sin(finger_.theta_1), -depth * np.cos(finger_.theta_1), 0, 0, 0])
    if move == True:
        robot.move_p(next_pose_, 0.1, 0.5)
    return next_pose_


def move_to_next_point(init_pose_, finger, move_direction, stride, move=True):
    if move_direction == 'horizontal':
        next_pose_ = init_pose_ + np.array([-stride, 0, 0, 0, 0, 0])
        if move == True:
            robot.move_p(next_pose_, 0.1, 0.5)
    elif move_direction == 'vertical':
        next_pose_ = init_pose_ + np.array([-stride * np.cos(finger.beta), -stride * np.cos(finger.alpha),
                                            -stride * np.sin(finger.alpha) * np.sin(finger.theta_2), 0, 0, 0])
        if move == True:
            robot.move_p(next_pose_, 0.1, 0.5)
    else:
        next_pose_ = init_pose_
        print('Wrong parameters!')
    return next_pose_


if __name__ == '__main__':
    robot = Controller.UR10eController('../DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')

    # set the very first initial pose
    #             _row 1_
    #            /       \
    # column 1  /__row 2__\  column 2
    #          /           \
    #         /____row 3____\
    init_pose_col_1_base = np.array([0.470, -0.560, 0.133, 102 * np.pi / 180, 0, 0])
    init_pose_col_2_base = move_to_next_point(init_pose_=init_pose_col_1_base,
                                              finger=big_finger,
                                              move_direction='horizontal',
                                              stride=0,
                                              move=False)
    init_pose_row_1_base = move_to_next_point(init_pose_=init_pose_col_1_base,
                                              finger=big_finger,
                                              move_direction='horizontal',
                                              stride=0,
                                              move=False)
    init_pose_row_2_base = move_to_next_point(init_pose_=move_to_next_point(init_pose_col_1_base, big_finger, 'vertical', 0, move=False),
                                              finger=big_finger,
                                              move_direction='horizontal',
                                              stride=0,
                                              move=False)
    init_pose_row_3_base = move_to_next_point(init_pose_=move_to_next_point(init_pose_col_1_base, big_finger, 'vertical', 0, move=False),
                                              finger=big_finger,
                                              move_direction='horizontal',
                                              stride=0,
                                              move=False)

    pose = []
    sensor_data = []
    camera_data = []

    col_1_point = 21
    col_2_point = 21
    row_1_point = 5
    row_2_point = 5
    row_3_point = 6

    col_1_index = range(0, col_1_point)
    col_2_index = range(col_1_index.stop, col_1_index.stop + col_2_point)
    row_1_index = range(col_2_index.stop, col_2_index.stop + row_1_point)
    row_2_index = range(row_1_index.stop, row_1_index.stop + row_2_point)
    row_3_index = range(row_2_index.stop, row_2_index.stop + row_3_point)

    # start
    # column 1
    for i in col_1_index:
        if i == col_1_index[0]:
            pose.append(init_pose_col_1_base)
            robot.move_p(pose[0])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=big_finger,
                                           move_direction='vertical',
                                           stride=0.005,
                                           move=True))
        press_finger(pose[i], big_finger, d, move=True)
        ret, picture = cam.read()
        cv2.imwrite('./Pictures/{index}.jpg'.format(index=i), picture)
        robot.move_p(pose[i])

    # column 2
    for i in col_2_index:
        if i == col_2_index[0]:
            pose.append(init_pose_col_2_base)
            robot.move_p(pose[0])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=big_finger,
                                           move_direction='vertical',
                                           stride=0.005,
                                           move=True))
        press_finger(pose[i], big_finger, d, move=True)
        # save sensor data
        # save camera data
        robot.move_p(pose[i])

    # row 1
    for i in row_1_index:
        if i == row_1_index[0]:
            pose.append(init_pose_row_1_base)
            robot.move_p(pose[0])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=big_finger,
                                           move_direction='horizontal',
                                           stride=0.005,
                                           move=True))
        press_finger(pose[i], big_finger, d, move=True)
        # save sensor data
        # save camera data
        robot.move_p(pose[i])

    # row 2
    for i in row_2_index:
        if i == row_2_index[0]:
            pose.append(init_pose_row_1_base)
            robot.move_p(pose[0])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=big_finger,
                                           move_direction='horizontal',
                                           stride=0.005,
                                           move=True))
        press_finger(pose[i], big_finger, d, move=True)
        # save sensor data
        # save camera data
        robot.move_p(pose[i])

    # row 3
    for i in row_3_index:
        if i == row_3_index[0]:
            pose.append(init_pose_row_1_base)
            robot.move_p(pose[0])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=big_finger,
                                           move_direction='horizontal',
                                           stride=0.005,
                                           move=True))
        press_finger(pose[i], big_finger, d, move=True)
        # save sensor data
        # save camera data
        robot.move_p(pose[i])
