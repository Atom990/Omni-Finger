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

depth_list = [0.008, 0.010, 0.012, 0.014, 0.016, 0.018, 0.020]  # press depth
D = 0.005  # diameter


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
    init_pose_col_1_finger = np.array([0.01285, 0.0028, 0.12191, 102 * np.pi / 180, 0, 0])

    T = init_pose_col_1_base - init_pose_col_1_finger

    init_pose_col_2_base = np.array([-0.01285, 0.0028, 0.12191, 102 * np.pi / 180, 0, 0]) + T

    init_pose_row_1_base = np.array([0.010, 0.00322, 0.12264, 90 * np.pi / 180, 0, 0]) + T
    init_pose_row_2_base = np.array([0.010, 0.01189, 0.0791, 102 * np.pi / 180, 0, 0]) + T
    init_pose_row_3_base = np.array([0.0125, 0.01948, 0.04342, 102 * np.pi / 180, 0, 0]) + T

    pose = []
    sensor_data = []
    press_info_data = []

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

    for d in depth_list:
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
            force_data = FTSensor.GetReading(1000)

            pose_finger = pose[i] - T

            sensor_data.append(force_data)

            press_info_data.append(np.hstack((D, d, pose_finger[:3])))
            cv2.imwrite('./Data/Diameter_{Diameter}mm/depth_{depth}mm/{index}.jpg'.format(Diameter=D, depth=d, index=i), picture)

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
            ret, picture = cam.read()
            force_data = FTSensor.GetReading(1000)

            pose_finger = pose[i] - T

            sensor_data.append(force_data)

            press_info_data.append(np.hstack((D, d, pose_finger[:3])))
            cv2.imwrite('./Data/Diameter_{Diameter}mm/depth_{depth}mm/{index}.jpg'.format(Diameter=D, depth=d, index=i), picture)

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
            ret, picture = cam.read()
            force_data = FTSensor.GetReading(1000)

            pose_finger = pose[i] - T

            sensor_data.append(force_data)

            press_info_data.append(np.hstack((D, d, pose_finger[:3])))
            cv2.imwrite('./Data/Diameter_{Diameter}mm/depth_{depth}mm/{index}.jpg'.format(Diameter=D, depth=d, index=i), picture)

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
            ret, picture = cam.read()
            force_data = FTSensor.GetReading(1000)

            pose_finger = pose[i] - T

            sensor_data.append(force_data)

            press_info_data.append(np.hstack((D, d, pose_finger[:3])))
            cv2.imwrite('./Data/Diameter_{Diameter}mm/depth_{depth}mm/{index}.jpg'.format(Diameter=D, depth=d, index=i), picture)

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
            ret, picture = cam.read()
            force_data = FTSensor.GetReading(1000)

            pose_finger = pose[i] - T

            sensor_data.append(force_data)

            press_info_data.append(np.hstack((D, d, pose_finger[:3])))
            cv2.imwrite('./Data/Diameter_{Diameter}mm/depth_{depth}mm/{index}.jpg'.format(Diameter=D, depth=d, index=i), picture)

            robot.move_p(pose[i])

    sensor_data = np.array(sensor_data)
    press_info_data = np.array(press_info_data)

    np.save('./Data/Diameter_{Diameter}mm/{Diameter}_ForceSensor.npy'.format(Diameter=D), sensor_data)
    np.save('./Data/Diameter_{Diameter}mm/{Diameter}_PressInfo.npy'.format(Diameter=D), press_info_data)
