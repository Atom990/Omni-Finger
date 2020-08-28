import numpy as np
import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as dc
import Finger
import cv2
import FTReading


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


small_finger = Finger.Finger([78 * np.pi / 180, 84.29 * np.pi / 180], 78.06 * np.pi / 180, 84.41 * np.pi / 180)

depth_list = [0.010, 0.013, 0.016, 0.019, 0.022]
D = 0.005  # diameter = 5 mm
S = 0.005  # stride = 5 mm

# small finger
calibration_point_pose_base = np.array([0.374560, -0.174580, -0.015080, 2.221, 2.221, 0])
calibration_point_pose_finger = np.array([-0.250, 0.350, -0.077, 2.221, 2.221, 0])

# transform vector between the two frames
T = calibration_point_pose_base - calibration_point_pose_finger

init_pose_col_1_base = press_finger(np.array([0.0090, 0.00196, 0.08542, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_col_2_base = press_finger(np.array([-0.009, 0.00196, 0.08542, 1.782, 0, 0]), small_finger, -0.00296, False) + T

init_pose_row_1_base = press_finger(np.array([0.0060, 0.00225, 0.08601, 1.571, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_2_base = press_finger(np.array([0.0040, 0.00306, 0.08021, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_3_base = press_finger(np.array([0.0040, 0.00410, 0.07532, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_4_base = press_finger(np.array([0.0060, 0.00514, 0.07043, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_5_base = press_finger(np.array([0.0060, 0.00618, 0.06554, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_6_base = press_finger(np.array([0.0060, 0.00722, 0.06065, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_7_base = press_finger(np.array([0.0080, 0.00833, 0.05542, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_8_base = press_finger(np.array([0.0080, 0.00943, 0.05025, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_9_base = press_finger(np.array([0.0080, 0.01047, 0.04536, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_10_base = press_finger(np.array([0.0080, 0.01151, 0.04047, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_11_base = press_finger(np.array([0.0100, 0.01255, 0.03558, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_12_base = press_finger(np.array([0.0100, 0.01364, 0.03042, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_13_base = press_finger(np.array([0.0100, 0.01458, 0.02601, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_14_base = press_finger(np.array([0.0100, 0.01562, 0.02112, 1.782, 0, 0]), small_finger, -0.00296, False) + T
init_pose_row_15_base = press_finger(np.array([0.0120, 0.01666, 0.01623, 1.782, 0, 0]), small_finger, -0.00296, False) + T

robot = dc.UR10eController('./DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')

cam = cv2.VideoCapture(2)
FTSensor = FTReading.FTReading("192.168.1.1")

FTSensor.InitFT()

pose = []
FT_data = []
PressInfo = []

col_1_point = 15
col_2_point = 15
row_1_point = 4
row_2_point = 3
row_3_point = 3
row_4_point = 4
row_5_point = 4
row_6_point = 4
row_7_point = 5
row_8_point = 5
row_9_point = 5
row_10_point = 5
row_11_point = 6
row_12_point = 6
row_13_point = 6
row_14_point = 6
row_15_point = 7

col_1_index = range(0, col_1_point)
col_2_index = range(col_1_index.stop, col_1_index.stop + col_2_point)
row_1_index = range(col_2_index.stop, col_2_index.stop + row_1_point)
row_2_index = range(row_1_index.stop, row_1_index.stop + row_2_point)
row_3_index = range(row_2_index.stop, row_2_index.stop + row_3_point)
row_4_index = range(row_3_index.stop, row_3_index.stop + row_4_point)
row_5_index = range(row_4_index.stop, row_4_index.stop + row_5_point)
row_6_index = range(row_5_index.stop, row_5_index.stop + row_6_point)
row_7_index = range(row_6_index.stop, row_6_index.stop + row_7_point)
row_8_index = range(row_7_index.stop, row_7_index.stop + row_8_point)
row_9_index = range(row_8_index.stop, row_8_index.stop + row_9_point)
row_10_index = range(row_9_index.stop, row_9_index.stop + row_10_point)
row_11_index = range(row_10_index.stop, row_10_index.stop + row_11_point)
row_12_index = range(row_11_index.stop, row_11_index.stop + row_12_point)
row_13_index = range(row_12_index.stop, row_12_index.stop + row_13_point)
row_14_index = range(row_13_index.stop, row_13_index.stop + row_14_point)
row_15_index = range(row_14_index.stop, row_14_index.stop + row_15_point)


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


for d in depth_list:
    # column 1
    print('move to column 1')
    for i in col_1_index:
        if i == col_1_index[0]:
            pose.append(init_pose_col_1_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='vertical_col_1',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # column 2
    print('move to column 2')
    for i in col_2_index:
        if i == col_2_index[0]:
            pose.append(init_pose_col_2_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='vertical_col_2',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 1
    print('move to row 1')
    for i in row_1_index:
        if i == row_1_index[0]:
            pose.append(init_pose_row_1_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 2
    print('move to row 2')
    for i in row_2_index:
        if i == row_2_index[0]:
            pose.append(init_pose_row_2_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 3
    print('move to row 3')
    for i in row_3_index:
        if i == row_3_index[0]:
            pose.append(init_pose_row_3_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 4
    print('move to row 4')
    for i in row_4_index:
        if i == row_4_index[0]:
            pose.append(init_pose_row_4_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 5
    print('move to row 5')
    for i in row_5_index:
        if i == row_5_index[0]:
            pose.append(init_pose_row_5_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 6
    print('move to row 6')
    for i in row_6_index:
        if i == row_6_index[0]:
            pose.append(init_pose_row_6_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 7
    print('move to row 7')
    for i in row_7_index:
        if i == row_7_index[0]:
            pose.append(init_pose_row_7_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 8
    print('move to row 8')
    for i in row_8_index:
        if i == row_8_index[0]:
            pose.append(init_pose_row_8_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 9
    print('move to row 9')
    for i in row_9_index:
        if i == row_9_index[0]:
            pose.append(init_pose_row_9_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 10
    print('move to row 10')
    for i in row_10_index:
        if i == row_10_index[0]:
            pose.append(init_pose_row_10_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 11
    print('move to row 11')
    for i in row_11_index:
        if i == row_11_index[0]:
            pose.append(init_pose_row_11_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 12
    print('move to row 12')
    for i in row_12_index:
        if i == row_12_index[0]:
            pose.append(init_pose_row_12_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 13
    print('move to row 13')
    for i in row_13_index:
        if i == row_13_index[0]:
            pose.append(init_pose_row_13_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 14
    print('move to row 14')
    for i in row_14_index:
        if i == row_14_index[0]:
            pose.append(init_pose_row_5_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

    # row 15
    print('move to row 15')
    for i in row_15_index:
        if i == row_15_index[0]:
            pose.append(init_pose_row_15_base)
            robot.move_p(pose[i])
        else:
            pose.append(move_to_next_point(init_pose_=pose[i - 1],
                                           finger=small_finger,
                                           move_direction='horizontal',
                                           stride=S,
                                           move=True))
        press_finger(pose[i], small_finger, d, move=True)
        # read from camera
        ret, picture = cam.read()
        cv2.imwrite('./Data/Diameter_{Diameter}mm_depth_{depth}mm_{index}.jpg'.format(Diameter=D * 1000, depth=d * 1000, index=i), picture)
        # read from FT sensor
        force_data = FTSensor.GetReading(1000)
        FT_data.append(force_data)
        pose_finger = pose[i] - T
        robot.move_p(pose[i])

np.save('./Data/pose.npy', pose)
