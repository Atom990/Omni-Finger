import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as Controller
import numpy as np
import DataAcquisition.Finger as Finger


def press_finger(init_pose_, finger_, depth):
    next_pose_ = init_pose_ + np.array([0, -depth * np.sin(finger_.theta_1), -depth * np.cos(finger_.theta_1), 0, 0, 0])
    robot.move_p(next_pose_, 0.1, 0.5)
    return next_pose_


def move_to_next_point(init_pose_, move_direction, stride):
    if move_direction == 'horizontal':
        next_pose_ = init_pose_ + np.array([-stride, 0, 0, 0, 0, 0])
        robot.move_p(next_pose_, 0.1, 0.5)
    elif move_direction == 'vertical':
        next_pose_ = init_pose_ + np.array([-stride * np.cos(finger.beta), -stride * np.cos(finger.alpha),
                                            stride * np.sin(finger.alpha) * np.sin(finger.theta_2), 0, 0, 0])
        robot.move_p(next_pose_, 0.1, 0.5)
    else:
        next_pose_ = init_pose_
        print('Wrong parameters!')
    return next_pose_


if __name__ == '__main__':
    finger = Finger.Finger(0, 0, [0.03042, 0.05542, 0.08601], [78 * np.pi / 180, 84.29 * np.pi / 180],
                           78.06 * np.pi / 180, 84.41 * np.pi / 180)
    robot = Controller.UR10eController('../DeepClawBenchmark/configs/robcell-ur10e-hande-d435/ur10e.yaml')
    # state = robot.get_state()
    # print(state)
    init_pose_base = [np.array([0.600, -0.700, 0.250, -258 * np.pi / 180, 0, 0])]
    robot.move_p(init_pose_base[0], 0.1, 0.5)

    for i in range(0, 5):
        press_finger(init_pose_base[i], finger, 0.03)
        robot.move_p(init_pose_base[i], 0.1, 0.5)
        init_pose_base.append(move_to_next_point(init_pose_base[i], 'horizontal', 0.005))
