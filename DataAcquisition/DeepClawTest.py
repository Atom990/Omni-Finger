import DeepClawBenchmark.deepclaw.driver.arms.UR10eController as Controller
import numpy as np
import DataAcquisition.Finger as Finger

if __name__ == '__main__':
    big_finger = Finger.Finger(0, 0, [1, 2, 3], [4, 5], 0, 0)
    print(big_finger)
