import numpy as np

L = []
a = np.array([0.470, -0.560, 0.133, 102 * np.pi / 180, 0, 0])
b = np.array([1, 1, 1, 1, 1, 1])
c = np.array([2, 2, 2, 2, 2, 2])
L.append(a)
L.append(b)
L.append(c)
test = np.array(L)
print(test)
np.save("test_data.npy", test)
read_data = np.load('test_data.npy')
print(read_data)
