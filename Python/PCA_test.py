from stagger.parameters import PCA
import numpy as np
from random import randint, random
import matplotlib.pyplot as plt

var = 20
points: list[list[int]] = [[randint(-var, var)*random(), randint(-var, var)*random()] for _ in range(200)]
curve = np.array(points)
data = PCA(curve)
print(data)
x = curve[:,0]
y = curve[:,1]
plt.plot(x[:], y[:], '.r')
plt.plot(data[0][:], data[2][:])
plt.plot(data[0][:], data[3][:])
plt.show()