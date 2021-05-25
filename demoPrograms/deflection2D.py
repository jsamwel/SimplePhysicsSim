
import numpy as np
import matplotlib.pyplot as plt

pos = np.array([[.1,1],[.5,-.5]])
mX = .5

# Calculate original function
a =  (pos[0][1] - pos[1][1]) / (pos[0][0] - pos[1][0])
b = pos[0][1] - a * pos[0][0]

# Calculate b of the inverse function
b2 = mX - (-a) * ((b-mX)/-a)

x = np.array(range(2))
y = a * x + b
y2 = -a * x + b2
yMirror = np.full(2,mX)

pos3=np.array([pos[1][0], -a * pos[1][0] + b2])

plotDataX = [pos[0][0], pos[1][0], pos3[0]]
plotDataY = [pos[0][1], pos[1][1], pos3[1]]
plt.plot(plotDataX, plotDataY, 'ro')
plt.plot(x,y)
plt.plot(x,y2)
plt.plot(x, yMirror)
plt.show()