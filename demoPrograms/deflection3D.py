
import numpy as np
import matplotlib.pyplot as plt

herz = 60
freq = 1 / herz

s = np.array(range(3))/100

mLine = np.array([0,0,0])

pos = np.array([0.5,0.5,0.5])
vel = np.array([-100,-.5,-10])

fpos = pos + vel * s[1]

# b original function
b = pos - vel * s[0]

mpos = np.full(3,0.0)
axis2 = np.full((3,3),s)

# calculate mirrored pos
for i in range(len(fpos)):
    if fpos[i] < mLine[i]:
        b2 = mLine[i] - (-vel[i]) * ((b[i]-mLine[i])/-vel[i])
        mpos[i] = -vel[i] * s[1] + b2
        axis2[i] = b2 - vel[i] * s
    else:
        mpos[i] = pos[i] + vel[i] * s[1]
        axis2[i] = b[i] + vel[i] * s

# plot the positions
fig = plt.figure()
ax = plt.axes(projection='3d')

ax.scatter([pos[0], fpos[0]], [pos[1], fpos[1]], [pos[2], fpos[2]], marker='o')
ax.scatter(mpos[0], mpos[1], mpos[2], marker='x')

# plot lines to make the graph clearer

x = b[0] + vel[0] * s
y = b[1] + vel[1] * s
z = b[2] + vel[2] * s

ax.plot3D(x,y,z)

ax.plot3D(axis2[0],axis2[1],axis2[2])
ax.plot3D([0,0,0],[.5,.5,.5],[.3,.4,.5])

plt.show()