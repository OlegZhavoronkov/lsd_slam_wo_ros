import sys
import os
import numpy as np
import re
import matplotlib.pyplot as plt
regexp= '^.*[\[](?P<a>.*) (?P<b>.*) (?P<c>.*)[\]]'
data:np.array=np.array([])
file = open('/home/skylark/dev/src/lotos/skylark_lsd_slam/lsd_slam_wo_ros/build/tools/coords2.txt')
#with os.open('//home//skylark//dev//src//lotos//track_to.txt',flags=os.O_RDONLY) as file:

for l in file.readlines():
    line:str=l
    matches=re.match(regexp,line)
    #row=np.array()
    if matches:
        a=float(matches.group('a'))
        b=float(matches.group('b'))
        c=float(matches.group('c'))
        row=np.array([[a,b,c]])
        if np.shape(data)[0] ==0:
            data=row
        else:
            data=np.concatenate((data,row))
            #data=np.array([data[:,:],row])



ax = plt.figure().add_subplot(projection='3d')

# Plot a sin curve using the x and y axes.
x = data[:,0]
y=data[:,1]
z=data[:,2]
ax.plot3D(x,y,z)
#ax.plot(x, y, zs=0, zdir='z', label='curve in (x, y)')

# Plot scatterplot data (20 2D points per colour) on the x and z axes.
#colors = ('r', 'g', 'b', 'k')

# Fixing random state for reproducibility
#np.random.seed(19680801)

#x = np.random.sample(20 * len(colors))
#y = np.random.sample(20 * len(colors))
#c_list = []
#for c in colors:
#    c_list.extend([c] * 20)
# By using zdir='y', the y value of these points is fixed to the zs value 0
# and the (x, y) points are plotted on the x and z axes.
#ax.scatter(x, y, zs=0, zdir='y', c=c_list, label='points in (x, z)')

# Make legend, set axes limits and labels
ax.legend()
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Customize the view angle so it's easier to see that the scatter points lie
# on the plane y=0
#ax.view_init(elev=20., azim=-35, roll=0)

plt.show(block= True)




