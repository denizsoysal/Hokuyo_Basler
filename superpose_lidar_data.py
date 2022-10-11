import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import cv2
import matplotlib.pyplot as plt

import time
import os 

folder_name ="inside_wall_dry\RecordingDuringSpraying_14_41_5"


f = open(folder_name+"/log_0.txt", 'r')
ang = f.readline()
ang = ang.strip('][\n').split(', ')
ang_float = [float(ele)*3.14/180 for ele in ang]


dist = f.read()
dist = dist.strip('][\n').split(' ')
last_elem = dist[-1].split(']\n')
dist[-1] = last_elem[0]
time_stamp = last_elem[1]
#NEED TO FORMAT the timestamp !!!
dist_float = [float(ele.strip('\n')) for ele in dist]
dist_filter = dist_float
for i in range(len(dist_float)):
    if (dist_float[i]>3):
        dist_filter[i] = 0

#Configuration of the display window for Lidar data in polar coordinates
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
line1, = ax.plot(ang_float, dist_filter, 'o', color='black',  markersize=2)        # so that we can update data later
ax.set_rmax(3)
ax.set_rticks([0.5, 1, 1.5, 2, 2.5, 3])  # Less radial ticks
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)

"""
# display Lidar data in the window
cv2.namedWindow("Lidar",cv2.WINDOW_NORMAL);
cv2.resizeWindow('Lidar', 720, 480)
"""


for i in range(300):

    #reading Lidar logs
    f = open(folder_name+"/log_%d.txt" % i, 'r')
    ang = f.readline()
    
    dist = f.read()
    dist = dist.strip('][\n').split(' ')
    last_elem = dist[-1].split(']\n')
    dist[-1] = last_elem[0]
    time_stamp = last_elem[1]
    #NEED TO FORMAT the timestamp !!!
    dist_float = [float(ele.strip('\n')) for ele in dist]
    dist_filter = dist_float
    for j in range(len(dist_float)):
        if (dist_float[j]>3):
            dist_filter[j] =0
    
    now=time.localtime()
    time_str = "t= {}:{}:{}".format(now.tm_hour,now.tm_min,now.tm_sec)
    ax.set_title("Superposition of the measurements of the LiDAR during spraying")
    ax.plot(ang_float, dist_filter, 'o', color='black',  markersize=2)        # so that we can update data later

    
plt.show()