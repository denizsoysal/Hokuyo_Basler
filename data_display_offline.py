import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import cv2
import matplotlib.pyplot as plt

import time
import os 

folder_name = "collected_data\inside_wall_dry\RecordingDuringSpraying_14_41_5"

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
line1, = ax.plot(ang_float, dist_filter, 'ko-')        # so that we can update data later
ax.set_rmax(3)
ax.set_rticks([0.5, 1, 1.5, 2, 2.5, 3])  # Less radial ticks
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)

#put saving = 1 if you just want to save
#put saving=0 to display the data
saving = 1
i = 10
if saving==1:
    
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
    ax.set_title("Real-time Lidar data visualization, "+time_str, va='bottom')
    line1.set_ydata(dist_filter)
    # redraw the canvas
    fig.canvas.draw()
    
    # convert canvas to image
    img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
            sep='')
    img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    
    # img is rgb, convert to opencv's default bgr
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    
    
    cv2.imwrite("lidar_img_%d.png" % i, img)
else:
    for i in range(300):
        print(i)

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
        ax.set_title("Real-time Lidar data visualization, "+time_str, va='bottom')
        line1.set_ydata(dist_filter)
        # redraw the canvas
        fig.canvas.draw()
        
        # convert canvas to image
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
                sep='')
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        
        # img is rgb, convert to opencv's default bgr
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        

        
        # display Lidar data in the window
        cv2.namedWindow("Lidar",cv2.WINDOW_NORMAL);
        cv2.resizeWindow('Lidar', 720, 480)
        cv2.imshow("Lidar",img)
        
        cam_frame = cv2.imread(folder_name+"/basler_img_%d.png" % i)
        cv2.namedWindow("Basler",cv2.WINDOW_NORMAL);
        cv2.resizeWindow('Basler', 720, 480)
        cv2.imshow("Basler",cam_frame)
        
        time.sleep(0.1) #to set the rate of the loop 
        


        #escape the loop if pressed
        k = cv2.waitKey(33)
        if k == 27: #press esc to exit
            break
    cv2.destroyAllWindows()
