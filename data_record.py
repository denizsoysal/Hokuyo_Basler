import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import cv2
import matplotlib.pyplot as plt

import time
import os 

from pypylon import pylon

import serial

from lidar.lib.src.hokuyo.driver import hokuyo
from lidar.lib.src.hokuyo.tools import serial_port

uart_port = 'COM3'
uart_speed = 9200


now=time.localtime()
folder_name = "Recording_{}_{}".format(now.tm_hour,now.tm_min)
os.mkdir(folder_name) #should determine something more adequate

# conecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()

# converting to opencv bgr format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

#Initialize Lidar
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)
print(laser.laser_on())
ang,dist,timestamp = laser.get_scan()
ang = np.array(ang)
ang=ang*(3.14/180)
dist = np.array(dist)
dist = dist*0.001

#Configuration of the display window for Lidar data in polar coordinates
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
line1, = ax.plot(ang, dist, 'ko-')        # so that we can update data later
ax.set_rmax(4)
ax.set_rticks([0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4])  # Less radial ticks
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)

num_of_images = 10


while camera.IsGrabbing():
    for i in range(1000):

        print('---')
        #print('get scan',laser.get_scan2())
        ang,dist,timestamp = laser.get_scan()
        dist = np.array(dist)
        dist = dist*0.001
        #here save the lidar data 
        f = open(folder_name+"/log_%d.txt" % i, 'w')
        f.write("%s\n" %ang)
        f.write("%s\n" %dist)
        f.write("%s\n\n" %timestamp)
        f.close()
        #read the camera feed
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            # Access the image data
            image = converter.Convert(grabResult)
            frame = image.GetArray()
        #saving the image in the folder   
        cv2.imwrite(folder_name+"/img_%d.png" % i,frame)
        grabResult.Release()
        
        
        if (i%5 == 0): #we display in real time only one picture every 5 saved in memory
            now=time.localtime()
            time_str = "t= {}:{}:{}".format(now.tm_hour,now.tm_min,now.tm_sec)
            ax.set_title("Real-time Lidar data visualization, "+time_str, va='bottom')
            line1.set_ydata(dist)
            # redraw the canvas
            fig.canvas.draw()
            
            # convert canvas to image
            img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
                    sep='')
            img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        
            # img is rgb, convert to opencv's default bgr
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            
            # display Lidar data in the window
            cv2.imshow("Lidar",img)
            
            #display image from camera in window
            cv2.imshow("cam",frame)
            
        #escape the loop if pressed
        k = cv2.waitKey(33)
        if k == 27: #press esc to exit
            break
    # Releasing the resource    
    camera.StopGrabbing()
print(laser.reset())
print('---')
print(laser.laser_off())
cv2.destroyAllWindows()