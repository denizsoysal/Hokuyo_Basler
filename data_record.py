import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import cv2
import matplotlib.pyplot as plt

import time
import os 
import keyboard

from pypylon import pylon

import serial

from lidar.lib.src.hokuyo.driver import hokuyo
from lidar.lib.src.hokuyo.tools import serial_port

import pyrealsense2 as rs


uart_port = 'COM7'
uart_speed = 115200


now=time.localtime()
folder_name = "Recording_{}_{}_{}".format(now.tm_hour,now.tm_min,now.tm_sec)
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

#configure intel real sense D435i

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)


while camera.IsGrabbing():
    for i in range(1000):
        start = time.time()
        print('recording:',i)
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
        cv2.imwrite(folder_name+"/basler_img_%d.png" % i,frame)
        grabResult.Release()
        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        

 
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


        # Show images
        #save
        print("saving")
        cv2.imwrite(folder_name+"/real_sense_depth_img_%d.png" % i,depth_colormap)
        cv2.imwrite(folder_name+"/real_sense_color_img_%d.png" % i,color_image)
        cv2.waitKey(1)
    
        if (i !=0 and i%5 == 0): #we display in real time only one picture every 5 saved in memory
            
            #save raw depth without apply heat map
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            distance = depth_image.astype(float)*depth_scale 
            np.savetxt(folder_name+"/depth_values_real_sense_%d.csv" %i, distance, delimiter=",")
            
            
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
            
            #display real sense image
            cv2.namedWindow('depth real sense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('depth real sense', depth_colormap)
        
        end =  time.time()
        time_duration = end - start
        print(time_duration)
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