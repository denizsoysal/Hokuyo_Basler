'''
A simple Program for grabing video from basler camera and converting it to opencv img.
Tested on Basler acA1300-200uc (USB3, linux 64bit , python 3.5)

'''
from pypylon import pylon
import cv2
import time

# conecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()

# converting to opencv bgr format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned


num_of_images = 10
period_of_shot = 1 #in second
while camera.IsGrabbing():
    for i in range(1,num_of_images):
        grabResult = camera.RetrieveResult(500, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            # Access the image data
            image = converter.Convert(grabResult)
            img = image.GetArray()
        stat = cv2.imwrite("saved_pypylon_img_%d.jpeg" % i,img) 
        grabResult.Release()
        time.sleep(1)
    # Releasing the resource    
    camera.StopGrabbing()

cv2.destroyAllWindows()