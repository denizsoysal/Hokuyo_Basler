import serial

from Hokuyo_Basler.lidar.lib.src.hokuyo.driver import hokuyo
from Hokuyo_Basler.lidar.lib.src.hokuyo.tools import serial_port
from threading import Thread
import time


uart_port = 'COM3'
uart_speed = 115200


number_of_measurement = 10
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)
print(laser.laser_on())
for i in range(0,number_of_measurement):

    ang,dist,timestamp = laser.get_scan()
    #print(ang,dist,timestamp)
    time.sleep(1)
#Thread(target = func2).start()

print('---***************************************************************-------------')

print('---')
print(laser.reset())
print('---')
print(laser.laser_off())
print("THE END")


