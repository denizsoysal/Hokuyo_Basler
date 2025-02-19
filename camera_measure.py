import serial

from Hokuyo_Basler.lidar.lib.src.hokuyo.driver import hokuyo
from Hokuyo_Basler.lidar.lib.src.hokuyo.tools import serial_port


uart_port = 'COM3'
uart_speed = 9200

__author__ = 'paoolo'

if __name__ == '__main__':
    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)
    

    print(laser.laser_on())
    print('---')
    #print('get scan',laser.get_scan2())
    laser.get_scan()
    print('---***************************************************************-------------')
    print(laser.get_version_info())
    print('---')
    print(laser.get_sensor_specs())
    print('---')
    print(laser.get_sensor_state())
    print('---')
    print(laser.set_high_sensitive())
    print('---')
    print(laser.set_high_sensitive(False))
    print('---')
    print(laser.set_motor_speed(10))
    print('---')
    print(laser.set_motor_speed())
    print('---')
    print(laser.reset())
    print('---')
    print(laser.laser_off())
    print("THE END")