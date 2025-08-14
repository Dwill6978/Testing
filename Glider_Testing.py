import logging
import time
import csv
import pygame
import signal
import numpy as np
from inputs import get_gamepad
from inputs import get_key
from xbox360controller import Xbox360Controller
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#UINT16_MAX
max = 65535
tolerance = 0.03


def setMotor1(axis):
    setValue = abs(axis*(max/2))
    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value('motorPowerSet.m1', max/2)
        print("Setting Motor M1 to " + str(max/2))
        time.sleep(0.1)
    elif axis < -tolerance:
        cf.param.set_value('motorPowerSet.m1', (max/2)-int(setValue))
        print("Setting Motor M1 to " + str(int((max/2)-setValue)))
        time.sleep(0.1)
    elif axis > tolerance:
        cf.param.set_value('motorPowerSet.m1', (max/2)+int(setValue))
        print("Setting Motor M1 to " + str(int((max/2)+setValue)))
        time.sleep(0.1)

def setMotor2(axis):
    setValue = abs(axis*(max/2))
    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value('motorPowerSet.m2', 35388)
        print("Setting Motor M2 to " + str(35388))
        time.sleep(0.1)
    elif axis < -tolerance:
        cf.param.set_value('motorPowerSet.m2', (max/2)+int(setValue))
        print("Setting Motor M2 to " + str(int((max/2)+setValue)))
        time.sleep(0.1)
    elif axis > tolerance:
        cf.param.set_value('motorPowerSet.m2', (max/2)-int(setValue))
        print("Setting Motor M2 to " + str(int((max/2)-setValue)))
        time.sleep(0.1)

def setMotor3(axis):
    setValue = abs(axis*(max/2))
    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value('motorPowerSet.m3', 32767)
        print("Setting Motor M3 to " + str(32767))
        time.sleep(0.1)
    elif axis < -tolerance:
        cf.param.set_value('motorPowerSet.m3', (max/2)+int(setValue))
        print("Setting Motor M3 to " + str(int((max/2)+setValue)))
        time.sleep(0.1)
    elif axis > tolerance:
        cf.param.set_value('motorPowerSet.m3', (max/2)-int(setValue))
        print("Setting Motor M3 to " + str(int((max/2)-setValue)))
        time.sleep(0.1)

def setMotor4(axis):
    setValue = abs(axis*(max/2))
    if axis > 0:
        cf.param.set_value('motorPowerSet.m4', int(setValue))
        print("Setting Motor M4 to " + str(int(setValue)))
        time.sleep(0.1)
    else:
        cf.param.set_value('motorPowerSet.m4', int(setValue+(max/2)))
        print("Setting Motor M4 to " + str(int(setValue+(max/2))))
        time.sleep(0.1)

def setBLMotor(axis,lastVal):
    setValue = round((abs(axis)*65535)/10000)*10000
    if setValue > max:
        setValue = max
    if axis > -0.05 and axis < 0.05:
        cf.param.set_value('servo.servoAngle', 0)
        print("Setting BL Motor to 0")
        time.sleep(0.1)
    else:
            if setValue > lastVal+0.03 :
                for i in range(lastVal,setValue,2500):
                    cf.param.set_value('servo.servoAngle', i)
                    print("Setting BL Motor to " + str(i))
                    time.sleep(0.1)
            elif lastVal-0.03 > setValue: 
                for i in range(lastVal,setValue,-2500):
                    cf.param.set_value('servo.servoAngle', i)
                    print("Setting BL Motor to " + str(i))
                    time.sleep(0.1)
            else:
                cf.param.set_value('servo.servoAngle', lastVal)
                print("Setting BL Motor to " + str(lastVal))
                time.sleep(0.1)
    return setValue



    
    
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick name: {joystick.get_name()}")
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")

    try:
                while True:
                    pygame.event.pump()
                    x = joystick.get_axis(0) #Roll
                    x = round(x, 2)
                    y = joystick.get_axis(1) #Pitch
                    y = round(y, 2)
                    twist = joystick.get_axis(2) #Yaw
                    twist = round(twist, 2)
                    throttle = joystick.get_axis(3) #Throttle   
                    throttle = round(throttle, 2)

                    buttons = []
                    for i in range(joystick.get_numbuttons()):
                        buttons.append(joystick.get_button(i))
                    print(buttons)
        

                    
                    
                    
    except KeyboardInterrupt:
                print("Exiting...")
                
                time.sleep(0.1)
                pygame.quit()
                pass
    

