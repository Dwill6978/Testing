import logging
import time
import signal
import numpy as np
import inputs
from xbox360controller import Xbox360Controller
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


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
        cf.param.set_value('motorPowerSet.m2', max/2)
        print("Setting Motor M2 to " + str(max/2))
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
        cf.param.set_value('motorPowerSet.m3', max/2)
        print("Setting Motor M3 to " + str(max/2))
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
    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value('motorPowerSet.m4', max/2)
        print("Setting Motor M4 to " + str(max/2))
        time.sleep(0.1)
    elif axis < -tolerance:
        cf.param.set_value('motorPowerSet.m4', (max/2)+int(setValue))
        print("Setting Motor M4 to " + str(int((max/2)+setValue)))
        time.sleep(0.1)
    elif axis > tolerance:
        cf.param.set_value('motorPowerSet.m4', (max/2)-int(setValue))
        print("Setting Motor M4 to " + str(int((max/2)-setValue)))
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

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('motorPowerSet.enable', '1')
        setMotor3(0)
        setMotor2(0)
        lastVal = 0
        right_x = 0.0
        right_y = 0.0
        left_y = 0.0
        events = inputs.get_gamepad
        try:
            while True:
                if events():
                    for event in events():
                        if event.code == 'ABS_RY':
                            right_y = event.state / 32767.0
                        elif event.code == 'ABS_RX':
                            right_x = event.state / 32767.0
                        elif event.code == 'ABS_Y':
                            left_y = event.state / 32767.0
                        right_x = round(right_x,2)
                        right_y = round(right_y,2) 
                        left_y = round(left_y,2)
                        
                    setMotor2(right_x)
                    setMotor3(right_y)
                    lastVal = setBLMotor(left_y,lastVal)
                    
                    
        except KeyboardInterrupt:
                print("Exiting...")
                cf.param.set_value('motorPowerSet.enable', '0')
                time.sleep(0.1)
                pass