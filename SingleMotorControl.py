import logging
import time
import signal
import numpy as np
from xbox360controller import Xbox360Controller
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#UINT16_MAX
max = 65535


def setMotor1(axis):
    setValue = abs(axis*(max/2))
    if axis > 0:
        cf.param.set_value('motorPowerSet.m1', int(setValue))
        print("Setting Motor M1 to " + str(int(setValue)))
        time.sleep(0.1)
    else:
        cf.param.set_value('motorPowerSet.m1', int(setValue+(max/2)))
        print("Setting Motor M1 to " + str(int(setValue+(max/2))))
        time.sleep(0.1)

def setMotor2(axis):
    setValue = abs(axis*(max/2))
    if axis > 0:
        cf.param.set_value('motorPowerSet.m2', int(setValue))
        print("Setting Motor M2 to " + str(int(setValue)))
        time.sleep(0.1)
    else:
        cf.param.set_value('motorPowerSet.m2', int(setValue+(max/2)))
        print("Setting Motor M2 to " + str(int(setValue+(max/2))))
        time.sleep(0.1)

def setMotor3(axis):
    setValue = abs(axis*(max/2))
    if axis > 0:
        cf.param.set_value('motorPowerSet.m3', int(setValue))
        print("Setting Motor M3 to " + str(int(setValue)))
        time.sleep(0.1)
    else:
        cf.param.set_value('motorPowerSet.m3', int(setValue+(max/2)))
        print("Setting Motor M3 to " + str(int(setValue+(max/2))))
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

def setBLMotor(axis):
    cf.param.set_value('servo.servoAngle', int(abs(axis)*50000))#Need to change to max but setting limit low for testing
    print("Setting BL Motor to " + str(abs(axis)*50000)) 
    
    


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('motorPowerSet.enable', '1')
        setMotor1(0)
        setMotor2(0)
        with Xbox360Controller(1, axis_threshold=0.2) as controller:
            try:
                while True:
                    left = controller.axis_l
                    right = controller.axis_r
                    trigger = controller.trigger_l
                    trigger_l = round(trigger._value,2)
                    left_x = round(left.x,2)
                    left_y = round(left.y,2)
                    right_x = round(right.x,2)
                    right_y = round(right.y,2) 
                    #setMotor1(right_y)
                    setMotor2(right_y)
                    #setMotor3(left_y)
                    setMotor1(right_x)
                    setBLMotor(left_y)
                    
                    
            except KeyboardInterrupt:
                print("Exiting...")
                cf.param.set_value('motorPowerSet.enable', '0')
                time.sleep(0.1)
                pass