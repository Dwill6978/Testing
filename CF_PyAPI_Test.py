import logging
import time
import signal
from xbox360controller import Xbox360Controller
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
max = 65535

def setMotor(value):
   
    print("Setting Motor to " + str(value))
    cf.param.set_value('servo.servoAngle', int(value))
    

def setServo1(value):
   
    print("Setting Servo to " + str(value))
    cf.param.set_value('motorPowerSet.m4', int(value))
   

def setServo2(value):
    
    print("Setting Servo to " + str(value))
    cf.param.set_value('motorPowerSet.m2', int(value))
    




if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('motorPowerSet.enable', '1')
        setMotor(0)
        setServo1(0)
        setServo2(0)

        print("Ramping Both Up:")
        for i in range(0, 5):
            setMotor(15000+(1666*i))
            setServo1(0+(max/5)*i)
            setServo2(0+(max/5)*i)
            time.sleep(0.2)

        time.sleep(1)
        print("Ramping Motor Down:")
        for i in range(0, 5):
            setMotor(25000-(1666*i))
            time.sleep(0.2)  
        setMotor(0)

        time.sleep(1)
        print("Ramping Servo Down:")
        for i in range(0, 5):
            setServo1(max-(max/5)*i)
            setServo2(max-(max/5)*i)
            time.sleep(0.2)
        setServo1(0)

        time.sleep(1)
        print("Differential Servo:")
        setServo2(max/2)
        setServo1(max/2)
        for i in range(0, 5):
            setServo1(max/2-(max/10)*i)
            setServo2(max/2+(max/10)*i)
            time.sleep(0.2)
        setServo1(0)
        setServo2(0)

        print("Done.")

            
        
        

        

        