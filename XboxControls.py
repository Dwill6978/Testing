import signal
import time
from xbox360controller import Xbox360Controller
import matplotlib.pyplot as plt
import numpy as np
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#UINT16_MAX
max = 65535

'''
#m2 and m4 connectors 
def setServo1(value):
   
    print("Setting Servo to " + str(value))
    #cf.param.set_value('motorPowerSet.m4', int(value))
   

def setServo2(value):
    
    print("Setting Servo to " + str(value))
    #cf.param.set_value('motorPowerSet.m2', int(value))
'''

def trialsetMotor(axis):
    motorSet = 0
    if axis >= 0:
        print("Setting Motor to " + str(0))
        cf.param.set_value('motorPowerSet.m3', 0)
        time.sleep(0.1)
    else:
        #--rounding the value 
        print("Setting Motor to " + str(round(axis*-65000)))
        time.sleep(0.1)
        cf.param.set_value('motorPowerSet.m3', int(round(axis*-max)))
        motorSet = round(axis*-65000)
    return(motorSet)

# -- half interval algorithm to calculate steps
# -- slower for larger values; faster for smaller values
def uniformStepsCounter(oldValue,newValue):
    error = 0.01 #--the smallest value for step
    count = 0
    while abs(newValue)-abs(oldValue)>error:
        count+=1
        delta = abs(abs(newValue)-abs(oldValue))/2
        oldValue+=delta 
    #print('newValue',newValue)
    return count


# -- function for stepping 
def stepRun(oldValue,newValue):
    #print("\noldValue",oldValue)
    #print("newValue",newValue,'\n')
    smallValue = 0.00001 # --should be greater than epsillon

    yValues = [0]

    if newValue<0:
        epsillon = uniformStepsCounter(oldValue,newValue)
        #print("epsillon",epsillon,'\n')

        if epsillon>0:
            delta = abs(abs(newValue)-abs(oldValue))/epsillon
            if ((abs(delta)-0)>smallValue and (abs(newValue)-0)>smallValue):
            #--change so doesn't run when value is negetive

            #print("delta",delta)
            #print("left_x",newValue)

            #print(abs(delta)<abs(newValue))

                while abs(delta)<abs(newValue):

                    #print(delta)

                    if newValue<0:
                        motorSet = trialsetMotor(-delta)
                    else:
                        motorSet = trialsetMotor(delta)
                    delta+=delta
                    yValues.append(motorSet)
                    clock2.append(time.perf_counter())
                    # -- pauses before iteration
                   # time.sleep(0.1)
            # -- doesn't reset to 0 after change in deflection
            motorSet = trialsetMotor(newValue)
            yValues.append(motorSet)
            clock2.append(time.perf_counter())
    return yValues, clock2

x = []
y = []
clock = []
clock2 = []
start = time.perf_counter()
clock.append(time.perf_counter())
    # Initialize the low-level drivers
cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf
    cf.param.set_value('motorPowerSet.enable', '1')
    with Xbox360Controller(1, axis_threshold=0.2) as controller:
        # --changed index to default 0
        try:

            # Left and right axis move event
            # controller.axis_l.when_moved = on_axis_moved
            # controller.axis_r.when_moved = on_axis_moved
            
            prevleft_x = 0

            # --prevleft_x = prevleft_y = prevright_x = prevright_y = 0 

            while True:
                left = controller.axis_l
                right = controller.axis_r
                left_x = round(left.x,2)
                left_y = round(left.y,2)
                right_x = round(right.x,2)
                right_y = round(right.y,2)  
                #print("Left: ({0},{1})".format(left_x,left_y), "Right: ({0},{1})".format(right_x,right_y))
                print("Prev Left: {0}".format(prevleft_x))
                yValues, clock2 = stepRun(prevleft_x,left_x)
                y.extend(yValues)
                if left_x<0:
                    prevleft_x=left_x
                else:
                    prevleft_x=0.0
                    
                
                x.append(left_x)
                clock.append(time.perf_counter())

                #print(x)
                #print(y)
                #print(clock2)

                #trialsetMotor(left_y)
                #trialsetMotor(right_x)
                #trialsetMotor(right_y)
                time.sleep(0.1)

            signal.pause()
        except KeyboardInterrupt:
            cf.param.set_value('motorPowerSet.enable', '0')
            print("Scionera Suckers")
        pass

figure, (ax1, ax2) = plt.subplots(2,1)
x.append(0)
ax1.plot(clock,x)
ax1.set_title("input")

setValue=len(clock)
while setValue<len(y):
    clock.append(0)
    setValue+=1

ax2.plot(clock,y)
ax2.set_title("output")
plt.show()