import signal
import time
from xbox360controller import Xbox360Controller


def on_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    print('Axis {0} moved to {1}'.format(axis.name, axis.y))

def setBLMotor(axis):
    setValue = round((abs(axis)*65535)/10000)*10000
    if axis > -0.05 and axis < 0.05:
        print("Setting BL Motor to 0")
        time.sleep(0.1)
    else:
            if setValue > lastVal:
                for i in range(lastVal,setValue,1000):
                    #cf.param.set_value('servo.servoAngle', i)
                    print("Setting BL Motor to " + str(i))
                    time.sleep(0.05)
            elif lastVal > setValue: 
                for i in range(lastVal,setValue,-1000):
                    #cf.param.set_value('servo.servoAngle', i)
                    print("Setting BL Motor to " + str(i))
                    time.sleep(0.05)
            else:
                print("Setting BL Motor to " + str(setValue))
                time.sleep(0.1)
    return setValue
    
  
    

try:
    lastVal = 0
    with Xbox360Controller(1, axis_threshold=0.2) as controller:
        # Button A events
        controller.button_a.when_pressed = on_button_pressed
        controller.button_a.when_released = on_button_released

        # Left and right axis move event
        # controller.axis_l.when_moved = on_axis_moved
        # controller.axis_r.when_moved = on_axis_moved
        
        while True:
            left = controller.axis_l
            right = controller.axis_r
            trigger = controller.trigger_l
            left_x = round(left.x,2)
            left_y = round(left.y,2)
            right_x = round(right.x,2)
            right_y = round(right.y,2)  
            lastVal = setBLMotor(left_y)
            
            #print("Left: ({0},{1})".format(left_x,left_y), "Right: ({0},{1})".format(right_x,right_y))
            #print("Trigger: {0}".format(trigger._value))
            time.sleep(0.1)

        signal.pause()
except KeyboardInterrupt:
    pass
