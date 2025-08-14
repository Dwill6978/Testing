import logging
import time
import csv
import pygame
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#UINT16_MAX
max = 65535
tolerance = 0.03
logging.basicConfig(level=logging.ERROR)



# Data storage for live plotting
roll_data = deque(maxlen=100)  # Store the last 100 roll values
pitch_data = deque(maxlen=100)  # Store the last 100 pitch values
yaw_data = deque(maxlen=100)  # Store the last 100 yaw values
accx_data = deque(maxlen=100)  # Store the last 100 accelerometer x values
timestamps = deque(maxlen=100)  # Store the last 100 timestamps

# Initialize the plot
fig, (ax, ax2) = plt.subplots(1,2,sharex=True)

line_accx, = ax2.plot([], [], label="Acc X (Gs)", color="orange")
ax2.set_ylim(-5, 5)  # Adjust y-axis range based on expected values
ax2.set_title("Live Accelerometer Data")
ax2.set_xlabel("Time (relative)")
ax2.set_ylabel("Acceleration (Gs)")
ax2.legend()

line_roll, = ax.plot([], [], label="Roll (degrees)", color="blue")
line_pitch, = ax.plot([], [], label="Pitch (degrees)", color="green")
line_yaw, = ax.plot([], [], label="Yaw (degrees)", color="red")
#ax.set_xlim(0, 100)  # Fixed x-axis range for 100 data points
ax.set_ylim(-50, 50)  # Adjust y-axis range based on expected values
ax.set_title("Live Stabilizer Data")
ax.set_xlabel("Time (relative)")
ax.set_ylabel("Angle (degrees)")
ax.legend()



def setMotor(axis,num):
    num  = str(num)
    param = "motorPowerSet.m" + num
    statement = "Setting Motor M" + num + " to "
    setValue = abs(axis*(max/2))

    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value(param, max/2)
        print(statement + str(max/2))
        time.sleep(0.1)
    elif axis < -tolerance:
        cf.param.set_value(param, (max/2)-int(setValue))
        print(statement + str(int((max/2)-setValue)))
        time.sleep(0.1)
    elif axis > tolerance:
        cf.param.set_value(param, (max/2)+int(setValue))
        print(statement + str(int((max/2)+setValue)))
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
                for i in range(lastVal,setValue,1000):
                    cf.param.set_value('servo.servoAngle', i)
                    print("Setting BL Motor to " + str(i))
                    time.sleep(0.1)
            elif lastVal-0.03 > setValue: 
                for i in range(lastVal,setValue,-1000):
                    cf.param.set_value('servo.servoAngle', i)
                    print("Setting BL Motor to " + str(i))
                    time.sleep(0.1)
            else:
                cf.param.set_value('servo.servoAngle', lastVal)
                print("Setting BL Motor to " + str(lastVal))
                time.sleep(0.1)
    return setValue

def log_callback(timestamp, data, logconf):
    """
    Callback function to handle log data received from the Crazyflie.
    This function will be called whenever new log data is received.
    """
    roll = data['stabilizer.roll']
    pitch = data['stabilizer.pitch']
    yaw = data['stabilizer.yaw']
    accx = data['acc.x']
    timestamp = timestamp / 1000.0  # Convert to seconds for better readability

    # Append data to the deque for live graphing
    roll_data.append(roll)
    pitch_data.append(pitch)
    yaw_data.append(yaw)
    accx_data.append(accx)
    timestamps.append(timestamp)  # Use relative time for simplicity


def log_async(cf,logconf):
     
     cf.log.add_config(logconf)
     logconf.data_received_cb.add_callback(log_callback)

def update_plot(frame):
    """
    Update function for the live plot.
    """
    line_roll.set_data(timestamps, roll_data)
    line_pitch.set_data(timestamps, pitch_data)
    line_yaw.set_data(timestamps, yaw_data)
    line_accx.set_data(timestamps, accx_data)
    ax.relim()  # Recalculate limits based on new data
    ax.autoscale_view()  # Autoscale the view to fit the new data
    plt.pause(0.01)  # Pause to allow the plot to update
    return line_roll, line_pitch, line_yaw, line_accx

def main():

    cflib.crtp.init_drivers()


    global cf


    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('acc.x', 'float')




    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('motorPowerSet.enable', '1')
        log_async(cf, lg_stab)  
        print("Connected to Crazyflie with URI:", uri)
        lg_stab.start()
        # Start the live graphing
        anim = animation.FuncAnimation(fig, update_plot, interval=100)  # Update every 100ms
        plt.tight_layout()
        plt.show()  # Non-blocking to allow Crazyflie communication
        try:
            while plt.fignum_exists(fig.number):  # Keep running while the plot is open
                    time.sleep(0.1)
        except KeyboardInterrupt:
                print("Interrupted by user.")
        finally:
                lg_stab.stop()
        
if __name__ == '__main__':
    main()