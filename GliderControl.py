import logging
import time
import csv
import pygame
import argparse
import numpy as np
from scipy.optimize import curve_fit
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import array as arr

# Constants
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
max = 65535
tolerance = 0.05

#Tail Deflection Data
global m2_pwm
m2_pwm = np.array([0, 15000, 23000 ,25000 ,32767 ,45000, 50000 ,55000 ,65535])
global m2_angle
m2_angle = np.array([-40, -12, -3, 0, 4 , 12, 14, 15, 20])

global m3_pwm
m3_pwm = np.array([0, 15000, 32767, 40000, 45000, 55000, 63000])
global m3_angle
m3_angle = np.array([-40, -20, -7, 0, 9, 21, 40])

# Configure logging
logging.basicConfig(level=logging.ERROR)

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--debug', action='store_true', help='Enable debug output')
args = parser.parse_args()


# Data storage for live plotting
gyrox_data = deque(maxlen=100)  # Store the last 100 Gyro X values
gyroy_data = deque(maxlen=100)  # Store the last 100 Gyro Y values
gyroz_data = deque(maxlen=100)  # Store the last 100 Gyro Z values
accx_data = deque(maxlen=100)  # Store the last 100 Accelerometer X values
accy_data = deque(maxlen=100)  # Store the last 100 Accelerometer Y values
accz_data = deque(maxlen=100)  # Store the last 100 Accelerometer Z values
motor2_data = deque(maxlen=100)  # Store the last 100 Motor M2 values
motor3_data = deque(maxlen=100)  # Store the last 100 Motor M3 values
pm_data = deque(maxlen=100)  # Store the last 100 PM values
timestamps = deque(maxlen=100)  # Store the last 100 timestamps
timestamps2 = deque(maxlen=100)  # Store the last 100 timestamps for motor data

# Initialize the plot
fig, ((ax, ax2), (ax3, ax4)) = plt.subplots(2,2,sharex=True)

line_gyrox, = ax.plot([], [], label="X Angular Rate (degrees/s)", color="blue")
line_gyroy, = ax.plot([], [], label="Y Angular Rate (degrees/s)", color="green")
line_gyroz, = ax.plot([], [], label="Z Angular Rate (degrees/s)", color="red")
ax.set_ylim(-90, 90)  # Adjust y-axis range based on expected values
ax.set_title("Live Gyro Data")
ax.set_xlabel("Time (seconds)")
ax.set_ylabel("Angular Rate (degrees/s)")
ax.legend()

line_accx, = ax2.plot([], [], label="Accelerometer X (Gs)", color="orange")
line_accy, = ax2.plot([], [], label="Accelerometer Y (Gs)", color="purple")
line_accz, = ax2.plot([], [], label="Accelerometer Z (Gs)", color="brown")
ax2.set_ylim(-5, 5)  # Adjust y-axis range based on expected values
ax2.set_title("Live Accelerometer Data")
ax2.set_xlabel("Time (seconds)")
ax2.set_ylabel("Acceleration (Gs)")
ax2.legend()

line_motor2, = ax3.plot([], [], label="Motor M2 Power", color="cyan")
line_motor3, = ax3.plot([], [], label="Motor M3 Power", color="magenta")
ax3.set_ylim(-45,45)  # Adjust y-axis range based on expected values
ax3.set_title("Live Motor Power Data")
ax3.set_xlabel("Time (seconds)")
ax3.set_ylabel("CS Deflection Angle (degrees)")
ax3.legend()

line_pm, = ax4.plot([], [], label="Battery Level", color="lime")
ax4.set_ylim(0, 100)  # Adjust y-axis range based on expected values
ax4.set_title("Live Battery Level")
ax4.set_xlabel("Time (seconds)")
ax4.set_ylabel("Battery Level (%)")
ax4.legend()

def quadratic(x,a,b,c):
    """
    Quadratic function for curve fitting.
    """
    return a*x**2 + b*x + c

def round_to_nearest(value, step):
    """
    Round a value to the nearest multiple of step.
    """
    value = round(value / step) * step
    if value > max:
        value = max
    return value

def setMotor(axis,num):
    num  = str(num)
    param = "motorPowerSet.m" + num
    statement = "Setting Motor M" + num + " to "
    setValue = abs(axis*(max/2))

    if num == 3 and round_to_nearest((max/2)+int(setValue), 1000) > 50000:
        setValue = 50000
        if args.debug:
            print("Motor M3 value capped at 50000")

    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value(param, max/2)
        if args.debug:
            print(statement + str(max/2))
        time.sleep(0.01)
    elif axis < -tolerance:
        cf.param.set_value(param, round_to_nearest((max/2)-int(setValue), 1000))
        if args.debug:
            print(statement + str(round_to_nearest((max/2)-int(setValue), 1000)))
        time.sleep(0.01)
    elif axis > tolerance:
        cf.param.set_value(param, round_to_nearest((max/2)+int(setValue),1000))
        if args.debug:
            print(statement + str(round_to_nearest((max/2)+int(setValue),1000)))
        time.sleep(0.01)


def setBLMotor(axis,lastVal):
    setValue = round((abs(axis)*65535)/10000)*10000
    if setValue > max:
        setValue = max
    if axis > -0.05 and axis < 0.05:
        cf.param.set_value('servo.servoAngle', 0)
        if args.debug:
            print("Setting BL Motor to 0")
        time.sleep(0.1)
    else:
            if setValue > lastVal+0.03 :
                for i in range(lastVal,setValue,1000):
                    cf.param.set_value('servo.servoAngle', i)
                    if args.debug:
                        print("Setting BL Motor to " + str(i))
                    time.sleep(0.1)
            elif lastVal-0.03 > setValue: 
                for i in range(lastVal,setValue,-1000):
                    cf.param.set_value('servo.servoAngle', i)
                    if args.debug:
                        print("Setting BL Motor to " + str(i))
                    time.sleep(0.1)
            else:
                cf.param.set_value('servo.servoAngle', lastVal)
                if args.debug:
                    print("Setting BL Motor to " + str(lastVal))
                time.sleep(0.1)
    return setValue


def update_plot(frame):
    """
    Update function for the live plot.
    """
    line_gyrox.set_data(timestamps, gyrox_data)
    line_gyroy.set_data(timestamps, gyroy_data)
    line_gyroz.set_data(timestamps, gyroz_data)
    line_accx.set_data(timestamps, accx_data)
    line_accy.set_data(timestamps, accy_data)
    line_accz.set_data(timestamps, accz_data)
    line_motor2.set_data(timestamps2, motor2_data)
    line_motor3.set_data(timestamps2, motor3_data)
    line_pm.set_data(timestamps2, pm_data)
    ax.relim()  # Recalculate limits based on new data
    ax.autoscale_view()  # Autoscale the view to fit the new data
    
    return line_gyrox, gyroy_data, gyroz_data, line_accx, line_accy, line_accz, line_motor2, line_motor3, line_pm
     

def main():
    global cf

    #Calculate PWM to Angle Curve Fit
    params2, covariance = curve_fit(quadratic, m2_pwm, m2_angle)
    a2, b2, c2 = params2

    params3, covariance3 = curve_fit(quadratic, m3_pwm, m3_angle)
    a3, b3, c3 = params3


    filename = input("Enter the filename to save data (without extension): ")
    filename = filename + ".csv"
    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['Timestamp', 'Gyro X', 'Gyro Y', 'Gyro Z', 'Acc X', 'Acc Y', 'Acc Z', 'Motor M2', 'Motor M3', 'Battery Level'])

        def log_callback(timestamp, data, logconf):
            """
            Callback function to handle log data received from the Crazyflie.
            This function will be called whenever new log data is received.
            """
            gyrox = data['gyro.x']
            gyroy = data['gyro.y']
            gyroz = data['gyro.z']
            accx = data['acc.x']
            accy = data['acc.y']
            accz = data['acc.z']
            timestamp = timestamp / 1000.0  # Convert to seconds for better readability

            # Append data to the deque for live graphing
            gyrox_data.append(gyrox)
            gyroy_data.append(gyroy)
            gyroz_data.append(gyroz)
            accx_data.append(accx)
            accy_data.append(accy)
            accz_data.append(accz)
            timestamps.append(timestamp)  
            # Write data to CSV file
            csv_writer.writerow([timestamp, gyrox, gyroy, gyroz, accx, accy, accz,'','',''])  # Empty placeholders for motor data

        def log_callback2(timestamp, data, logconf):
            """
            Callback function to handle log data received from the Crazyflie.
            This function will be called whenever new log data is received.
            """

            motor_m2 = float(data['motor.m2'])
            motor_m3 = float(data['motor.m3'])
            battery_level = float(data['pm.batteryLevel'])  # Battery Charge Level
            timestamp = timestamp / 1000.0  # Convert to seconds for better readability
            motor_m2 = quadratic(motor_m2, a2, b2, c2)#Convert to Angle of Deflection
            motor_m3 = quadratic(motor_m3, a3, b3, c3)#Convert to Angle of Deflection
            # Append data to the deque for live graphing
            motor2_data.append(motor_m2)
            motor3_data.append(motor_m3)
            pm_data.append(battery_level)
            timestamps2.append(timestamp)  
            # Write data to CSV file
            csv_writer.writerow(['','','','','','','',motor_m2, motor_m3, battery_level])  # Empty placeholders for gyro and acc data

        def log_async(cf,logconf):
            
            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback)

        def log_async_motor(cf,logconf):
            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback2)


        cflib.crtp.init_drivers()
        pygame.init()
        pygame.joystick.init()

        joystick = pygame.joystick.Joystick(0)
        joystick.init()



        lg_stab = LogConfig(name='Attitude', period_in_ms=10)
        lg_stab.add_variable('gyro.x', 'float')
        lg_stab.add_variable('gyro.y', 'float')
        lg_stab.add_variable('gyro.z', 'float')
        lg_stab.add_variable('acc.x', 'float')
        lg_stab.add_variable('acc.y', 'float')
        lg_stab.add_variable('acc.z', 'float')

        lg_motor = LogConfig(name='Motor', period_in_ms=10)
        lg_motor.add_variable('motor.m2', 'uint16_t')
        lg_motor.add_variable('motor.m3', 'uint16_t')
        lg_motor.add_variable('pm.batteryLevel', 'uint8_t')  # Battery Charge Level


        
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

                cf = scf.cf
                cf.param.set_value('motorPowerSet.enable', '1')
                log_async(cf, lg_stab)  
                log_async_motor(cf, lg_motor)
                print("Connected to Crazyflie with URI:", uri)

                setMotor(0,2)
                setMotor(0,3)
                lastVal = 0
                lg_stab.start()
                lg_motor.start()

                trimmed = False

                # Update the live plot    
                anim = animation.FuncAnimation(fig, update_plot, interval=1,cache_frame_data=False)  # Update every 1ms
                plt.tight_layout()  # Adjust layout to prevent overlap
                plt.show(block=False)  # Non-blocking mode for live plotting

                try:
                        while True:
                            plt.pause(0.01)  # Allow the plot to update

                            pygame.event.pump()

                            x = joystick.get_axis(0) #Roll
                            x = round(x, 2)

                            y = joystick.get_axis(1) #Pitch
                            y = round(y, 2)

                            twist = joystick.get_axis(2) #Yaw
                            twist = round(twist, 2)

                            throttle = joystick.get_axis(3) #Throttle   
                            throttle = round(throttle, 2)

                            if joystick.get_button(0) == 1:  # Button A
                                trimmed = True
                            if joystick.get_button(1) == 1:  # Button B
                                trimmed = False

                            if not trimmed:
                                setMotor(x,3)
                                setMotor(y,2)
                        # lastVal = setBLMotor(throttle,lastVal)
                            
                            
                            
                except KeyboardInterrupt:
                        print("Exiting...")
                        cf.param.set_value('motorPowerSet.enable', '0')
                        time.sleep(0.1)
                        pygame.quit()
                        plt.close(fig)
                        lg_motor.stop()
                        lg_stab.stop()
                        time.sleep(0.1)
                        csvfile.close()
                        pass
        
if __name__ == '__main__':
    main()