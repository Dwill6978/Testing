import logging
import time
import csv
import pygame
import argparse
import numpy as np
from scipy.optimize import curve_fit
import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.commander import Commander
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

# Configure logging
logging.basicConfig(level=logging.ERROR)

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--debug', action='store_true', help='Enable debug output')
args = parser.parse_args()


# Data storage for live plotting
gyroroll_data = deque(maxlen=100) 
gyropitch_data = deque(maxlen=100) 
gyroyaw_data = deque(maxlen=100)  
setpitch_data = deque(maxlen=100)  
setroll_data = deque(maxlen=100)  
setyaw_data = deque(maxlen=100)  
motor2_data = deque(maxlen=100)  
motor4_data = deque(maxlen=100)  
motor1_data = deque(maxlen=100)
vbat_data = deque(maxlen=100)  
rssi_data = deque(maxlen=100)
accx_data = deque(maxlen=100)
accy_data = deque(maxlen=100)
accz_data = deque(maxlen=100)
timestamps = deque(maxlen=100)  
timestamps2 = deque(maxlen=100) 
timestamps3 = deque(maxlen=100) 
timestamps4 = deque(maxlen=100)

# Initialize the plot
fig, ((ax, ax2, ax3),(ax4, ax5, ax6)) = plt.subplots(2,3,sharex=True)

line_gyroroll, = ax.plot([], [], label="Roll Rate", color="blue")
line_setroll, = ax.plot([], [], label="Roll Setpoint", color="red")
ax.set_ylim(-5, 5)  # Adjust y-axis range based on expected values
ax.set_title("Roll")
ax.set_xlabel("Time (seconds)")
ax.set_ylabel("Angular Rate (degrees/s)")
ax.legend()

line_gyropitch, = ax2.plot([], [], label="Pitch Rate", color="blue")
line_setpitch, = ax2.plot([], [], label="Pitch Setpoint", color="red")
ax2.set_ylim(-5, 5)  # Adjust y-axis range based on expected values
ax2.set_title("Pitch")
ax2.set_xlabel("Time (seconds)")
ax2.set_ylabel("Angular Rate (degrees/s)")
ax2.legend()

line_gyroyaw, = ax3.plot([], [], label="Yaw Rate", color="blue")
line_setyaw, = ax3.plot([], [], label="Yaw Setpoint", color="red")
#ax3.set_ylim(0,50)  # Adjust y-axis range based on expected values
ax3.set_title("Yaw")
ax3.set_xlabel("Time (seconds)")
ax3.set_ylabel("Angular Rate (degrees/s)")
ax3.legend()

line_motor1, = ax4.plot([], [], label="Motor 1 Command", color="red")
line_motor2, = ax4.plot([], [], label="Motor 2 Command", color="blue")
line_motor4, = ax4.plot([], [], label="Motor 4 Command", color="green")
ax4.set_ylim(-10, 66000)  # Adjust y-axis range based on expected values
ax4.set_title("Live Motor Commands")
ax4.set_xlabel("Time (seconds)")
ax4.set_ylabel("Motor Command Value")
ax4.legend()

line_rssi, = ax5.plot([], [], label="RSSI", color="red")
ax5.set_title("RSSI")
ax5.set_xlabel("Time (seconds)")
ax5.set_ylabel("RSSI Value")
ax5.set_ylim(0, 60)
ax5.legend()

line_accx, = ax6.plot([], [], label="X Acceleration", color="red")
line_accy, = ax6.plot([], [], label="Y Acceleration", color="blue")
line_accz, = ax6.plot([], [], label="Z Acceleration", color="green")
ax6.set_title("Accelerometer Data")
ax6.set_xlabel("Time (seconds)")
ax6.set_ylabel("Gravity (g)")
ax6.set_ylim(-5.0, 5.0)
ax6.legend()



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

    if axis >= -tolerance and axis <= tolerance:
        cf.param.set_value(param, max/2)
        if args.debug:
            print(statement + str(max/2))
        #time.sleep(0.001)
    elif axis < -tolerance:
        cf.param.set_value(param, round_to_nearest((max/2)-int(setValue), 1))
        if args.debug:
            print(statement + str(round_to_nearest((max/2)-int(setValue), 1)))
        #time.sleep(0.001)
    elif axis > tolerance:
        cf.param.set_value(param, round_to_nearest((max/2)+int(setValue),1))
        if args.debug:
            print(statement + str(round_to_nearest((max/2)+int(setValue),1)))
        #time.sleep(0.001)


def setBLMotor(axis,lastVal):
    setValue = round((abs(axis)*65535)/1000)*1000
    if setValue > max:
        setValue = max
    if axis > -0.17 and axis < 0.17:
        cf.param.set_value('servo.servoAngle', 0)
        if args.debug:
            print("Setting BL Motor to 0")
        time.sleep(0.01)
    else:
            if setValue > lastVal+0.03 :
                for i in range(lastVal,setValue,1000):
                    cf.param.set_value('servo.servoAngle', i)
                    if args.debug:
                        print("Setting BL Motor to " + str(i))
                    time.sleep(0.01)
            elif lastVal-0.03 > setValue: 
                for i in range(lastVal,setValue,-1000):
                    cf.param.set_value('servo.servoAngle', i)
                    if args.debug:
                        print("Setting BL Motor to " + str(i))
                    time.sleep(0.01)
            else:
                cf.param.set_value('servo.servoAngle', lastVal)
                if args.debug:
                    print("Setting BL Motor to " + str(lastVal))
                time.sleep(0.01)
    return setValue


def update_plot(frame):
    """
    Update function for the live plot.
    """
    line_gyroroll.set_data(timestamps, gyroroll_data)
    line_gyropitch.set_data(timestamps, gyropitch_data)
    line_gyroyaw.set_data(timestamps, gyroyaw_data)
    line_setroll.set_data(timestamps, setroll_data)
    line_setpitch.set_data(timestamps, setpitch_data)
    line_setyaw.set_data(timestamps, setyaw_data)
    line_motor2.set_data(timestamps2, motor2_data)
    line_motor1.set_data(timestamps2, motor1_data)
    line_motor4.set_data(timestamps2, motor4_data)
    line_rssi.set_data(timestamps3, rssi_data)
    line_accx.set_data(timestamps4, accx_data)
    line_accy.set_data(timestamps4, accy_data)
    line_accz.set_data(timestamps4, accz_data)
    
    ax.relim()  # Recalculate limits based on new data
    ax.autoscale_view()  # Autoscale the view to fit the new data
    
    return line_gyroroll, line_gyropitch,line_gyroyaw, line_setroll, line_setpitch, line_setyaw, line_motor2, line_motor4, line_motor1, line_rssi, line_accx, line_accy, line_accz

def console_callback(text: str):
    print(text,end='')

def main():
    
    global cf

    # Data file setup
    filename = input("Enter the filename to save data (without extension): ")
    with open(filename+"_Controller.csv", 'w', newline='') as csvfile, open(filename+"_Motor.csv", 'w', newline='') as csvfile2, open(filename+"_Connection.csv", 'w', newline='') as csvfile3, open(filename+"_Accelerometer.csv", 'w', newline='') as csvfile4:
        controller_writer = csv.writer(csvfile)
        motor_writer = csv.writer(csvfile2)
        connection_writer = csv.writer(csvfile3)
        accelerometer_writer = csv.writer(csvfile4)

        #Logging Functions
        def log_callback(timestamp, data, logconf):
            """
            Callback function to handle log data received from the Crazyflie.
            This function will be called whenever new log data is received.
            """
            gyro_roll = data['controller.r_roll']
            gyro_pitch = data['controller.r_pitch']
            gyro_yaw = data['controller.r_yaw'] 
            set_pitch = data['controller.pitchRate']
            set_roll = data['controller.rollRate']
            set_yaw = data['controller.yawRate'] 
            timestamp = timestamp / 1000.0  # Convert to seconds for better readability

            # Append data to the deque for live graphing
            gyroroll_data.append(gyro_roll)
            gyropitch_data.append(gyro_pitch)
            gyroyaw_data.append(gyro_yaw)
            setpitch_data.append(set_pitch)
            setroll_data.append(set_roll)
            setyaw_data.append(set_yaw)
            timestamps.append(timestamp)  
            # Write data to CSV file
            controller_writer.writerow([timestamp, gyro_roll, gyro_pitch, gyro_yaw,set_roll , set_pitch, set_yaw])  # Empty placeholders for motor data

        def log_callback2(timestamp, data, logconf):
            """
            Callback function to handle log data received from the Crazyflie.
            This function will be called whenever new log data is received.
            """

            motor_m4 = float(data['motor.m4'])
            motor_m1 = float(data['motor.m1'])
            motor_m2 = float(data['motor.m2'])
            timestamp = timestamp / 1000.0  # Convert to seconds for better readability
            # Append data to the deque for live graphing
            motor4_data.append(motor_m4)
            motor1_data.append(motor_m1)
            motor2_data.append(motor_m2)
            timestamps2.append(timestamp)  
            # Write data to CSV file
            motor_writer.writerow([timestamp,motor_m4, motor_m1, motor_m2])  # Empty placeholders for gyro and acc data

        def log_callback3(timestamp, data, logconf):
            """
            Callback function to handle log data received from the Crazyflie.
            This function will be called whenever new log data is received.
            """
            rssi = float(data['radio.rssi'])
            vbat = float(data['pm.vbat'])
            timestamp = timestamp / 1000.0  # Convert to seconds for better readability
            rssi_data.append(rssi)
            vbat_data.append(vbat)
            timestamps3.append(timestamp)
            if timestamp%60==0 or vbat<7:
                print("VBAT:",vbat)
            connection_writer.writerow([timestamp,rssi, vbat])  # Empty placeholders for gyro, motor and acc data
        
        def log_callback4(timestamp, data, logconf):
            """
            Callback function to handle log data received from the Crazyflie.
            This function will be called whenever new log data is received.
            """
            acc_x = float(data['acc.x'])
            acc_y = float(data['acc.y'])
            acc_z = float(data['acc.z'])
            timestamp = timestamp / 1000.0
            accx_data.append(acc_x)
            accy_data.append(acc_y)
            accz_data.append(acc_z)
            timestamps4.append(timestamp)
            accelerometer_writer.writerow([timestamp,acc_x, acc_y, acc_z])  # Empty placeholders for gyro and motor data

        def log_async(cf,logconf):
            
            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback)

        def log_async_motor(cf,logconf):
            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback2)
        
        def log_async_conn(cf,logconf):
            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback3)
        
        def log_async_acc(cf,logconf):
            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback4)

        #Initialization Functions
        cflib.crtp.init_drivers()
        pygame.init()
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        #Set Up Logging Configurations
        lg_stab = LogConfig(name='Controller', period_in_ms=10)
        lg_stab.add_variable('controller.r_roll', 'float')
        lg_stab.add_variable('controller.r_pitch', 'float')
        lg_stab.add_variable('controller.r_yaw', 'float')
        lg_stab.add_variable('controller.pitchRate', 'float')
        lg_stab.add_variable('controller.rollRate', 'float')
        lg_stab.add_variable('controller.yawRate', 'float')

        lg_motor = LogConfig(name='Motor', period_in_ms=10)
        lg_motor.add_variable('motor.m4', 'uint16_t')
        lg_motor.add_variable('motor.m1', 'uint16_t')
        lg_motor.add_variable('motor.m2', 'uint16_t')

        lg_conn = LogConfig(name='Connection', period_in_ms=10)
        lg_conn.add_variable('radio.rssi', 'uint8_t')
        lg_conn.add_variable('pm.vbat', 'float') 

        lg_acc = LogConfig(name='Accelerometer', period_in_ms=10)
        lg_acc.add_variable('acc.x', 'float')
        lg_acc.add_variable('acc.y', 'float')
        lg_acc.add_variable('acc.z', 'float') 


        # Communication with Flight Computer
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                
                # Setup Instances
                cf = scf.cf
                Commander = scf.cf.commander
                print("Connected to Crazyflie with URI:", uri)

                #Set Parameters for Flight
                cf.param.set_value('motorPowerSet.enable', '0')
                cf.param.set_value('flightmode.stabModeRoll', '0')
                cf.param.set_value('flightmode.stabModePitch', '0')
                cf.param.set_value('flightmode.stabModeYaw', '0')
                cf.param.set_value('pid_rate.pitch_kp', -200)
                cf.param.set_value('pid_rate.pitch_ki', 25)
                cf.param.set_value('pid_rate.pitch_kd', 0)
                cf.param.set_value('pid_rate.pitch_kff', 0)
                cf.param.set_value('pid_rate.yaw_kp', 100)
                cf.param.set_value('pid_rate.yaw_ki', 25)
                cf.param.set_value('pid_rate.yaw_kd', 0)
                cf.param.set_value('pid_rate.yaw_kff', 0) 
                cf.param.set_value('pid_rate.roll_kp', -100)
                cf.param.set_value('pid_rate.roll_ki', 25)
                cf.param.set_value('pid_rate.roll_kd', 0)
                cf.param.set_value('pid_rate.roll_kff', 0)  

                #Start Handling Communication from Flight Computer
                log_async(cf, lg_stab)  
                log_async_motor(cf, lg_motor)
                log_async_conn(cf, lg_conn)
                log_async_acc(cf, lg_acc)
                cf.console.receivedChar.add_callback(console_callback)

                stabRun = input("Log Rate Controller? (0 = False, 1 = True): ")
                if float(stabRun)==1:
                    lg_stab.start()
                    controller_writer.writerow(['Timestamp', 'Gyro Roll', 'Gyro Pitch', 'Gyro Yaw', 'Set Roll', 'Set Pitch', 'Set Yaw'])

                motorRun = input("Log Motor Data? (0 = False, 1 = True): ")
                if float(motorRun)==1:
                    lg_motor.start()  
                    motor_writer.writerow(['Timestamp', 'Motor 4', 'Motor 1', 'Motor 2'])

                connRun = input("Log Connection Data? (0 = False, 1 = True): ")
                if float(connRun)==1:
                    lg_conn.start()
                    connection_writer.writerow(['Timestamp', 'RSSI', 'VBAT'])

                accRun = input("Log Accelerometer Data? (0 = False, 1 = True): ")
                if float(accRun)==1:
                    lg_acc.start()
                    accelerometer_writer.writerow(['Timestamp', 'Acc X', 'Acc Y', 'Acc Z'])
                time.sleep(0.1)            

                #Initalize Flight Controls
                setMotor(0,2)
                setMotor(0,3)
                lastVal = 0
                trimmed = False
                motor = False  
                autnomous = False  
         
                cf.platform.send_arming_request(True)
                time.sleep(0.1) 
                print("Glider Confiugred and Ready for Control Input")
                
                # Update the live plot    
                anim = animation.FuncAnimation(fig, update_plot, interval=1,cache_frame_data=False)  # Update every 1ms
                plt.tight_layout()  # Adjust layout to prevent overlap
                plt.show(block=False)  # Non-blocking mode for live plotting

                #Main Control Loop
                try:
                        while True:
                            plt.pause(0.0001)  # Allow the plot to update

                            pygame.event.pump()

                            x = joystick.get_axis(3) #Roll
                            x = round(x, 4)

                            y = joystick.get_axis(4) #Pitch
                            y = round(y, 4)

                            twist = joystick.get_axis(0) #Yaw
                            twist = round(twist, 2)

                            throttle = joystick.get_axis(2) #Throttle  
                            throttle = (throttle+1)/2
                            throttle = round(throttle, 2)

                            if joystick.get_button(0) == 1:  # Trim Button
                                trimmed = True
                            if joystick.get_button(3) == 1:  # Untrim Button
                                trimmed = False
                            if joystick.get_button(8) == 1:  # DAQ Breakpoint Button
                                controller_writer.writerow('Break')
                                motor_writer.writerow('Break')
                                accelerometer_writer.writerow('Break')
                                connection_writer.writerow('Break')
                            if joystick.get_button(4) == 1:  # Arm Motor Button
                                motor = True
                                cf.param.set_value('servo.servoAngle', 11000)
                            if joystick.get_button(5) == 1:  # Disarm Motor Button  
                                motor = False
                                lastVal = setBLMotor(0,lastVal)
                            if joystick.get_button(2) == 1: # Autonomous Mode Enable Button
                                autnomous = True
                                cf.param.set_value('motorPowerSet.enable', '0')
                                time.sleep(0.01)
                            if joystick.get_button(1) == 1: # Autonomous Mode Disable Button
                                autnomous = False
                                cf.param.set_value('motorPowerSet.enable', '0')
                                time.sleep(0.01)
                            if joystick.get_button(9) ==1: # PID Tuning Button
                                tempKP = input("Enter new Pitch KP value: ")
                                tempKI = input("Enter new Pitch KI value: ")
                                tempKD = input("Enter new Pitch KD value: ")
                                cf.param.set_value('pid_rate.pitch_kp', float(tempKP))
                                cf.param.set_value('pid_rate.pitch_ki', float(tempKI))
                                cf.param.set_value('pid_rate.pitch_kd', float(tempKD))
                                tempKP = input("Enter new Yaw KP value: ")
                                tempKI = input("Enter new Yaw KI value: ")
                                tempKD = input("Enter new Yaw KD value: ")
                                cf.param.set_value('pid_rate.yaw_kp', float(tempKP))
                                cf.param.set_value('pid_rate.yaw_ki', float(tempKI))
                                cf.param.set_value('pid_rate.yaw_kd', float(tempKD))
                                tempKP = input("Enter new Roll KP value: ")
                                tempKI = input("Enter new Roll KI value: ")
                                tempKD = input("Enter new Roll KD value: ")
                                cf.param.set_value('pid_rate.roll_kp', float(tempKP))
                                cf.param.set_value('pid_rate.roll_ki', float(tempKI))
                                cf.param.set_value('pid_rate.roll_kd', float(tempKD))
                            
                            if autnomous:
                                setpoint = 0
                                Commander.send_setpoint(setpoint,-setpoint,-setpoint,10001)
                                time.sleep(0.01)
                            else:     
                                if not trimmed:
                                    #setMotor(y,4)
                                    #setMotor(x,3)
                                    #setMotor(twist,1)
                                    Commander.send_setpoint(x*10,-y*5,-twist*5,10001)
                                if motor:
                                    lastVal = setBLMotor(throttle,lastVal)
                            
                            
                            
                except KeyboardInterrupt:
                        print("Exiting...")
                        cf.param.set_value('motorPowerSet.enable', '0')
                        time.sleep(0.1)
                        pygame.quit()
                        Commander.send_stop_setpoint()
                        plt.close(fig)
                        lg_motor.stop()
                        lg_stab.stop()
                        lg_conn.stop()
                        time.sleep(0.1)
                        csvfile.close()
                        cf.close_link()
                        pass
        
if __name__ == '__main__':
    main()