import logging
import time
import csv
import signal
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from xbox360controller import Xbox360Controller
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
#UINT16_MAX
max = 65535
    

def syn_log(scf,log_stab):
    with SyncLogger(scf, log_stab) as logger:
        with open('log.csv', mode='a', newline='') as csvfile:
            fieldNames = ['time', 'roll', 'pitch', 'yaw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldNames) 
            writer.writeheader()
            for log_entry in logger:
                global  time, data
                time = log_entry[0]
                data = log_entry[1]
                name = log_entry[2] 
        
                roll = data['stabilizer.roll']
                pitch = data['stabilizer.pitch']    
                yaw = data['stabilizer.yaw']
                #print('[%s] [%s] [%s]' % (data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw']))
                writer.writerow({'time': time, 'roll': roll, 'pitch': pitch, 'yaw': yaw})

            

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    log_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    log_stab.add_variable('stabilizer.roll', 'float')   
    log_stab.add_variable('stabilizer.pitch', 'float')
    log_stab.add_variable('stabilizer.yaw', 'float')
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
    
        syn_log(scf,log_stab)
        
        
        