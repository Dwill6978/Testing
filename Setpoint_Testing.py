
import cflib.crtp
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

def main():
    cflib.crtp.init_drivers()
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        commander = cf.commander
        cf.param.set_value('flightmode.stabModeRoll', '0')
        cf.param.set_value('flightmode.stabModePitch', '0')
        cf.param.set_value('flightmode.stabModeYaw', '0')


if __name__ == '__main__':
    main()