import logging
import threading
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncLogger import SyncLogger



from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5

logging.basicConfig(level=logging.ERROR)

def take_off_simple(scf):
    with PositionHlCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.up(0.25, 0.3)
        time.sleep(3)
        mc.up(0.25, 0.3)
        time.sleep(3)
        mc.back(0.5, 0.3)
        time.sleep(3)
        mc.left(0.5, 0.3)
        time.sleep(3)
        mc.land()

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    # time.sleep(1)
    # logconf.stop()
    

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        time.sleep(1)

        lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('stateEstimate.z', 'float')

        thread = threading.Thread(target=simple_log_async, args=(scf, lg_stab))
        thread.start()
        # thread = threading.Thread(target=take_off_simple, args=(scf))
        # thread.start()
        while(True):
            time.sleep(1)
        