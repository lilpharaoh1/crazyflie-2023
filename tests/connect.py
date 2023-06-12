import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander



from cflib.utils import uri_helper

URI_DEFAULT = 'radio://0/80/2M/E7E7E7E7E7'

URI = uri_helper.uri_from_env(default=URI_DEFAULT)

DEFAULT_HEIGHT = 0.00001

def spin_motors(scf, t=0.1):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(t)
        mc.land()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to ", URI_DEFAULT)
        time.sleep(3)
        spin_motors(scf)
