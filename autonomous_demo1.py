import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander



from cflib.utils import uri_helper



URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def take_off_simple(scf):
    with PositionHlCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.up(0.3, 0.3)
        time.sleep(3)
        mc.up(0.5, 0.3)
        time.sleep(3)
        mc.back(0.5, 0.3)
        time.sleep(3)
        mc.left(0.5, 0.3)
        time.sleep(3)
        mc.land()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        time.sleep(3)
        take_off_simple(scf)
