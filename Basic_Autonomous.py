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
        mc.up(0.5, 0.2)
        time.sleep(300)
        #mc.forward(0.5, 1)
        #time.sleep(1)
        #mc.back(0.5, 1)
        #time.sleep(1)
        #mc.left(0.5, 1)
        #time.sleep(1)
        #mc.right(0.5, 1)
        #mc.down(0.3, 1)
        # mc.turn_left(90, 45)
        # time.sleep(1)
        # mc.turn_right(90, 45)
        #time.sleep(1)
        #mc.go_to(1,1,1.5)
        #time.sleep(2)
        #time.sleep(1)
        #mc.land()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
        #                                  cb=param_deck_flow)
        
        time.sleep(1)

        # if not deck_attached_event.wait(timeout=5):
        #     print('No flow deck detected!')
        #     sys.exit(1)

        take_off_simple(scf)
