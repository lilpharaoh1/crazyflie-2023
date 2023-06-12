import time

import cflib.crtp
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from connect import DEFAULT_HEIGHT

URIS = [
    'radio://0/80/2M/E8E8E9E8E8',
    'radio://0/80/2M/E8E8E9E8E9'
]

DEFAULT_HEIGHT = 0.00001

def spin_motors(scf, t=0.1):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(t)
        mc.land()

seq_args = {}
for uri in URIS:
    seq_args[uri] = [(uri, 0.1)]

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(URIS, factory=factory) as swarm:
        print('Connected to  Crazyflies')
        swarm.parallel_safe(spin_motors, args_dict=seq_args)