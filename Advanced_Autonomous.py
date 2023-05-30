import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

uris = [
    'radio://0/80/2M/E8E8E9E8E8',
    'radio://0/80/2M/E8E8E9E8E9',
    'radio://0/80/2M/E1E1E1E1E1',
    'radio://0/80/2M/E1E1E1E1E2',

    # Add more URIs if you want more copters in the swarm
]

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)


def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()



h = 0.7 # remain constant height similar to take off height
x0, y0 = +1.0, +1.0
x1, y1 = -1.0, -1.0

t = 1.5

x_bias = 1.75

y_bias = 1.75

#    x   y   z  time
sequence0 = [
    (0*x_bias, 0.75*y_bias, h, t),
    (0.75*x_bias/2, 0.75*y_bias/2, h, t),
    (0*x_bias, 0.25*y_bias, h, t),
    (-0.25*x_bias, -0.25*y_bias, h, t),
    (0.25*x_bias, -0.25*y_bias, h, t),
    (0*x_bias,  0.25*y_bias, h, t),
    (0*x_bias, 0.75*y_bias, h, t),

]


sequence1 = [
    (0*x_bias, -.25*y_bias, h, t),
    (-0.75*x_bias/2, -0.75*y_bias/2, h, t),
    (-0.25*x_bias, -0.25*y_bias, h, t),
    (0.25*x_bias, -0.25*y_bias, h, t),

    (0*x_bias, 0.25*y_bias, h, t),
    (-0.25*x_bias,  -0.25*y_bias, h, t),
    (0*x_bias, 0.25*y_bias, h, t),

]

sequence2 = [
    (0*x_bias, -0.75*y_bias, h, t),
    (0.75*x_bias/2, -0.75*y_bias/2, h, t),
    (0.25*x_bias, -0.25*y_bias, h, t),
    (0*x_bias, 0.25*y_bias, h, t),
    (-0.25*x_bias, -0.25*y_bias, h, t),
    (0.25*x_bias,  -0.25*y_bias, h, t),
    (0*x_bias, -0.75*y_bias, h, t),

]

sequence3 = [
    (0*x_bias, 0.25*y_bias, h, t),
    (-0.75*x_bias/2, 0.75*y_bias/2, h, t),
    (0*x_bias, 0*y_bias, h, t),
    (0*x_bias, 0*y_bias, h-0.3, t),
    (0*x_bias,0*y_bias, h+0.3, t),
    (0*x_bias, 0*y_bias, h-0.3, t),
    (0*x_bias, 0*y_bias, h-0.3, t),

]

seq_args = {
    uris[0]: [sequence0],
    uris[1]: [sequence1],
    uris[2]: [sequence2],
    uris[3]: [sequence3]
}


def hover_sequence(scf):
    take_off(scf)
    land(scf)


def run_sequence(scf, sequence):
    cf = scf.cf

    for arguments in sequence:
        commander = scf.cf.high_level_commander

        x, y, z = arguments[0], arguments[1], arguments[2]
        duration = arguments[3]

        print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
        commander.go_to(x, y, z, 0, duration, relative=False)
        time.sleep(duration)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to  Crazyflies')
        # swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        # swarm.sequential(hover_sequence)
        swarm.parallel_safe(take_off)
        swarm.parallel_safe(run_sequence, args_dict=seq_args)
        swarm.parallel_safe(land)