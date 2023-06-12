#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
VELOCITY = 0.5
inputs = [0, 1, 2, 3, 2, 1, 0]


if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


class Drone:
    def __init__(self, default_height=0.3):
        pass

    def drive(self, velocity, mc):
        velocity_x, velocity_y, velocity_z = velocity
        mc.start_linear_motion(
                        velocity_x, velocity_y, velocity_z)


    def check_if_close(self, multiranger, velocity):
        if is_close(multiranger.front):
            velocity[0] -= VELOCITY
        if is_close(multiranger.back):
            velocity[0] += VELOCITY

        if is_close(multiranger.left):
            velocity[1] -= VELOCITY
        if is_close(multiranger.right):
            velocity[1] += VELOCITY
        
        if is_close(multiranger.up):
            velocity[2] -= VELOCITY
        if is_close(multiranger.down):
            velocity[2] += VELOCITY

        return velocity

    def input_to_velocity(self, input):
        if input == 0:
            return [0.0, 0.1, 0.0]
        if input == 1:
            return [0.1, 0.0, 0.0]
        if input == 2:
            return [0.0, 0.0, 0.1]
        return [0.0, 0.0, 0.0]
            

    def spin(self):
        # Initialize the low-level drivers
        cflib.crtp.init_drivers()

        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(URI, cf=cf) as scf:
            with MotionCommander(scf) as motion_commander:
                with Multiranger(scf) as multiranger:
                    keep_flying = True
                    for _ in range(2):
                        for input in inputs:
                            velocity = self.input_to_velocity(input)
                            velocity = self.check_if_close(multiranger, velocity)
                            self.drive(velocity, motion_commander)
                            time.sleep(0.5)

                print('Demo terminated!')
                motion_commander.land()



if __name__ == '__main__':
    drone = Drone()
    drone.spin()