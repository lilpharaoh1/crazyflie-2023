import logging
import threading
import sys
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncLogger import SyncLogger



from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

class Drone: 
    def __init__(self, waypoints=None):
        cflib.crtp.init_drivers()

        self.pose = np.zeros((6, 1))
        self.drive = np.zeros((3, 1))
        self.waypoints = waypoints
        self.obj = None#self.waypoints[0]
        self.done = False
        # self.scf = 

        self.logger = LogConfig(name='stateEstimate', period_in_ms=10)
        self.logger.add_variable('stateEstimate.x', 'float')
        self.logger.add_variable('stateEstimate.y', 'float')
        self.logger.add_variable('stateEstimate.z', 'float')

    def log_pose(self, scf):
        with SyncLogger(scf, self.logger) as logger:

            for log_entry in logger:
                _, data, _ = log_entry

                self.pose[0, 0] = data['stateEstimate.x']
                self.pose[1, 0] = data['stateEstimate.y']
                self.pose[2, 0] = data['stateEstimate.z']
                break

    def at(self, point):
        return [abs(point[idx]-self.pose[idx, 0]) < 0.1 for idx in range(3)]

    def calc_distance(self):
        if isinstance(self.obj, np.ndarray):
            return self.obj - self.pose[0:3]
        return [[0], [0], [0]]

    def spin(self):
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            time.sleep(1)
            while(not self.done):
                self.log_pose(scf)
                print(self.pose)
                print(self.at([0,0,0]))
                time.sleep(0.05)

drone = Drone()
drone.spin()
