import logging
import threading
import sys
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncLogger import SyncLogger



from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_VELOCITY = 0.75
POINT_TOLERANCE = 0.2
WAIT_TIME = 1.0
waypoints = np.array([[0.0,  0.0,  1.0],
                      [0.3,  0.3,  1.0],
                      [-0.4,  -0.3,  1.0]])#,
                    #   [-0.4, 0.3, 1.0],
                    #   [0.0, -0.3, 0.5],
                    #   [0.0, -0.3, 1.3],
                    #   [0.0, 0.3, 1.0],
                    #   [0.0, 0.0, 1.0]])


class Drone: 
    def __init__(self, waypoints=None):
        cflib.crtp.init_drivers()

        self.pose = np.zeros((6, 1))
        self.drive = np.zeros((3, 1))
        self.waypoints = waypoints
        self.obj = None if not len(self.waypoints) else self.waypoints[0].reshape((3,1))
        self.done = False

        self.logger = LogConfig(name='stateEstimate', period_in_ms=10)
        self.logger.add_variable('stateEstimate.x', 'float')
        self.logger.add_variable('stateEstimate.y', 'float')
        self.logger.add_variable('stateEstimate.z', 'float')

    def log_pose(self, scf, logger):
        for log_entry in logger:
            _, data, _ = log_entry

            self.pose[0, 0] = data['stateEstimate.x']
            self.pose[1, 0] = data['stateEstimate.y']
            self.pose[2, 0] = data['stateEstimate.z']
            break

    def at(self, point):
        if np.allclose(point, self.pose[0:3], atol=POINT_TOLERANCE):
            return True
        return False
    
    def waypoint_select(self, at):
        if not at:
            return self.obj
        self.waypoints = np.delete(self.waypoints, 0, axis=0)
        self.obj = None if not len(self.waypoints) else self.waypoints[0].reshape((3,1))

    def calc_obj_distance(self):
        return self.obj - self.pose[0:3]

    def spin(self):
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            with PositionHlCommander(scf, default_height=0.01) as mc:
                with SyncLogger(scf, self.logger) as logger:
                    while(not self.done):
                        print("------------------------")
                        self.log_pose(scf, logger)
                        self.waypoint_select(self.at(self.obj))
                        print(self.pose[0:3], self.obj)
                        if not isinstance(self.obj, np.ndarray):
                            self.done = True
                            mc.land()
                            time.sleep(3)
                            break
                        mc.go_to(self.obj[0,0], self.obj[1,0], self.obj[2,0])
                        time.sleep(WAIT_TIME)

drone = Drone(waypoints=waypoints)
drone.spin()
