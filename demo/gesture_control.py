#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging
import sys
import time
import cv2
import numpy as np
from sklearn.metrics import pairwise

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
VELOCITY = 0.5
INPUT_MIN_THRESH = 136

ROI_TOP = 100  # Region of interest in pixels, where top left is (0,0)
ROI_BOTTOM = 300
ROI_LEFT = 10
ROI_RIGHT = 210
DEFAULT_BACKGROUND_WEIGHT = 0.5
DEFAULT_THRESHOLD = 12

inputs = [0, 1, 2, 3, 2, 1, 0]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

class FrameHandler:
    def __init__(self):
        self.background_init = False
        self.background_collect = False
        self.background = None
        self.background_count = 0
        self.background_weight = DEFAULT_BACKGROUND_WEIGHT

    def background_avg(self, img):  # Average background together
        # For first time, create the background from a copy of the frame.
        if self.background is None:
            self.background = img.copy().astype("float")
            return None
        # Accumulate weights and update background
        cv2.accumulateWeighted(img, self.background, self.background_weight)

    def clean_frame(self, frame):
        frame = cv2.flip(frame, 1)  # horizontal flip
        frame_copy = frame.copy()

        # press b to reset background
        cv2.putText(frame_copy, 'press b to', (10, 390), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
        cv2.putText(frame_copy, 'set background', (10, 420), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)

        roi = frame[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT]  # Grab the ROI from the frame
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  # Apply grayscale and blur to ROI
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        if self.background_collect and self.background_count < 60:
            self.background_count += 1
            self.background_avg(gray)
            # if num_frames <= 59:
            cv2.putText(frame_copy, "Averaging Background,", (0, 30), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            cv2.putText(frame_copy, "  please remove hand.", (0, 70), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            cv2.imshow("Finger Count",frame_copy)

        elif self.background_count == 60:
            self.background_init = True
            self.background_collect = False
            self.background_count = 0
            self.background_weight = DEFAULT_BACKGROUND_WEIGHT # reset weight when finished

        return frame_copy, gray

    def calc_input(self, frame, gray):
        thresholded = self.segment_hand(gray, DEFAULT_THRESHOLD) # get thresholded hand image

        fingers, gest = 0, 'none'

        if thresholded is not None:
            kernel = np.ones((3,3),np.uint8)  # extrapolate the hand to fill dark spots within
            thresholded = cv2.dilate(thresholded,kernel,iterations = 1)
            fingers, gest = self.count_fingers(thresholded) # count the fingers

            # Display count and create a circle around the center
            if gest != 'none':
                cv2.putText(frame, str(fingers), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
            else:
                cv2.putText(frame, gest, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
            max_distance = (ROI_RIGHT - ROI_LEFT) // 2
            radius = int(0.75 * max_distance)
            cv2.circle(frame, (((ROI_LEFT - ROI_RIGHT) // 2)+ROI_RIGHT,((ROI_BOTTOM - ROI_TOP) // 2)+ROI_TOP),radius,150,3)

            cv2.imshow("Thresholded", thresholded)  # display the thresholded image

        cv2.rectangle(frame, (ROI_LEFT, ROI_TOP), (ROI_RIGHT, ROI_BOTTOM), (255,255,0), 2)

        return fingers, gest, frame

    def segment_hand(self, frame, threshold):
        # Difference between background and current frame
        diff = cv2.absdiff(self.background.astype("uint8"), frame)

        # Use a threshold to find the hand in diff
        _ , thresholded = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)

        # Find the contours
        contours, hierarchy = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return None
        return thresholded

    def count_fingers(self, thresholded): # Count fingers based on threshold and circular mask
        gesture = 'None'  # Gesture variables
        thumb = False
        pinky = False
        fing_conts = []  # List of finger contours

        cX = (ROI_RIGHT - ROI_LEFT) // 2  # Center of roi
        cY = (ROI_BOTTOM - ROI_TOP) // 2

        max_distance = (ROI_RIGHT - ROI_LEFT) // 2  # Max distince of circular mask
        radius = int(0.75 * max_distance)  # Create a circle around the center

        # Draw roi and use bit-wise AND with threshold to find fingertip contours
        circular_roi = np.zeros(thresholded.shape[:2], dtype="uint8")
        cv2.circle(circular_roi, (((ROI_RIGHT - ROI_LEFT) // 2), ((ROI_BOTTOM - ROI_TOP) // 2)), radius, 255, 3)
        circular_roi = cv2.bitwise_and(thresholded, thresholded, mask=circular_roi)
        contours, hierarchy = cv2.findContours(circular_roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        count = 0  # Finger count starts at 0
        # fing_conts = []  # List of finger contours
        dist_check = True


        for cnt in contours: # Count fingers
            (x, y, w, h) = cv2.boundingRect(cnt)  # Bounding box of countour

            for fing_cnt in fing_conts: # Don't count contours if they are close together
                (x_f, y_f, w_f, h_f) = cv2.boundingRect(fing_cnt)
                fing_cntX = x_f + w_f/2
                fing_cntY = y_f - h_f/2
                cnt_distance = pairwise.euclidean_distances([(cX, cY)], [(fing_cntX, fing_cntY)])[0]
                dist_check = cnt_distance.max() > 0.07*ROI_BOTTOM and cnt_distance.max() > 0.07*ROI_RIGHT

            out_of_wrist = ((cY + (cY * 0.4)) > (y + h))  # Check for wrist on bottom of roi
            if  out_of_wrist and dist_check:  # add to contour count
                fing_conts.append(cnt)
                count += 1
                if count > 5:
                    count = 5

            # Additional gestures from thumb and pinky recognition
            if count == 1 and thumb:                    gesture = 'thumb'
            elif count == 1 and pinky:                  gesture = 'pinky'
            elif count == 2 and thumb and pinky:        gesture = 'pinky and thumb'
            elif count == 2 and thumb and not pinky:    gesture ='L'
            elif count == 2 and not thumb and pinky:     gesture = 'horns'
            elif count == 3 and thumb and not pinky:    gesture = 'three thumb'
            elif count == 3 and pinky and thumb:        gesture = 'rock out!'
            else: gesture = str(count)

            cv2.imshow('circ_roi',circular_roi)
        return count, gesture
    

class Drone:
    def __init__(self, default_height=0.3):
        self.fh = FrameHandler()
        self.input = [0, 0]

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

    def handle_input_thresholding(self, input):
        if self.input[0] == input:
            self.input[1] += 1
        else:
            self.input = [input, 0]
        return

    def fingers_to_velocity(self, fingers, gest):
        if gest == 'thumb': # Right
            velocity = [0.0, 0.1, 0.0]
            input = 'right'
        elif gest == 'pinky': # Left
            velocity = [0.0, -0.1, 0.0]
            input = 'left'
        elif gest == 'horns': # Forward
            velocity = [0.1, 0.0, 0.0]
            input = 'forward'
        elif gest == 'rock out!': # Backward
            velocity = [-0.1, 0.0, 0.0]
            input = 'backward'
        elif gest == '1': # Up
            velocity = [0.0, 0.0, 0.1]
            input = 'up'
        elif gest == '2': # Down
            velocity = [0.0, 0.0, -0.1]
            input = 'down'
        else:
            velocity = [0.0, 0.0, 0.0]
            input = 'none'

        return velocity, input

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
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FPS, 48)

        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(URI, cf=cf) as scf:
            with MotionCommander(scf, default_height=0.01) as motion_commander:
                with Multiranger(scf) as multiranger:
                    keep_flying = True
                    while (keep_flying):
                        ret, frame = cam.read()
                        frame, gray = self.fh.clean_frame(frame)
                        
                        print(frame.shape)
                        if not self.fh.background is None:
                            fingers, gest, frame = self.fh.calc_input(frame, gray)
                            velocity, input = self.fingers_to_velocity(fingers, gest)
                            self.handle_input_thresholding(input)
                            print("Input : ", self.input)
                            if self.input[1] > INPUT_MIN_THRESH:
                                velocity = self.check_if_close(multiranger, velocity)
                                self.drive(velocity, motion_commander)
                        cv2.imshow("Finger Count", frame)
                        pressedKey = cv2.waitKey(1)
                        if pressedKey == ord('q'):  # q to quit
                            break
                        elif pressedKey == ord('b'):  # b to create background
                            self.fh.background_collect = True
                print('Demo terminated!')
                motion_commander.land()
         # Release the camera and destroy all the windows
        cam.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    drone = Drone()
    drone.spin()