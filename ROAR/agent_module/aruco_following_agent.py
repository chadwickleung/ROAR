from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
import cv2
import cv2.aruco as aruco
import numpy as np
import os
from pathlib import Path


def loadCalib(caliberation_file: Path):
    npzfile = np.load(caliberation_file.as_posix())
    return npzfile['intrinsics'], npzfile['distortion'], \
           npzfile['new_intrinsics'], npzfile['roi']

def findArucoMarkers(img, arucoDict, arucoParam, intrinsics, distortion):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam,
                                               cameraMatrix=intrinsics,
                                               distCoeff=distortion)
    return bboxs, ids, rejected

class ArucoFollowingAgent(Agent):
    markerSize = 5
    totalMarkers = 1
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    intrinsics, distortion, new_intrinsics, roi = loadCalib(Path("../../misc/calib.npz"))
    cap = cv2.VideoCapture(0)

    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle, agent_settings, **kwargs)

    def load_tvec(self, markerSize, totalMarkers, intrinsics: np.ndarray, distortion: np.ndarray):
        success, img = cap.read()
        if success:
            corners, ids, rejected = findArucoMarkers(img, arucoDict, arucoParam, intrinsics, distortion)
            if len(corners) > 0:
                for i in range(0, len(ids)):
                    # Estimate pose of each marker and return the values rvec and tvec---
                    #   (different from those of camera coefficients)
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02,
                                                                                   intrinsics,
                                                                                   distortion)
                    # Draw a square around the markers
                    cv2.aruco.drawDetectedMarkers(img, corners)

                    # Draw Axis
                    cv2.aruco.drawAxis(img, intrinsics, distortion, rvec, tvec, 0.01)
                    print(f"id = {ids[i]} --> tvec = {tvec}")
        return tvec







    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super().run_step(sensors_data=sensors_data, vehicle=vehicle)
        tvec = self.load_tvec()
        print(tvec)
        depth = 0
        horizontal = 1
        depth_threshold = 2
        left_threshold = -0.5
        right_threshold = 0.5

        distance = tvec[depth]
        position = tvec[horizontal]
        if distance < threshold:
            print("Slow down")
        else:
            if position < left_threshold:
                print("Turn left")
            elif position > right_threshold:
                print("Turn right")
            else:
                print("Keep going")

        # if depth <= threshold (too close, highest priority)
        #   slow down
        # else
        #   if yaw == 0 (img perpendicular to us, use tvec or rvec to determine)
        #
        #       if img_pos == center (img's corners within a range)
        #           just follow
        #       elif img_pos < center (img on the left)
        #           move left and drive forward (Vehicle Control)
        #       elif img_pos > center (img on the right)
        #           move right and drive  (Vehicle Control)
        #
        #   elif yaw > 0 (img is rotated, right side closer, left side further)
        #       turn right (Vehicle control)
        #
        #   elif yaw < 0 (left side closer, right side further)
        #       turn left (Vehicle control)


        # self.logger.info(self.vehicle.get_speed(self.vehicle))

        return VehicleControl(throttle=0.4, steering=0)
