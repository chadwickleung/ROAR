from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
import cv2
import cv2.aruco as aruco
import numpy as np
from collections import deque


class ArucoFollowingAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle, agent_settings, **kwargs)
        key = getattr(aruco, f'DICT_{5}X{5}_{250}')
        self.arucoDict = aruco.Dictionary_get(key)
        self.arucoParam = aruco.DetectorParameters_create()
        self.tracking_id = 0
        self.marker_length = 0.005       # in meters

        self.distance_threshold = 0.04  # in tvec unit
        self.center = 0
        self.turn_threshold = 0.01
        # self.left_threshold = -0.01     # 0.0018
        # self.right_threshold = 0.01    # 0.013
        self.steering_boundary = [-0.5, 0.5]

        self._error_buffer = deque(maxlen=10)
        self._dt = 0.03

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super().run_step(sensors_data=sensors_data, vehicle=vehicle)
        result: dict = self.findArucoMarkers()  # {marker id -> bbox}
        if result:
            img = self.front_rgb_camera.data.copy()
            if self.tracking_id in result:
                bbox = result[self.tracking_id]
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(bbox, self.marker_length,
                                                                               self.front_rgb_camera.intrinsics_matrix,
                                                                               self.front_rgb_camera.distortion_coefficient)
                x, y, z = tvec[0][0]
                print(x, y, z)

                if z < 0.05:
                    # stopping criteria
                    return VehicleControl(throttle=0, steering=0)
                else:
                    if x < self.center - self.turn_threshold:
                        # if the car is to your left
                        turn = self.run_in_series(x)
                        print("Turn left")
                        return VehicleControl(throttle=0.16, steering=turn)
                    elif x > self.center + self.turn_threshold:
                        # if the car is to your right
                        turn = self.run_in_series(x)
                        print("Turn right")
                        return VehicleControl(throttle=0.16, steering=-turn)
                    else:
                        # reset error buffer
                        self._error_buffer = deque(maxlen=10)
                        return VehicleControl(throttle=0.16, steering=0)


        return self.vehicle.control

    def run_in_series(self, x) -> float:
        target_x = self.center
        current_x = x

        k_p, k_d = 20, -5
        error = target_x - current_x

        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            # print(self._error_buffer[-1], self._error_buffer[-2])
            _de = (self._error_buffer[-2] - self._error_buffer[-1]) / self._dt
        else:
            _de = 0.0


        output = float(np.clip((k_p * error) + (k_d * _de), self.steering_boundary[0],
                               self.steering_boundary[1]))
        # self.logger.debug(f"curr_speed: {round(current_speed, 2)} | kp: {round(k_p, 2)} | kd: {k_d} | ki = {k_i} | "
        #       f"err = {round(error, 2)} | de = {round(_de, 2)} | ie = {round(_ie, 2)}")
        # f"self._error_buffer[-1] {self._error_buffer[-1]} | self._error_buffer[-2] = {self._error_buffer[-2]}")
        return output

    def findArucoMarkers(self):
        if self.front_rgb_camera.data is not None:
            img = self.front_rgb_camera.data.copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            bboxs, ids, rejected = aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParam,
                                                       cameraMatrix=self.front_rgb_camera.intrinsics_matrix,
                                                       distCoeff=self.front_rgb_camera.distortion_coefficient)
            log = dict()
            if ids is None:
                return log
            else:
                for i in ids:
                    log[i[0]] = bboxs
                return log
        return dict()
