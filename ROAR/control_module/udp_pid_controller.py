from ROAR.control_module.controller import Controller
from ROAR.utilities_module.data_structures_models import Transform
from ROAR.utilities_module.vehicle_models import Vehicle
from ROAR.utilities_module.vehicle_models import VehicleControl
from collections import deque
import numpy as np

class SimplePIDController(Controller):
    def __init__(self, agent, distance_to_keep=0.5, center_x=-0.2, **kwargs):
        super().__init__(agent, **kwargs)

        self.yaw_error_buffer = deque(maxlen=20)

        self.lat_error_queue = deque(maxlen=20)  # this is how much error you want to accumulate
        self.long_error_queue = deque(maxlen=50)  # this is how much error you want to accumulate
        self.center_x = center_x
        self.lat_kp = 1  # this is how much you want to steer
        self.lat_kd = 0  # this is how much you want to resist change
        self.lat_ki = 0.005  # this is the correction on past error
        self.x_error_weight = 1
        self.yaw_error_weight = 0.9

        self.distance_to_keep = distance_to_keep
        self.max_throttle = 0.18
        self.lon_kp = 0.17  # this is how much you want to steer
        self.lon_kd = 0.1  # this is how much you want to resist change
        self.lon_ki = 0.025  # this is the correction on past error

    def run_in_series(self, next_waypoint=None, **kwargs) -> VehicleControl:
        control = VehicleControl()
        # self.lateral_pid_control(next_waypoint=next_waypoint, control=control)
        self.long_pid_control(next_waypoint=next_waypoint, control=control)
        print(control)
        return control

    def long_pid_control(self, next_waypoint, control):
        error =
        return