from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
import cv2
from collections import deque
from ROAR.utilities_module.udp_multicast_communicator import UDPMulticastCommunicator
from ROAR.control_module.aruco_pid_controller import SimplePIDController


class UDPMultiCastAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle, agent_settings, **kwargs)
        self.udp_multicast = UDPMulticastCommunicator(mcast_group="224.1.1.1",
                                                      mcast_port=5004,
                                                      threaded=True,
                                                      update_interval=0.1)
        self.add_threaded_module(self.udp_multicast)
        # self.target_x
        # self.target_y
        # self.target_z
        self.position = []
        self.throttle = 0
        self.steering = 0
        self.controller = SimplePIDController(agent=self, distance_to_keep=1, center_x=-0.3)


    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super().run_step(sensors_data=sensors_data, vehicle=vehicle)
        self.position = self.vehicle.to_array()[1:4]
        self.udp_multicast.send_msg(f"{'chadwick'},{','.join(map(str, self.vehicle.to_array()))}")
        target = 'Xingyu'
        if self.udp_multicast.data[0] == target:
            print(self.udp_multicast.data)
            target_position = self.udp_multicast.data[1:4]
            self.throttle, self.steering = self.controller(target_position)

            # self.target_x, self.target_y, self.target_z = self.udp_multicast.data[1:4]



        return VehicleControl(throttle=0, steering=0)
