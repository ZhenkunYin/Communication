#!/usr/bin/env python

from binhex import binhex
from cgi import print_arguments
from distutils.log import error


class messages__:
    def __init__(self) -> None:
        pass

    Speed_actual={
        'length': 8,
        'name': 0x01,
        'data': 0x00
    }

    Speed_target={
        'length': 8,
        'name': 0x02,
        'data': 0x00
    }

    Steering_angle_actual={
        'length': 8,
        'name': 0x03,
        'data': 0x00
    }

    Steering_angle_target={
        'length': 8,
        'name': 0x04,
        'data': 0x00
    }

    Brake_hydr_actual={
        'length': 8,
        'name': 0x05,
        'data': 0x00
    }

    Brake_hydr_target={
        'length': 8,
        'name': 0x06,
        'data': 0x00
    }

    Motor_moment_actual={
        'length': 8,
        'name': 0x07,
        'data': 0x00
    }

    Motor_moment_target={
        'length': 8,
        'name': 0x08,
        'data': 0x00
    }

    Acceleration_longitudinal={
        'length': 16,
        'name': 0x09,
        'data': 0x0000
    }

    Acceleration_lateral={
        'length': 16,
        'name': 0x0a,
        'data': 0x0000
    }

    Yaw_rate={
        'length': 16,
        'name': 0x0b,
        'data': 0x0000
    }

    AS_state={
        'length': 3,
        'name': 0,
        'data': 0x00
    }

    EBS_state={
        'length': 2,
        'name': 3,
        'data': 0x00
    }

    AMI_state={
        'length': 3,
        'name': 5,
        'data': 0x00
    }

    Steering_state={
        'length': 1,
        'name': 8,
        'data': 0x00
    }

    Service_brake_state={
        'length': 2,
        'name': 9,
        'data': 0x00
    }

    Lap_counter={
        'length': 4,
        'name': 11,
        'data': 0x00
    }

    Cones_count_actual={
        'length': 8,
        'name': 15,
        'data': 0x00
    }

    Cones_count_all={
        'length': 17,
        'name': 23,
        'data': 0x00000
    }

    def get_Speed_actual(self):
        return self.Speed_actual['data']

    def set_Speed_actual(self,data):
        if data > 255 or data < 0:
            error("error from set_Speed_actual: wrong data")
            exit(0)
        self.Speed_actual['data'] = data

    def get_Speed_target(self):
        return self.Speed_target['data']

    def set_Speed_target(self,data):
        if data > 255 or data < 0:
            error("error from set_Speed_target: wrong data")
            exit(0)
        self.Speed_target['data'] = data

    def get_Steering_actual(self):
        return self.Steering_angle_actual['data']

    def set_Steering_actual(self,data):
        if data > 127 or data < -128:
            error("error from set_STeering_actua: wrong data")
            exit(0)
        self.Steering_angle_actual['data'] = data

    def get_Steering_target(self):
        return self.Steering_angle_target['data']

    def set_Steering_target(self,data):
        if data > 127 or data < -128:
            error("error from set_Steering_target: wrong data")
            exit(0)
        self.Steering_angle_target['data'] = data

    def get_Brake_hydr_actual(self):
        return self.Brake_hydr_actual['data']

    def set_Brake_hydr_actual(self,data):
        if data > 255 or data < 0:
            error("error from set_Brake_hydr_actual: wrong data")
            exit(0)
        self.Steering_angle_actual['data'] = data

    def get_Brake_hydr_target(self):
        return self.Brake_hydr_target['data']

    def set_Brake_hydr_target(self,data):
        if data > 255 or data < 0:
            error("error from set_Brake_hydr_target: wrong data")
            exit(0)
        self.Steering_angle_target['data'] = data

    def get_Motor_moment_actual(self):
        return self.Motor_moment_actual['data']

    def set_Motor_moment_actual(self,data):
        if data > 127 or data < -128:
            error("error from set_Motor_moment_actual: wrong data")
            exit(0)
        self.Motor_moment_actual['data'] = data
    
    def get_Motor_moment_target(self):
        return self.Motor_moment_target['data']

    def set_Motor_moment_target(self,data):
        if data > 127 or data < -128:
            error("error from set_Motor_moment_target: wrong data")
            exit(0)
        self.Motor_moment_target['data'] = data

    def get_Acceleration_longitudinal(self):
        return self.Acceleration_longitudinal['data']

    def set_Acceleration_longitudinal(self,data):
        if data > 32767 or data < -32768:
            error("error from set_Acceleration_longitudinal: wrong data")
            exit(0)
        self.Acceleration_longitudinal['data'] = data

    def get_Acceleration_lateral(self):
        return self.Acceleration_lateral['data']

    def set_Acceleration_lateral(self,data):
        if data > 32767 or data < -32768:
            error("error from set_Acceleration_lateral: wrong data")
            exit(0)
        self.Acceleration_lateral['data'] = data

    def get_Yaw_rate(self):
        return self.Yaw_rate['data']

    def set_Yaw_rate(self,data):
        if data > 32767 or data < -32768:
            error("error from set_Yaw_rate: wrong data")
            exit(0)
        self.Yaw_rate['data'] = data

    def get_AS_state(self):
        return self.AS_state['data']

    def set_AS_state(self,data):
        if data > 5 or data < 0:
            error("error from set_AS_state: wrong data")
            exit(0)
        self.AS_state['data'] = data

    def get_EBS_state(self):
        return self.EBS_state['data']

    def set_EBS_state(self,data):
        if data > 3 or data < 0:
            error("error from set_EBS_state: wrong data")
            exit(0)
        self.EBS_state['data'] = data

    def get_AMI_state(self):
        return self.AMI_state['data']

    def set_AMI_state(self,data):
        if data > 6 or data < 0:
            error("error from set_AMI_state: wrong data")
            exit(0)
        self.AMI_state['data'] = data

    def get_Steering_state(self):
        return self.Steering_state['data']

    def set_Steering_state(self,data):
        if data > 1 or data < 0:
            error("error from set_Steering_state: wrong data")
            exit(0)
        self.Steering_state['data'] = data

    def get_Service_brake_state(self):
        return self.Service_brake_state['data']

    def set_Service_brake_state(self,data):
        if data > 3 or data < 0:
            error("error from set_Service_brake_state: wrong data")
            exit(0)
        self.Service_brake_state['data'] = data

    def get_Lap_counter(self):
        return self.Lap_counter['data']

    def set_Lap_counter(self,data):
        if data > 15 or data < 0:
            error("error from set_Lap_counter: wrong data")
            exit(0)
        self.Lap_counter['data'] = data

    def get_Cones_count_actual(self):
        return self.Cones_count_actual['data']

    def set_Cones_count_actual(self,data):
        if data > 255 or data < 0:
            error("error from set_Cones_count_actual: wrong data")
            exit(0)
        self.Cones_count_actual['data'] = data

    def get_Cones_count_all(self):
        return self.Cones_count_all['data']

    def set_Cones_count_all(self,data):
        if data > 131071 or data < 0:
            error("error from set_Cones_count_all: wrong data")
            exit(0)
        self.Cones_count_all['data'] = data

    def get_state(self):
        str_state = ((self.AS_state['data'] & 0x7) << self.AS_state['name']) | ((self.AMI_state['data'] & 0x7) << self.AMI_state['name']) | ((self.EBS_state['data'] & 0x3) << self.EBS_state['name']) | ((self.Steering_state['data'] & 0x1) << self.Steering_state['name']) | ((self.Service_brake_state['data'] & 0x3) << self.Service_brake_state['name']) | ((self.Cones_count_actual['data'] & 0xf) << self.Cones_count_actual['name']) | ((self.Cones_count_all['data'] & 0x1ff) << self.Cones_count_all['name'])
        return str_state

