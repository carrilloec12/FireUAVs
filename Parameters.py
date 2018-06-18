'''
Class holding all static parameters used in simulation. Shoved them in here to condense everything
'''
import numpy as np


class Params(object):

    def __init__(self):
        self.N = 1  # Number of UAVs
        self.mode_version = 'NonSync'
        self.width = 10
        self.height = 10
        self.partition_size = 45  # meters
        self.base_location = (2, 2)
        self.Dc_coefficient = 0.3
        self.Cs_coefficient = 1 - self.Dc_coefficient
        self.switch_penalty = 10
        self.min_fuel_lim = 300
        self.UAV_speed = 15  # m/s
        self.max_water_capacity = 50  # L
        self.ext_vol = np.array([1.2, 0.95, 0.7])  # L
        self.ext_vol = self.ext_vol * self.partition_size * self.partition_size / 4.5
        self.burn_times = np.array([1 / 0.045, 1 / 0.040, 1 / 0.035])  # L
        self.burn_times = self.burn_times * self.partition_size
        self.stop_fail = 0.5
        self.time_step = self.partition_size / self.UAV_speed  # temporary for paper purposes
        self.sim_time = 3000
        self.stop_interval = self.time_step*5

        # Parameters related to visualization
        self.WIDTH = 40
        self.HEIGHT = 40
        self.MARGIN = 5
        self.fire_color = [(255, 0, 0), (255, 0, 0), (255, 0, 0)]
        self.obs_color = (145, 145, 102)
        self.fuel_color = [(0, 0, 0)]  # need to choose fuel colors...

    def __str__(self):
        return 'Params...'

    def __repr__(self):
        return self.__str__()
