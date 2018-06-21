'''
Class holding all static parameters used in simulation. Shoved them in here to condense everything
'''
import numpy as np
import math

class Params(object):

    def __init__(self):
        self.N = 4  # Number of UAVs
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
        self.burn_times = np.array([1, 1, 1])   #np.array([1 / 0.045, 1 / 0.040, 1 / 0.035])  # L
        self.burn_times = self.burn_times * 10.0  #self.burn_times = self.burn_times * self.partition_size
        self.stop_fail = 0.5
        self.update_step = 1.0 #self.partition_size / self.UAV_speed  # temporary for paper purposes
        self.time_step = self.update_step / 10.0
        self.sim_time = 3000
        self.stop_interval = self.update_step*50
        self.max_fire_intensity = 3
        self.fire_update_times = [0.0, self.burn_times[0], self.burn_times[1], self.burn_times[2]]

        # Parameters related to visualization
        self.WIDTH = 40
        self.HEIGHT = 40
        self.MARGIN = 2
        self.MARGIN_HALF = 1
        self.TRI_BASE = 20
        self.TRI_HEIGHT = 30
        self.TRI_BACK_ANG = -math.pi + math.atan(self.TRI_BASE/self.TRI_HEIGHT)
        self.FRONT_VECTOR = [self.TRI_HEIGHT/2, 0]
        self.BACK_BOT_VECT = [-self.TRI_HEIGHT/2, -self.TRI_BASE/2]
        self.BACK_TOP_VECT = [-self.TRI_HEIGHT/2, self.TRI_BASE/2]
        self.fire_color = [(255, 238, 61), (255, 170, 61), (252, 61, 27)]
        self.obs_color = (130, 130, 130)
        self.fuel_color = [(3, 25, 6), (12, 106, 26), (16, 140, 35), (16, 190, 35), (16, 255, 35), (81, 255, 101),
                           (165, 255, 176)]  # need to choose fuel colors...

        # Parameters related to simulation
        self.sim_throttle = 0.1

    def __str__(self):
        return 'Params...'

    def __repr__(self):
        return self.__str__()
