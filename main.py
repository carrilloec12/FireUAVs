'''
Author: Joshua Shaffer

Purpose: Driver function for UAV fire sim. Will be retooled to work with the DroneKit on real-time basis

Created: June 5, 2018
Updated: June 13, 2018
'''

import numpy as np
import math
from Parameters import Params
from SimFunctions import simulation_loop
from FleetAndAgents import Agent, Fleet
from FireEnv import Env, Cell
from Dynamics import Dynamics
from Graph import Graph

gra = Graph(Dynamics)
gra.generate_graph_from_dynamics([1.0, 1.0, math.pi/2.0], [3, (1., 10., 0), (1., 10., 0), (0., 3.0*math.pi/2.0, 1)],
                                 [[1., 0.], [1.0*math.pi/2.0, math.pi/2.0], [1.0*math.pi/2.0, -math.pi/2.0],
                                  [0.0, 0.0]], 1., 0.1, 0.1)

# Populate obstacles and starting fire locations (plus intensities)
params = Params()
obstacles = np.zeros((params.width, params.height))
fires = np.zeros((params.width, params.height))

# Obstacles and fires
obs = [(2, 8), (3, 8), (3, 7), (4, 6), (6, 5), (6, 6), (6, 7), (7, 4), (7, 5), (7, 6), (8, 4), (8, 5), (8, 6), (8, 7)]
fires = [(3, 4), (3, 5)]

# Environment holder
env = Env()

# Populate the environment initial conditions
for i in range(1, params.width+1):
    for j in range(1, params.height+1):
        Water_Accum = 0

        if (i, j) in obs:
            Obstacle = 1
            Fuel = 0
        else:
            Obstacle = 0
            Fuel = 6

        if (i, j) in fires:
            Fires = 1
            FireUpdate = 0
        else:
            Fires = 0
            FireUpdate = 0

        env.add_cell((i, j), Obstacle, Fuel, Fires, FireUpdate, Water_Accum)

# State setup
# Starting states
starts = [[9.0, 9.0, 0.0], [9.0, 2.0, math.pi/2.0], [7.0, 1.0, math.pi], [9.0, 9.0, math.pi*3.0/2.0], [2.0, 3.0, 0.0], [2.0, 3.0, 0.0]]
fleet = Fleet(gra)

for i in range(0, params.N):
    fleet.add_agent(Agent(starts[i], starts[i], 'UAV'+str(i+1), Dynamics, [6.0, 8.0, math.pi*3.0/2.0], 1, 100, params.stop_interval))



# Begin simulation
monte_carlo_number = 1  # Number of desired monte carlo simulations
results = dict()

for monte in range(1, monte_carlo_number+1):
    results[monte] = simulation_loop(fleet, env, params, True)









