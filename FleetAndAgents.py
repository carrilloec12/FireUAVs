'''
These classes contain all individual UAV agents and the entire fleet. Building this very modular so that
update rules for each agent can easily be added in, plus each UAV could have their own individual dynamics.
Still need to add in update rules for the UAVs beyond dynamic update. Need to also decide how each UAV will
hold the top-level synthesized controllers. Also need to translate Estefany's allocation function from Matlab
to python
'''
import os
import imp
import csv
from Estefany_module import allocation_function
from WaterControl_controller import TulipStrategy

class Agent(object):
    def __init__(self, state_truth, state_belief, name, dynamics, goal, region, water_level):
        # Initialize state of the agent
        self.state_truth = state_truth
        self.state_belief = state_belief
        self.name = name
        self.dynamic_model = dynamics()
        self.goal = goal
        self.prev_goal = goal
        self.region = region
        self.water_level = water_level
        self.ctrler = None
        self.wtr_ctrler = TulipStrategy()
        self.wtr_ctrler.move(0, 0, 0)
        self.syncSignal = 1  # Will need to change eventually... (part of a function that observes other UAVs

    def __str__(self):
        return 'Agent: ' + self.name + ' State: ' + str(self.state)

    def __repr__(self):
        return self.__str__()

    def update_state_truth(self, tau, ctrl, dist, x_override=None):
        if x_override is None:
            self.state = self.dynamic_model.integrate_state(tau, self.state, ctrl, dist)
        else:
            self.state = self.dynamic_model.integrate_state(tau, x_override, ctrl, dist)

    def update_state_belief(self, state):
        self.state_belief = state

    def sensed_information(self):
        return 'I see nothing for now...'

    def update_objective_state(self, loc):
        self.prev_goal = self.goal
        self.goal = loc
        return True if self.prev_goal != self.goal else False
        
    def update_region(self, reg):
        self.region = reg

    def update_water_lev(self, water):
        self.water_level = water


class Fleet(object):
    def __init__(self):
        self.agents = {}

    def __str__(self):
        msg = 'Agents:'
        for i in self.agents:
            msg += '\n  Agent: ' + str(i)
        return msg

    def __repr__(self):
        return self.__str__()

    def add_agent(self, agent):
        self.agents[agent.name] = agent

    def allocate(self, env, params):
        allocation_function(self, env, params)

    def update_ctrls(self, params):
        for i in self.agents:
            trigger1 = self.agents[i].update_objective_state(somethingfromallocation)
            if trigger1 is not True:
                region = self.region_interpreter(self.agents[i].belief_state, self.agents[i].goal)
                trigger2 = False if region == self.agents[i].region else True
                self.agents[i].region = region
            else:
                self.agents[i].region = self.region_interpreter(self.agents[i].belief_state, self.agents[i].goal)
                trigger2 = True

            if trigger1 is True or trigger2 is True:
                hand = self.directory_interpreter(self.agents[i].belief_state, self.agents[i].goal)
                self.agents[i].ctrler = hand.TulipStrategy()
                if self.agents[i].region == 1:
                    self.agents[i].ctrler.move(0, self.agents[i].syncSignal, 0)
                else:
                    self.agents[i].ctrler.move(0, 0)









        return 'uhhh'  # TODO incorporate the simulation functions to update the goal location and higher level
                       # agent state controllers

    def update(self, env, params):
        return 'uhhh'  # TODO update plant models of the UAVs and their actions
                       # I added in 'env' for future environmental feedback options

    # returns the module for accessing the class (use return.myClass())
    def directory_interpreter(self, state, goal):

        file_name = 'G' + str(goal[0]) + str(goal[1]) + 'Pos' + str(state[0])\
                    + str(state[1]) + 'Ori' + str(state[0]) + '.py'
        file_name2 = 'G' + str(goal[0]) + str(goal[1]) + 'Pos' + str(state[0])\
                    + str(state[1]) + 'Ori' + str(state[0]) + 'NB.py'
        top_directory = 'Goal' + str(goal[0]) + str(goal[1])


        if os.path.exists('ctrls/' + top_directory + '/' + file_name2):
            return imp.load_source('ctrls/' + top_directory + '/' + file_name2)
        else:
            return imp.load_source('ctrls/' + top_directory + '/' + file_name)

    # return region associated with goal
    def region_interpreter(self, state, goal):
        file_name = 'Goal' + str(goal[0]) + str(goal[1]) + '.csv'
        state_name = 'Pos' + str(state[0]) + str(state[1]) + 'Ori' + str(state[2])
        with open(file_name, 'rb') as f:
            reader = csv.reader(f)
            listy = list(reader)

        for i in range(0, len(listy)):
            if state_name in listy[i]:
                return i+1

        return None