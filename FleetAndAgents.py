'''
These classes contain all individual UAV agents and the entire fleet. Building this very modular so that
update rules for each agent can easily be added in, plus each UAV could have their own individual dynamics.
Still need to add in update rules for the UAVs beyond dynamic update. Need to also decide how each UAV will
hold the top-level synthesized controllers. Also need to translate Estefany's allocation function from Matlab
to python
'''
import os, imp, csv, re
#from Estefany_module import allocation_function
from WaterControl_controller import TulipStrategy
from random import uniform


class Agent(object):
    def __init__(self, state_truth, state_belief, name, dynamics, goal, region, water_level, pause_interval):
        # Initialize state of the agent
        self.state_truth = state_truth
        self.state_belief = state_belief
        self.name = name
        self.dynamic_model = dynamics()
        self.goal = goal
        self.prev_goal = goal
        self.goal_ind = 0
        self.base = 0
        self.region = region
        self.water_level = water_level
        self.water_dropped = 0
        self.ctrler = None
        self.wtr_ctrler = TulipStrategy()
        self.wtr_output = self.wtr_ctrler.move(0, 0, 0)
        self.sync_signal = 1  # Will need to change eventually... (part of a function that observes other UAVs)
        self.pause_time = -pause_interval

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
        # allocation_function(self, env, params)
        return 'uhhhh'

    # Used for management of all controllers attached to the agents
    def update_ctrls(self):
        for i in self.agents:
            trigger1 = True  # Temporary TODO self.agents[i].update_objective_state(somethingfromallocation)
            if trigger1 is not True:
                region = self.region_interpreter(self.agents[i].belief_state, self.agents[i].goal)
                trigger2 = False if region == self.agents[i].region else True
                self.agents[i].update_region(region)
            else:
                self.agents[i].update_region(self.region_interpreter(self.agents[i].belief_state, self.agents[i].goal))
                trigger2 = True

            if trigger1 is True or trigger2 is True:
                hand = self.directory_interpreter(self.agents[i].belief_state, self.agents[i].goal)
                self.agents[i].ctrler = hand.TulipStrategy()
                if self.agents[i].region == 1:
                    self.agents[i].ctrler.move(0, self.agents[i].sync_signal, 0)
                else:
                    self.agents[i].ctrler.move(0, 0)
        return

    def update(self, env, params, time):

        # Layout:
        # 1. Update the controllers (call move functions)
        for i in self.agents:
            # if time hasn't exceeded the original pause time for the agent plus the interval, don't update agent
            if time - self.agents[i].pause_time < params.pause_interval:
                continue

            # gather current location and fire status
            loc = (self.agents[i].state_belief[0], self.agents[i].state_belief[1])
            fire = 1 if env.cells[loc].fire > 0 else 0

            # stop signal logic for updating an agent
            stop_signal = 1 if uniform(0,1) > 1 - params.stop_fail else 0
            self.agents[i].pause_time = time if stop_signal == 1 else -params.pause_interval

            # move synthesized controller given updates on environment
            output = self.agents[i].ctrler.move(fire, self.agents[i].sync_signal, stop_signal)

            values = re.findall('\d+', output["loc"])
            state = (values[0], values[1], values[2])
            base = output["Base"]
            goal = output["GoalPos"]

            # 2. Update the belief of the agent (for this purpose, this is tied directly to the output of the function)
            self.agents[i].belief_state = state
            self.agents[i].goal_ind = 1 if (goal and self.agents[i].sync_signal) else 0
            self.agents[i].base = 1 if base else 0

            # 3. Update the "truth" state (no propagation for now, simply set the value directly for now)
            self.agents[i].truth_state = state

            # 4. Update water controller
            wtr_out_prev = self.agents[i].wtr_output
            self.agents[i].wtr_output = self.agents[i].wtr_ctrler.move(base, agents[i].sync_signal, goal)
            val = re.findall('\d+', self.agents[i].wtr_output["loc"])
            # add water dropped by UAV to the appropriate cell
            if wtr_out_prev["loc"] != self.agents[i].wtr_output["loc"]:
                val2 = re.findall('\d+', wtr_out_prev)
                env.cells[(self.agents[i].state_truth[0], self.agents[i].state_truth[1])].cell_agent_update(
                    int(val[0]) - int(val2[0]))

            self.agents[i].water_level = int(val[0])  # not necessary I think

    # returns the module for accessing the class (use return.myClass())
    def directory_interpreter(self, state, goal):

        file_name = 'G' + str(goal[0]) + '_' + str(goal[1]) + 'Pos' + str(state[0]) \
                    + '_' + str(state[1]) + 'Ori' + str(state[0]) + '.py'
        file_name2 = 'G' + str(goal[0]) + '_' + str(goal[1]) + 'Pos' + str(state[0]) \
                     + '_' + str(state[1]) + 'Ori' + str(state[0]) + 'NB.py'
        top_directory = 'Goal' + str(goal[0]) + '_' + str(goal[1])


        if os.path.exists('ctrls/' + top_directory + '/' + file_name2):
            return imp.load_source('ctrls/' + top_directory + '/' + file_name2)
        else:
            return imp.load_source('ctrls/' + top_directory + '/' + file_name)

    # return region associated with goal
    def region_interpreter(self, state, goal):
        file_name = 'Goal' + str(goal[0]) + '_' + str(goal[1]) + '.csv'
        state_name = 'Pos' + str(state[0]) + '_' + str(state[1]) + 'Ori' + str(state[2])
        with open(file_name, 'rb') as f:
            reader = csv.reader(f)
            listy = list(reader)

        for i in range(0, len(listy)):
            if state_name in listy[i]:
                return i+1

        return None


