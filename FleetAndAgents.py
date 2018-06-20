'''
These classes contain all individual UAV agents and the entire fleet. Building this very modular so that
update rules for each agent can easily be added in, plus each UAV could have their own individual dynamics.
Still need to add in update rules for the UAVs beyond dynamic update. Need to also decide how each UAV will
hold the top-level synthesized controllers. Also need to translate Estefany's allocation function from Matlab
to python
'''
import os, imp, csv, re, math
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
        self.desired_state = goal
        self.prev_goal = goal
        self.goal_ind = 0
        self.base = 0
        self.region = region
        self.water_level = water_level
        self.water_dropped = 0
        self.ctrler = None
        self.wtr_ctrler = TulipStrategy()
        self.wtr_output = self.wtr_ctrler.move(0, 0, 0)
        self.sync_signal_prev = 1
        self.sync_signal = 1  # Will need to change eventually... (part of a function that observes other UAVs)
        self.pause_time = -pause_interval

    def __str__(self):
        return 'Agent: ' + self.name + ' State: ' + str(self.state)

    def __repr__(self):
        return self.__str__()

    def update_state_truth(self, tau, ctrl, dist, x_override=None):
        if x_override is None:
            self.state_truth = self.dynamic_model.integrate_state(tau, self.state, ctrl, dist)
        else:
            self.state_truth = self.dynamic_model.integrate_state(tau, x_override, ctrl, dist)

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
        return

    def update_water_lev(self, water):
        self.water_level = water
        return

    def display_loc(self, params):
        center = (self.state_truth[0] - 1, params.height - (self.state_truth[1] - 1))
        loc_center_screen = (params.WIDTH * center[0] + params.WIDTH/2, params.HEIGHT * center[1] - params.HEIGHT/2)
        vec1 = (loc_center_screen[0] + params.FRONT_VECTOR[0], loc_center_screen[1] + params.FRONT_VECTOR[1])
        vec2 = (loc_center_screen[0] + params.BACK_BOT_VECT[0], loc_center_screen[1] + params.BACK_BOT_VECT[1])
        vec3 = (loc_center_screen[0] + params.BACK_TOP_VECT[0], loc_center_screen[1] + params.BACK_TOP_VECT[1])
        return [vec1, vec2, vec3]


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
        return

    def allocate(self, env, params):
        # allocation_function(self, env, params)
        return 'uhhhh'

    # Used for management of all controllers attached to the agents
    def update_ctrls(self, time):
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

            # if time hasn't exceeded the original pause time for the agent plus the interval, don't update agent
            if time - self.agents[i].pause_time < params.pause_interval:
                continue

            # gather current location and fire status
            loc = (self.agents[i].state_belief[0], self.agents[i].state_belief[1])
            fire = 1 if env.cells[loc].fire > 0 else 0

            # stop signal logic for updating an agent
            stop_signal = 1 if uniform(0,1) > 1 - params.stop_fail else 0
            self.agents[i].pause_time = time if stop_signal == 1 else -params.pause_interval

            # move synthesized controller given updates on environment (need to modify so that only two inputs used if
            # other controller is used)
            if self.agents[i].region == 1:
                output = self.agents[i].ctrler.move(fire, self.agents[i].sync_signal, stop_signal)
            else:
                output = self.agents[i].ctrler.move(fire, stop_signal)

            # update controller outputs for angle to reflect true values
            values = re.findall('\d+', output["loc"])
            if values[2] == 1:
                values[2] = 0.0
            elif values[2] == 2:
                values[2] = math.pi/2.0
            elif values[2] == 3:
                values[2] = math.pi
            else:
                values[2] = 3.0 * math.pi / 2.0

            # update various belief states and previous states, plus desired states and control inputs
            self.agents[i].prev_state = self.agents[i].state_belief
            self.agents[i].desired_state = (values[0], values[1], values[2])
            # find control inputs from graph...
            self.agents[i].control_inputs = (0.0 , 0.0)

            base = self.agents[i].base
            goal = self.agents[i].goal_ind
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

            # update goal index and base index to agent's belief (done here because we are assuming the UAV makes it,
            # and that the controller move was enacted correctly
            self.agents[i].goal_ind = 1 if (output["GoalPos"] and self.agents[i].sync_signal_prev) else 0
            self.agents[i].base = 1 if output["Base"] else 0

        return

    def update(self, env, params, time_step):

        # Layout:
        for i in self.agents:
            # Propagate state forward for now (no environmental inputs at the moment)
            self.agents[i].state_truth = self.agents[i].dynamic_model.integrate_state(time_step,
                                            self.agents[i].state_truth, self.agents[i].control_inputs, (0.0, 0.0, 0.0))
            # 2. Update the belief of the agent (for this purpose, this is tied directly to the output of the function)
            self.agents[i].belief_state = self.agents[i].state_truth

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


