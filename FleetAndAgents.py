'''
These classes contain all individual UAV agents and the entire fleet. Building this very modular so that
update rules for each agent can easily be added in, plus each UAV could have their own individual dynamics.
Still need to add in update rules for the UAVs beyond dynamic update. Need to also decide how each UAV will
hold the top-level synthesized controllers. Also need to translate Estefany's allocation function from Matlab
to python
'''
import os, imp, csv, re, math, numpy
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
        rot = numpy.array([[math.cos(self.state_truth[2]), -math.sin(self.state_truth[2])],
                           [math.sin(self.state_truth[2]), math.cos(self.state_truth[2])]])
        rot1 = numpy.matmul(rot, params.FRONT_VECTOR)
        rot2 = numpy.matmul(rot, params.BACK_BOT_VECT)
        rot3 = numpy.matmul(rot, params.BACK_TOP_VECT)

        loc_center_screen = (params.WIDTH * center[0] + params.WIDTH/2, params.HEIGHT * center[1] - params.HEIGHT/2)
        vec1 = (loc_center_screen[0] + rot1[0], loc_center_screen[1] - rot1[1])
        vec2 = (loc_center_screen[0] + rot2[0], loc_center_screen[1] - rot2[1])
        vec3 = (loc_center_screen[0] + rot3[0], loc_center_screen[1] - rot3[1])
        return [vec1, vec2, vec3]


class Fleet(object):
    def __init__(self, graph):
        self.agents = {}
        self.graph = graph

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
        for i in self.agents:
            self.agents[i].goal = [6.0, 8.0, math.pi*3.0/2.0]

    # Used for management of all controllers attached to the agents
    def update_ctrls(self, env, time, params):
        for i in self.agents:
            trigger1 = True  # Temporary TODO self.agents[i].update_objective_state(somethingfromallocation)
            if trigger1 is not True:
                region = self.region_interpreter(self.agents[i].state_belief, self.agents[i].goal)
                trigger2 = False if region == self.agents[i].region else True
                self.agents[i].update_region(region)
            else:
                #print(self.agents[i].state_belief)
                self.agents[i].update_region(self.region_interpreter(self.agents[i].state_belief, self.agents[i].goal))
                trigger2 = True

            if trigger1 is True or trigger2 is True:
                print(self.agents[i].state_belief)
                print(self.agents[i].goal)
                hand = self.directory_interpreter(self.agents[i].state_belief, self.agents[i].goal)
                self.agents[i].ctrler = hand.TulipStrategy()
                print(self.agents[i].region)
                if self.agents[i].region == 1:
                    output = self.agents[i].ctrler.move(0, self.agents[i].sync_signal, 0)
                else:
                    output = self.agents[i].ctrler.move(0, 0)
                print(output)
                #print(output["loc"])
            # if time hasn't exceeded the original pause time for the agent plus the interval, don't update agent TODO reimplement
            '''if time - self.agents[i].pause_time < params.stop_interval:
                print('skipped state output below')
                print(time)
                print(self.agents[i].pause_time)
                self.agents[i].control_inputs = (0.0, 0.0)
                continue'''

            # gather current location and fire status
            loc = (round(self.agents[i].state_belief[0]), round(self.agents[i].state_belief[1]))
            fire = 1 if env.cells[loc].fire > 0 else 0

            # stop signal logic for updating an agent TODO fix stop signal variable
            stop_signal = 0 # if uniform(0,1) > 1 - params.stop_fail else 0
            self.agents[i].pause_time = time if stop_signal == 1 else -params.stop_interval

            # move synthesized controller given updates on environment (need to modify so that only two inputs used if
            # other controller is used)
            if self.agents[i].region == 1:
                output = self.agents[i].ctrler.move(fire, self.agents[i].sync_signal, stop_signal)
            else:
                output = self.agents[i].ctrler.move(fire, stop_signal)
            print(output)
            #print(output["loc"])
            # update controller outputs for angle to reflect true values
            values = re.findall('\d+', output["loc"])

            if int(values[2]) == 1:
                values[2] = math.pi/2.0
            elif int(values[2]) == 2:
                values[2] = 0.0
            elif int(values[2]) == 3:
                values[2] = 3.0 * math.pi / 2.0
            else:
                values[2] = math.pi

            # update various belief states and previous states, plus desired states and control inputs
            self.agents[i].prev_state = self.agents[i].state_belief
            self.agents[i].desired_state = (float(values[0]), float(values[1]), values[2])
            print self.agents[i].prev_state
            print self.agents[i].desired_state

            # find control inputs from graph... TODO: FIX THIS PORTION
            for n in self.graph.graph:
                #print(n)
                #print(self.agents[i].prev_state)
                if (abs(self.agents[i].prev_state[0] - n[0]) < 0.00001 and
                    abs(self.agents[i].prev_state[1] - n[1]) < 0.00001 and
                    abs(self.agents[i].prev_state[2] - n[2]) < 0.00001):
                    parent_node = n
                    break
            for n in self.graph.graph[parent_node].children:
                #print n
                #print self.agents[i].desired_state
                if (abs(self.agents[i].desired_state[0] - n[0]) < 0.00001 and
                    abs(self.agents[i].desired_state[1] - n[1]) < 0.00001 and
                    abs(self.agents[i].desired_state[2] - n[2]) < 0.00001):
                    control_in = self.graph.graph[parent_node].children[n][0]
                    #print(control_in)
            self.agents[i].control_inputs = (control_in[0], control_in[1])
            #print(self.agents[i].control_inputs)

            base = self.agents[i].base
            goal = self.agents[i].goal_ind
            # 4. Update water controller
            wtr_out_prev = self.agents[i].wtr_output
            self.agents[i].wtr_output = self.agents[i].wtr_ctrler.move(base, self.agents[i].sync_signal, goal)
            val = re.findall('\d+', self.agents[i].wtr_output["loc"])
            # add water dropped by UAV to the appropriate cell
            if wtr_out_prev["loc"] != self.agents[i].wtr_output["loc"]:
                val2 = re.findall('\d+', wtr_out_prev["loc"])
                env.cells[(round(self.agents[i].state_truth[0]), round(self.agents[i].state_truth[1]))].cell_agent_update(
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
            if self.agents[i].state_truth[2] < 0.0:
                state2 = math.fmod(self.agents[i].state_truth[2], 2.0 * math.pi)
                state2 = 2.0 * math.pi + state2
            else:
                state2 = math.fmod(self.agents[i].state_truth[2], 2.0 * math.pi)

            self.agents[i].state_belief = [self.agents[i].state_truth[0], self.agents[i].state_truth[1], state2 ]

    # returns the module for accessing the class (use return.myClass())
    def directory_interpreter(self, state, goal):
        if state[2] < 0.0:
            state2 = math.fmod(state[2], 2.0 * math.pi)
            state2 = 2.0 * math.pi + state2
        else:
            state2 = math.fmod(state[2], 2.0 * math.pi)

        if abs(state2) < 0.000001:
            ori = '2'
        elif abs(state2 - math.pi/2.0) < 0.000001:
            ori = '1'
        elif abs(state2 - math.pi) < 0.000001:
            ori = '4'
        else:
            ori = '3'

        file_name = 'G' + str(int(round(goal[0]))) + '_' + str(int(round(goal[1]))) + 'Pos' + str(int(round(state[0])))\
                    + '_' + str(int(round(state[1]))) + 'Ori' + ori + '.py'
        file_name2 = 'G' + str(int(round(goal[0]))) + '_' + str(int(round(goal[1]))) + 'Pos' + str(int(round(state[0])))\
                     + '_' + str(int(round(state[1]))) + 'Ori' + ori + 'NB.py'
        top_directory = 'Goal' + str(int(round(goal[0]))) + '_' + str(int(round(goal[1])))

        #print('ctrls/' + top_directory + '/' + file_name)
        #print(os.path.exists('ctrls/' + top_directory + '/' + file_name))
        if os.path.exists('ctrls/' + top_directory + '/' + file_name2):
            return imp.load_source('TulipStrategy', 'ctrls/' + top_directory + '/' + file_name2)
        else:
            return imp.load_source('TulipStrategy', 'ctrls/' + top_directory + '/' + file_name)

    # return region associated with goal
    def region_interpreter(self, state, goal):
        if state[2] < 0.0:
            state2 = math.fmod(state[2], 2.0 * math.pi)
            state2 = 2.0 * math.pi + state2
        else:
            state2 = math.fmod(state[2], 2.0 * math.pi)

        if abs(state2) < 0.000001:
            ori = '2'
        elif abs(state2 - math.pi/2.0) < 0.000001:
            ori = '1'
        elif abs(state2 - math.pi) < 0.000001:
            ori = '4'
        else:
            ori = '3'

        file_name = 'Goal' + str(int(round(goal[0]))) + '_' + str(int(round(goal[1]))) + '.csv'
        #print(file_name)
        state_name = 'Pos' + str(int(round(state[0]))) + '_' + str(int(round(state[1]))) + 'Ori' + ori
        #print(state_name)
        with open(file_name, 'rb') as f:
            reader = csv.reader(f)
            listy = list(reader)

        for i in range(0, len(listy)):
            if state_name in listy[i]:
                return i+1

        return None


