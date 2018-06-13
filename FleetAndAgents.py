'''
These classes contain all individual UAV agents and the entire fleet. Building this very modular so that
update rules for each agent can easily be added in, plus each UAV could have their own individual dynamics.
Still need to add in update rules for the UAVs beyond dynamic update. Need to also decide how each UAV will
hold the top-level synthesized controllers. Also need to translate Estefany's allocation function from Matlab
to python
'''
from Estefany_module import allocation_function

class Agent(object):
    def __init__(self, state_truth, state_belief, name, dynamics, goal, region):
        # Initialize state of the agent
        self.state_truth = state_truth
        self.state_belief = state_belief
        self.name = name
        self.dynamic_model = dynamics()
        self.goal = goal
        self.region = region

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
        self.goal = loc
        
    def update_region(self, reg):
        self.region = reg


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
        return 'uhhh'  # TODO incorporate the simulation functions to update the goal location and higher level
                       # agent state controllers

    def update(self, env, params):
        return 'uhhh'  # TODO update plant models of the UAVs and their actions
                       # I added in 'env' for future environmental feedback options
