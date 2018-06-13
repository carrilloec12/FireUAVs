'''
Not specifically relevant to the UAV fire simulation, but this exists for creating graphs based on dynamics
and partition schemes under allowed controls. Can be used as sort of validation tool for transitions used, or
used for creating transition system in controller synthesis
'''

import numpy as np
import math
#from Dynamics import Dynamics


class Node(object):
    def __init__(self, id):
        self.id = id
        # dictionary of parent node ID's
        # key = id of parent
        # value = (edge cost,)
        self.parents = {}
        # dictionary of children node ID's
        # key = id of child
        # value = (edge cost,)
        self.children = {}
        # g approximation
        self.g = float('inf')
        # rhs value
        self.rhs = float('inf')

    def __str__(self):
        return 'Node: ' + str(self.id) + ' g: ' + str(self.g) + ' rhs: ' + str(self.rhs)

    def __repr__(self):
        return self.__str__()

    def update_parents(self, id, edge, ctr):
        self.parents[id] = [ctr, edge]

    def update_children(self, id, edge, ctr):
        self.children[id] = [ctr, edge]


class Graph(object):
    def __init__(self, dynamics):
        self.graph = {}
        self.dyn = dynamics()

    def __str__(self):
        msg = 'Graph:'
        for i in self.graph:
            msg += '\n  node: ' + str(i) + ' g: ' + \
                   str(self.graph[i].g) + ' rhs: ' + str(self.graph[i].rhs)
        return msg

    def __repr__(self):
        return self.__str__()

    # MAJOR TODO: Must redo how boundaries are set up, so that endpoints are not the NODE locations (this stems from how
    #       we recursively set up the nodes
    def generate_graph_from_dynamics(self, res, sta_range, ctr_range, tau, eta_1, eta_2):

        # sta_range is setup as [int=# of states, (start_range1, end_range1, 0 or 1 for real vs radial), ...]
        # resolutions is setup as (res1, res2, res3...)
        # ctr_range is [[ctr1] [ctr2] [ctr3]...] NOTE: for now this is manual, do divisions yet
        # tau is time step (fixed)
        # eta_1 and eta_2 are robustness parameters

        def modulate_radians(val):
            # Always assume 0 - 2pi
            return math.fmod(val, 2 * math.pi)

        def beta_function(eta1, tau1):
            return tau1 * eta1

        '''
        def ball_check_bounds(state_range, s_prime, bound):
            truth = 1

            for i in range(len(state_range) - 1):

                if state_range[i + 1][2] == 1:
                    continue
                if s_prime[i] < state_range[i + 1][0] or s_prime[i] > state_range[i + 1][1]:
                    truth = 0
                    break

                if abs(s_prime[i] - state_range[i + 1][0]) < abs(s_prime[i] - state_range[i + 1][1]):
                    boundary = state_range[i + 1][0]
                else:
                    boundary = state_range[i + 1][1]

                if abs(s_prime[i] - boundary) < bound:
                    truth = 0
                    break

            return truth'''

        def recursive(graph, resolutions, state_range, current, max, r):

            if current < max:
                for i in np.arange(state_range[current][0], state_range[current][1] + resolutions[current - 1],
                                   resolutions[current - 1]):
                    recursive(graph, resolutions, state_range, current + 1, max, r + (i,))
            elif current == max:
                for i in np.arange(state_range[current][0], state_range[current][1] + resolutions[current - 1],
                                   resolutions[current - 1]):
                    add_node_to_graph(graph, r + (i,),
                                      None)  # Not sure if I have to assign this to self.graph (don't believe so now)

        recursive(self.graph, res, sta_range, 1, sta_range[0], ())

        # will want to clean up this section with some function definitions at some point...
        #dyn = Dynamics()
        for nodes in self.graph:

            x_i = [nodes[i] for i in range(0, sta_range[0])]
            dist = [0 for i in range(0, sta_range[0])]

            for controls in ctr_range:

                x_f = self.dyn.integrate_state(tau, x_i, controls, dist)

                for nodes2 in self.graph:

                    sqr = [0.0]
                    for l in range(0, sta_range[0]):

                        if sta_range[l + 1][2] != 1:
                            sqr = sqr + (x_f[l] - nodes2[l]) * (x_f[l] - nodes2[l])
                        else:
                            opt3 = (modulate_radians(x_f[l]) - nodes2[l]) * (modulate_radians(x_f[l]) - nodes2[l])
                            opt4 = (modulate_radians(x_f[l]) - (nodes2[l]+2*math.pi)) * (
                                    modulate_radians(x_f[l]) - (nodes2[l]+2*math.pi))
                            opt5 = (modulate_radians(x_f[l]) - (nodes2[l]-2*math.pi)) * (
                                    modulate_radians(x_f[l]) - (nodes2[l]-2*math.pi))
                            choice = min(opt3, opt4, opt5)
                            sqr = sqr + (choice)

                    sqr = math.sqrt(sqr)

                    if sqr <= (beta_function(eta_1, tau) + eta_2):
                        self.graph[nodes].update_children(nodes2, 1, controls)
                        self.graph[nodes2].update_parents(nodes, 1, controls)
                        break


def add_node_to_graph(graph, id, neighbors, edge=1):
    node = Node(id)
    if neighbors is not None:
        for i in neighbors:
            # print(i)
            node.parents[i] = edge
            node.children[i] = edge

    graph[id] = node
    return graph
