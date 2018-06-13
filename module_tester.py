'''
For testing results of graph module, not used
'''
from Graph import Graph
import math
from Dynamics import Dynamics

gra = Graph(Dynamics)
gra.generate_graph_from_dynamics([1.0, 1.0, math.pi/2.0], [3, (1., 10., 0), (1., 10., 0), (0., 3.0*math.pi/2.0, 1)],
                                 [[1., 0.], [1.0*math.pi/2.0, math.pi/2.0], [1.0*math.pi/2.0, -math.pi/2.0],
                                  [0.0, 0.0]], 1., 0.1, 0.1)
print(gra)
for nodes in gra.graph:
    print(nodes)
    print(gra.graph[nodes].parents)
    print(gra.graph[nodes].children)
    g = input('Wait...')