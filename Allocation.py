import math
import numpy as np

def allocation_function(params, env, fleet):
    fires = list()
    for i in env.cells:
        fires.append([i, env.cells[i].fire])

    locs = list()
    for i in fleet.agents:
        locs.append(fleet.agents[i].belief_state)

    Nassign = 0

    if fires is None:
        return

    maxk = math.ceil((len(fires)+1)/2)

    numClusters, centroids, F = generate_clusters(1, params.width, 1, params.height, fires, maxk, 1)

    Nc = np.zeros((numClusters,3))
    for i in range(0, numClusters):
        cluster_ind = find(F(:,1)==i)
        Nc(i,1) = i
        Nc(i,2) = sum(F(cluster_ind,4))

    np.sort(Nc,1,)
    tot_priority = sum()


def generate_clusters()