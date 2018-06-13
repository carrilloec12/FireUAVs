'''
Primary simulation loop and interpreter functions for identifying where a UAV is in relation to synthesized
controllers (will want to move the latter to different module)
'''
import os
import imp
import csv
import pygame


# These following two functions are HIGHLY dependent on the format of state for this simulation and the files they call
def directory_interpreter(state, goal):

    file_name = 'G' + str(goal[0]) + str(goal[1]) + 'Pos' + str(state[0])\
                + str(state[1]) + 'Ori' + str(state[0]) + '.py'
    file_name2 = 'G' + str(goal[0]) + str(goal[1]) + 'Pos' + str(state[0])\
                + str(state[1]) + 'Ori' + str(state[0]) + 'NB.py'
    top_directory = 'Goal' + str(goal[0]) + str(goal[1])


    if os.path.exists('ctrls/' + top_directory + '/' + file_name2):
        return imp.load_source('ctrls/' + top_directory + '/' + file_name2)
    else:
        return imp.load_source('ctrls/' + top_directory + '/' + file_name)


def region_interpreter(state, goal):
    file_name = 'Goal' + str(goal[0]) + str(goal[1]) + '.csv'
    state_name = 'Pos' + str(state[0]) + str(state[1]) + 'Ori' + str(state[2])
    with open(file_name, 'rb') as f:
        reader = csv.reader(f)
        listy = list(reader)

    for i in range(0, len(listy)):
        if state_name in listy[i]:
            return i+1

    return None


def simulation_loop(fleet, env, Params, visualize=False):

    # Setup visuals
    if visualize:
        pygame.init()
        window_size = [(Params.WIDTH + Params.MARGIN) * Params.width + Params.MARGIN,
               (Params.HEIGHT + Params.MARGIN) * Params.height + Params.MARGIN]
        screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("UAVs on Fire")

    t = 0.0
    continue_sim = True
    while t <= Params.sim_time:

        if visualize:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    continue_sim = not continue_sim
            # Insert visualization update here

        if continue_sim:

            if divmod(t, Params.time_step) == 0:
                fleet.allocate(env, Params)
                fleet.update_ctrls(Params)

            env.update_cells(Params)
            env.update_cells_agent_action(Params, fleet)

            fleet.update(env, Params)

            t = t + Params.time_step
