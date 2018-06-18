'''
Primary simulation loop and interpreter functions for identifying where a UAV is in relation to synthesized
controllers (will want to move the latter to different module)
'''
import pygame


# These following two functions are HIGHLY dependent on the format of state for this simulation and the files they call


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

            for row in range(Params.width):
                for column in range(Params.height):
                    if env.cells[(row, column)].fire > 0:
                        color = Params.fire_color[env.cells[(row, column)].fire]
                    elif env.cells[(row, column)].obstacle == 1:
                        color = Params.obs_color
                    else:
                        color = Params.fuel_color[env.cells[(row, column)].fuel]

                    pygame.draw.rect(screen, color,
                             [(Params.MARGIN + Params.WIDTH) * column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN, Params.WIDTH,
                              Params.HEIGHT])

                    for i in fleet.agents:
                        loc = (fleet.agents[i].state_truth[0], fleet.agents[i].state_truth[1])
                        if fleet.agents[i].state_truth[1] == 1:
                            vert = [((Params.MARGIN + Params.WIDTH) * column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column/2 + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + row + Params.MARGIN)]
                        elif fleet.agents[i].state_truth[1] == 2:
                            vert = [((Params.MARGIN + Params.WIDTH) * column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column/2 + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + row + Params.MARGIN)]
                        elif fleet.agents[i].state_truth[1] == 3:
                            vert = [((Params.MARGIN + Params.WIDTH) * column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column/2 + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + row + Params.MARGIN)]
                        else:
                            vert = [((Params.MARGIN + Params.WIDTH) * column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + Params.MARGIN),
                              ((Params.MARGIN + Params.WIDTH) * (column) + column/2 + Params.MARGIN,
                              (Params.MARGIN + Params.HEIGHT) * row + row + Params.MARGIN)]

                        pygame.draw.polygon(screen, (0, 0, 80), vert)

                        # need to add numbers too...

                    node_name = 'x' + str(column) + 'y' + str(row)
            # Insert visualization update here

        continue_sim = False  # debugger code TODO remove

        if continue_sim:

            if divmod(t, Params.time_step) == 0:
                fleet.allocate(env, Params)
                fleet.update_ctrls(Params)

            env.update_cells(Params)
            env.update_cells_agent_action(Params, fleet)

            fleet.update(env, Params)

            t = t + Params.time_step
