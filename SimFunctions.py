'''
Primary simulation loop and interpreter functions for identifying where a UAV is in relation to synthesized
controllers (will want to move the latter to different module)
'''
import pygame
import math


# These following two functions are HIGHLY dependent on the format of state for this simulation and the files they call


def simulation_loop(fleet, env, Params, visualize=False):

    # Setup visuals
    if visualize:
        pygame.init()
        window_size = [(Params.WIDTH * Params.width),
               (Params.HEIGHT) * Params.height]
        screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("UAVs on Fire")
        # Used to manage how fast the screen updates
        clock = pygame.time.Clock()

    t = 0.0
    continue_sim = True
    while t <= Params.sim_time:

        if visualize:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    continue_sim = not continue_sim

            for row in range(Params.height):
                for column in range(Params.width):
                    if env.cells[(column+1, Params.width - (row))].fire > 0:
                        color = Params.fire_color[env.cells[(column+1, Params.width - (row))].fire-1]
                    elif env.cells[(column+1, Params.width - (row))].obstacle == 1:
                        color = Params.obs_color
                    else:
                        color = Params.fuel_color[env.cells[((column+1), Params.width - (row))].fuel]

                    pygame.draw.rect(screen, color,
                             [(Params.WIDTH) * column,
                              (Params.HEIGHT) * (row), Params.WIDTH,
                              Params.HEIGHT])
                    pygame.draw.rect(screen, (0, 0, 0),
                             [(Params.WIDTH) * column,
                              (Params.HEIGHT) * (row), Params.WIDTH,
                              Params.HEIGHT], Params.MARGIN_HALF)

            # TODO rewrite this to take in position and orientation of an agent, then display based on such by drawing triangle centered on position and oriented by shown value
            for i in fleet.agents:
                #pygame.draw.circle(screen, (94, 154, 249), fleet.agents[i].display_loc(Params), 10)
                pygame.draw.polygon(screen, (94, 154, 249), fleet.agents[i].display_loc(Params))


            # Insert visualization update here
            # Limit to 60 frames per second
            clock.tick(20)

            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

        #continue_sim = False  # debugger code TODO remove
        r = 2
        if continue_sim:
            #print('Current time:')
            #print(t)
            tar = divmod(t, Params.update_step)
            #print('ctr update')
            #print(tar)
            #print(math.fabs(tar[1]) < 0.1 * Params.sim_throttle)
            if math.fabs(tar[1]) < 0.1 * Params.sim_throttle or tar[1] == 0.0 or math.fabs(tar[1] - Params.update_step) < 0.1 * Params.sim_throttle:
                for i in fleet.agents:
                    print(fleet.agents[i].state_belief)
                input('Stop...')

                fleet.allocate(env, Params)
                fleet.update_ctrls(env, t, Params)
                env.update_cells(Params)
                env.update_cells_agent_action(Params, fleet)

                input('slow down man')

            tar = divmod(t, Params.time_step)
            #print('Prop update')
            #print(tar)
            #print(math.fabs(tar[1]) < 0.1 * Params.sim_throttle or tar[1] == 0.0)
            if math.fabs(tar[1]) >= 0.1 * Params.sim_throttle and tar[1] != 0.0 and math.fabs(tar[1] - Params.time_step) >= 0.1 * Params.sim_throttle:
                input('SKIPPED SOME INTEGRATION>>>')
            if math.fabs(tar[1]) < 0.1 * Params.sim_throttle or tar[1] == 0.0 or math.fabs(tar[1] - Params.time_step) < 0.1 * Params.sim_throttle:
                fleet.update(env, Params, Params.time_step)
                #if r == 1:
                for i in fleet.agents:
                    print(fleet.agents[i].state_belief)
                input('wait...')

            t = t + Params.sim_throttle
            print(t)
    return 'No results yet bud'