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
            # Insert visualization update here

        if continue_sim:

            if divmod(t, Params.time_step) == 0:
                fleet.allocate(env, Params)
                fleet.update_ctrls(Params)

            env.update_cells(Params)
            env.update_cells_agent_action(Params, fleet)

            fleet.update(env, Params)

            t = t + Params.time_step
