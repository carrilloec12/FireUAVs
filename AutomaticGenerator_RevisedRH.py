"""
Author: Joshua Shaffer
Date: Created - 12/31/2017, Revised 6/8/2018
Purpose: This script generates all of the necessary synthesized controllers for the paper "Receding Horizon Synthesis
         and Dynamic Allocation of UAVs to Fight Fires" written by Joshua Shaffer, Estefany Carrillo, and Huan Xu.
"""

# @import_section@
# Import the packages that we need
from __future__ import print_function
import logging
import time

from tulip import transys, spec, synth, dumpsmach
import csv

# @import_section_end@

# Grab logging information pertaining to tulip
logging.basicConfig(level=logging.WARNING)
logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
logging.getLogger('tulip.synth').setLevel(logging.WARNING)
logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)

# Setup variables pertaining to dimensions of the state space and obstacles
dim_x = 10
dim_y = 10
goal_synthesis_packages = dict()  # Not used currently, may hold all synthesized controllers in memory (eeck..)

# This variable includes all states that are never allowed to occur, including obstacle locations and locations that
# produce infeasible transitions
phi_states = ['Pos2_1Ori1', 'Pos2_1Ori3',
              'Pos3_1Ori1', 'Pos3_1Ori3',
              'Pos4_1Ori1', 'Pos4_1Ori3',
              'Pos5_1Ori1', 'Pos5_1Ori3',
              'Pos6_1Ori1', 'Pos6_1Ori3',
              'Pos7_1Ori1', 'Pos7_1Ori3',
              'Pos8_1Ori1', 'Pos8_1Ori3',
              'Pos9_1Ori1', 'Pos9_1Ori3',
              'Pos10_2Ori2', 'Pos10_2Ori4',
              'Pos10_3Ori2', 'Pos10_3Ori4',
              'Pos10_4Ori2', 'Pos10_4Ori4',
              'Pos10_5Ori2', 'Pos10_5Ori4',
              'Pos10_6Ori2', 'Pos10_6Ori4',
              'Pos10_7Ori2', 'Pos10_7Ori4',
              'Pos10_8Ori2', 'Pos10_8Ori4',
              'Pos10_9Ori2', 'Pos10_9Ori4',
              'Pos9_10Ori1', 'Pos9_10Ori3',
              'Pos8_10Ori1', 'Pos8_10Ori3',
              'Pos7_10Ori1', 'Pos7_10Ori3',
              'Pos6_10Ori1', 'Pos6_10Ori3',
              'Pos5_10Ori1', 'Pos5_10Ori3',
              'Pos4_10Ori1', 'Pos4_10Ori3',
              'Pos3_10Ori1', 'Pos3_10Ori3',
              'Pos2_10Ori1', 'Pos2_10Ori3',
              'Pos1_9Ori2', 'Pos1_9Ori4',
              'Pos1_8Ori2', 'Pos1_8Ori4',
              'Pos1_6Ori2', 'Pos1_6Ori4',
              'Pos1_7Ori2', 'Pos1_7Ori4',
              'Pos1_5Ori2', 'Pos1_5Ori4',
              'Pos1_4Ori2', 'Pos1_4Ori4',
              'Pos1_3Ori2', 'Pos1_3Ori4',
              'Pos1_2Ori2', 'Pos1_2Ori4',
              'Pos7_8Ori1', 'Pos7_8Ori3',
              'Pos1_10Ori1', 'Pos1_10Ori2', 'Pos1_10Ori3', 'Pos1_10Ori4',
              'Pos10_10Ori1', 'Pos10_10Ori2', 'Pos10_0Ori3', 'Pos10_10Ori4',
              'Pos10_1Ori1', 'Pos10_1Ori2', 'Pos10_1Ori3', 'Pos10_1Ori4',
              'Pos1_1Ori1', 'Pos1_1Ori2', 'Pos1_1Ori3', 'Pos1_1Ori4',
              'Pos2_8Ori1', 'Pos2_8Ori2', 'Pos2_8Ori3', 'Pos2_8Ori4',
              'Pos3_8Ori1', 'Pos3_8Ori2', 'Pos3_8Ori3', 'Pos3_8Ori4',
              'Pos3_7Ori1', 'Pos3_7Ori2', 'Pos3_7Ori3', 'Pos3_7Ori4',
              'Pos4_6Ori1', 'Pos4_6Ori2', 'Pos4_6Ori3', 'Pos4_6Ori4',
              'Pos6_7Ori1', 'Pos6_7Ori2', 'Pos6_7Ori3', 'Pos6_7Ori4',
              'Pos6_6Ori1', 'Pos6_6Ori2', 'Pos6_6Ori3', 'Pos6_6Ori4',
              'Pos6_5Ori1', 'Pos6_5Ori2', 'Pos6_5Ori3', 'Pos6_5Ori4',
              'Pos7_6Ori1', 'Pos7_6Ori2', 'Pos7_6Ori3', 'Pos7_6Ori4',
              'Pos7_5Ori1', 'Pos7_5Ori2', 'Pos7_5Ori3', 'Pos7_5Ori4',
              'Pos7_4Ori1', 'Pos7_4Ori2', 'Pos7_4Ori3', 'Pos7_4Ori4',
              'Pos8_4Ori1', 'Pos8_4Ori2', 'Pos8_4Ori3', 'Pos8_4Ori4',
              'Pos8_5Ori1', 'Pos8_5Ori2', 'Pos8_5Ori3', 'Pos8_5Ori4',
              'Pos8_6Ori1', 'Pos8_6Ori2', 'Pos8_6Ori3', 'Pos8_6Ori4',
              'Pos8_7Ori1', 'Pos8_7Ori2', 'Pos8_7Ori3', 'Pos8_7Ori4',
              'Pos2_9Ori1', 'Pos2_9Ori3',
              'Pos2_7Ori1', 'Pos2_7Ori3',
              'Pos2_6Ori1', 'Pos2_6Ori3',
              'Pos5_6Ori2', 'Pos5_6Ori4',
              'Pos7_7Ori1', 'Pos7_7Ori3',
              'Pos9_6Ori2', 'Pos9_6Ori4',
              'Pos9_5Ori2', 'Pos9_5Ori4']

# Add all obstacle locations to list
obstacle_location = list()
obstacle_location.append((2, 8))
obstacle_location.append((3, 8))
obstacle_location.append((3, 7))
obstacle_location.append((4, 6))
obstacle_location.append((6, 7))
obstacle_location.append((6, 6))
obstacle_location.append((6, 5))
obstacle_location.append((7, 6))
obstacle_location.append((7, 5))
obstacle_location.append((7, 4))
obstacle_location.append((8, 4))
obstacle_location.append((8, 5))
obstacle_location.append((8, 6))
obstacle_location.append((8, 7))

'''
Function Name: create_spec_space
Purpose: For the given goal location, generate the finite horizons (w regions) and the corresponding transition regions 
         for each w region. The transition regions include the attached w region and the first layer of locations in the 
         next closest w region. The purpose of the transition region is to provide the synthesizers with goal locations 
         for each horizon (i.e. the synthesizer creates controllers that drive the system into the next horizon).
            Note: each transition region and w region hold information not only on x,y location, but also orientation
'''


def create_spec_space(x_goal_loc, y_goal_loc, w_part, transition_part):
    w_range_lower = 0  # Keeps track of the lower bound on the current w region
    w_range_upper = 0  # Keeps track of the upper bound on the current w region
    trip_counter = 1  # Keeps track of whether or not a w region was created and added to. Used to stop loop
    w_count = 1  # Keeps track of the current region number

    # Generate all of the w regions and transition regions
    while trip_counter == 1:

        # Create lists for current w region and transition region
        w_part_sub = list()
        transition_sub = list()

        trip_counter = 0  # Note: trip_counter will only stay 0 if there are no more w regions to add for this goal

        # Cycle through all x and y values within the current w region and inside such (i.e. all locations inside square
        # formed around outer most possible w region location
        for x in range(-3 + w_range_lower, 3 + 1 + w_range_upper):
            for y in range(-3 + w_range_lower, 3 + 1 + w_range_upper):

                # This if statement is a bit hard to parse. If the addition of the absolute values of the x and y are
                # less than the inner portion of the w region, or greater than the outer edge, or outside of the system
                # bounds, then the current location is a part of the w region and is added to the w_part_sub
                if (False == ((abs(x) + abs(y) <= 3 * (w_count - 1)) or
                              (abs(x) + abs(y) > 3 * w_count) or (x_goal_loc + x > 10) or
                              (x_goal_loc + x < 1) or (y_goal_loc + y > 10) or (y_goal_loc + y < 1))):
                    trip_counter = 1
                    w_part_sub.append((x + x_goal_loc, y + y_goal_loc, 1))
                    w_part_sub.append((x + x_goal_loc, y + y_goal_loc, 2))
                    w_part_sub.append((x + x_goal_loc, y + y_goal_loc, 3))
                    w_part_sub.append((x + x_goal_loc, y + y_goal_loc, 4))

                # This if statement is similar to the above, except that the first statement accounts for the need to
                # include the states within the next horizon closest to the goal. This does not set trip_counter
                if (False == ((abs(x) + abs(y) < 3 * (w_count - 1) - 1) or
                              (abs(x) + abs(y) > 3 * w_count) or (x_goal_loc + x > 10) or
                              (x_goal_loc + x < 1) or (y_goal_loc + y > 10) or (y_goal_loc + y < 1))):
                    transition_sub.append((x + x_goal_loc, y + y_goal_loc, 1))
                    transition_sub.append((x + x_goal_loc, y + y_goal_loc, 2))
                    transition_sub.append((x + x_goal_loc, y + y_goal_loc, 3))
                    transition_sub.append((x + x_goal_loc, y + y_goal_loc, 4))

        # Exit while loop above if trip counter was never reset
        if trip_counter == 0:
            continue

        # Add origin to w region of innermost region
        if w_count == 1:
            w_part_sub.append((x_goal_loc, y_goal_loc, 1))
            w_part_sub.append((x_goal_loc, y_goal_loc, 2))
            w_part_sub.append((x_goal_loc, y_goal_loc, 3))
            w_part_sub.append((x_goal_loc, y_goal_loc, 4))

        # Add the generated w regions and transition regions to the overall set, at the correct w region index
        w_part.append(w_part_sub)
        transition_part.append(transition_sub)

        # Update the w region counters for the next while loop
        w_count = w_count + 1
        w_range_lower = w_range_lower - 3
        w_range_upper = w_range_upper + 3


'''
Function Name: create_w_specs_all_init_cond
Purpose: This function generates all of the synthesized controllers for the inputted w region and given goal location. 
         Controllers are synthesized for each initial condition (i.e. all states within the w region). If an initial 
         condition cannot be used to synthesize a controller, then it is removed from the current w region and added to 
         the next one above.
'''


def create_w_specs_all_init_cond(current_horizon, x_goal_loc, y_goal_loc, w_part,
                                 transition_part, dimension_x, dimension_y):

    # Don't synthesize for corner w regions (i.e. w region that's only a corner) that can't be fulfilled (... so ugly)
    #   Note: Better way of doing this would be to not add corner regions to w_part...
    if (w_part[current_horizon] == [(dimension_x, dimension_y, 1)] or
            w_part[current_horizon] == [(dimension_x, dimension_y, 2)] or
            w_part[current_horizon] == [(dimension_x, dimension_y, 3)] or
            w_part[current_horizon] == [(dimension_x, dimension_y, 4)] or
            w_part[current_horizon] == [(dimension_x, 1, 1)] or
            w_part[current_horizon] == [(dimension_x, 1, 2)] or
            w_part[current_horizon] == [(dimension_x, 1, 3)] or
            w_part[current_horizon] == [(dimension_x, 1, 4)] or
            w_part[current_horizon] == [(1, dimension_y, 1)] or
            w_part[current_horizon] == [(1, dimension_y, 2)] or
            w_part[current_horizon] == [(1, dimension_y, 3)] or
            w_part[current_horizon] == [(1, dimension_y, 4)] or
            w_part[current_horizon] == [(1, 1, 1)] or
            w_part[current_horizon] == [(1, 1, 2)] or
            w_part[current_horizon] == [(1, 1, 3)] or
            w_part[current_horizon] == [(1, 1, 4)]):
        return

    # Create FTS and actions
    sys_auto = transys.FTS()
    sys_auto.sys_actions.add_from({'Stop', 'Go'})

    # Fake AP's to enable all specs related to base and goal position
    sys_auto.states.add_from(['FAKE'])
    sys_auto.atomic_propositions.add_from({'Base', 'GoalPos'})
    sys_auto.states['FAKE']['ap'] |= {'Base', 'GoalPos'}

    # Add all AP's for the transition space, including those tied to 'Base' and 'GoalPos' if present in transition space
    for locat in transition_part[current_horizon]:
        add_ap_sub = 'Pos' + str(locat[0]) + '_' + str(locat[1]) + 'Ori' + str(locat[2])
        sys_auto.states.add_from([add_ap_sub])
        if locat[0] == 2 and locat[1] == 2:  # TODO add some global indicator of the base location
            sys_auto.states[add_ap_sub]['ap'] |= {'Base'}
        if locat[0] == x_goal_loc and locat[1] == y_goal_loc:
            sys_auto.states[add_ap_sub]['ap'] |= {'GoalPos'}

    # Initial condition, empty set
    sys_auto_init = set()

    # NOTE: that current_horizon is 1, 2, 3, ...
    # Create all transitions through brute force method of checking existence of locations in the transition region next
    # to every starting point, dependent on the orientation. This is ran on transition region and inclusion in w region
    # is checked in creating transitions. This is because all w regions are contained in the transition region, and
    # transitions from a w region are different than those form just transition regions
    for locat in transition_part[current_horizon]:
        init = 'Pos' + str(locat[0]) + '_' + str(locat[1]) + 'Ori' + str(locat[2])

        if locat in w_part[current_horizon]:
            sys_auto.transitions.add_comb({init}, {init}, sys_actions="Stop")
            # Transitions allowed if orientation is up
            if locat[2] == 1:
                if (locat[0], locat[1] + 1, 1) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0]) + '_' + str(locat[1] + 1) + 'Ori' + str(1)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] + 1, locat[1] + 1, 2) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] + 1) + '_' + str(locat[1] + 1) + 'Ori' + str(2)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] - 1, locat[1] + 1, 4) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] - 1) + '_' + str(locat[1] + 1) + 'Ori' + str(4)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
            # Transitions allowed if orientation is right
            elif locat[2] == 2:
                if (locat[0] + 1, locat[1], 2) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] + 1) + '_' + str(locat[1]) + 'Ori' + str(2)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] + 1, locat[1] + 1, 1) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] + 1) + '_' + str(locat[1] + 1) + 'Ori' + str(1)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] + 1, locat[1] - 1, 3) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] + 1) + '_' + str(locat[1] - 1) + 'Ori' + str(3)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
            # Transitions allowed if orientation is down
            elif locat[2] == 3:
                if (locat[0], locat[1] - 1, 1) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0]) + '_' + str(locat[1] - 1) + 'Ori' + str(1)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] + 1, locat[1] - 1, 2) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] + 1) + '_' + str(locat[1] - 1) + 'Ori' + str(2)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] - 1, locat[1] - 1, 4) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] - 1) + '_' + str(locat[1] - 1) + 'Ori' + str(4)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
            # Transitions allowed if orientation is left
            elif locat[2] == 4:
                if (locat[0] - 1, locat[1], 4) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] - 1) + '_' + str(locat[1]) + 'Ori' + str(4)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] - 1, locat[1] + 1, 1) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] - 1) + '_' + str(locat[1] + 1) + 'Ori' + str(1)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")
                if (locat[0] - 1, locat[1] - 1, 3) in transition_part[current_horizon]:
                    endloc = 'Pos' + str(locat[0] - 1) + '_' + str(locat[1] - 1) + 'Ori' + str(3)
                    sys_auto.transitions.add_comb({init}, {endloc}, sys_actions="Go")

        # Create possible transitions for initial conditions in only the transition region. Only transitions are
        # staying still, assigned to both action groups. This works because in implementation the controllers will
        # switch to the next horizon here
        else:
            sys_auto.transitions.add_comb({init}, {init}, sys_actions="Stop")
            sys_auto.transitions.add_comb({init}, {init}, sys_actions="Go")

    # Begin generating the specifications!!

    # Create the additional environmental variables, including signal for w region next to goal
    env_auto_vars = {'StopSignal', 'Fire'}
    if current_horizon == 1:
        env_auto_vars |= {'SyncSignal'}

    # Create the environment specifications, including progress related to sync signal next to goal
    env_auto_safe = set()
    env_auto_init = {'!StopSignal', '!Fire'}
    env_auto_prog = {'!StopSignal', '!Fire'}
    if current_horizon == 1:
        env_auto_prog |= {'SyncSignal'}

    # System variables and safety specification empty sets
    sys_auto_vars = set()
    sys_auto_safe = set()

    # Create stop signal requirements # TODO This should be changed to present actions...
    sys_auto_safe |= {'(StopSignal&&!Fire)->(X(sys_actions = "Stop"))'}
    sys_auto_safe |= {'(!(StopSignal&&!Fire))->(X(sys_actions = "Go"))'}

    # Add relevant phi variables to safety (i.e. all phi contained within this region). Never enter phi locations
    phi = ''
    for locat in transition_part[current_horizon]:
        state = 'Pos' + str(locat[0]) + '_' + str(locat[1]) + 'Ori' + str(locat[2])
        for names in phi_states:
            if state == names:
                phi = phi + '!(loc = "' + names + '")&&'
    phi = phi + 'True'
    sys_auto_safe |= {phi}

    # Empty initial progress
    sys_auto_prog = set()
    # Display progress statement to user
    print(('Beginning synthesis of W' + str(current_horizon) + ' for goal (' +
           str(x_goal_loc) + ',' + str(y_goal_loc) + ')!'))

    # Cycle through all initial conditions in this w region for synthesizing controller
    for locat in w_part[current_horizon]:
        locat_sub = locat

        # Create checks to see if current position is viable initial condition
        current_state = 'Pos' + str(locat_sub[0]) + '_' + str(locat_sub[1]) + 'Ori' + str(locat_sub[2])

        # Run through names in phi_states to check if it's included
        indicator = 0
        for names in phi_states:
            if current_state == names:
                indicator = 1

        # If current state passed, then add to initial condition and synthesize. Otherwise, move to next I.C.
        if indicator == 0:
            sys_auto_init = {'(loc = "' + current_state + '")'}
        else:
            continue

        # Create the progress statements and synthesize. Progress statements differ depending on current horizon
        # For horizon W1
        if current_horizon == 1:

            # This section is the 'sync' version of specs, which is just the normal "correct" version
            print('Sync synthesis for ' + current_state)

            # Start timer for synthesis
            start_time = time.time()

            # Spec just states that the sync signal implies goal position is there
            sync_spec = '(SyncSignal)->GoalPos'
            sys_auto_prog = {sync_spec}

            # Create the GR spec for all the generated env and sys specs
            specs_final_sync = spec.GRSpec(env_auto_vars, sys_auto_vars, env_auto_init, sys_auto_init,
                                           env_auto_safe, sys_auto_safe, env_auto_prog, sys_auto_prog)

            # Synthesizer attributes
            specs_final_sync.moore = True
            option = 'omega'
            specs_final_sync.qinit = '\E \A'
            # synthesizer should find initial system values that satisfy
            # `env_init /\ sys_init` and work, for every environment variable
            # initial values that satisfy `env_init`.

            # SYNTHESIZE!!!
            ctrl_final_sync = synth.synthesize(option, specs_final_sync, env=None, sys=sys_auto, ignore_sys_init=True)

            # Failure results in adjustments to the horizon related to current I.C.
            if ctrl_final_sync is None:
                print('Failed to synthesize ' + current_state + ', moving to next w_part and transition_part')
                w_part[current_horizon + 1].append(locat)
                transition_part[current_horizon + 1].append(locat)
                w_part[current_horizon].remove(locat)
                transition_part[current_horizon].remove(locat)

            # Stopwatch and print
            synth_time = time.time() - start_time
            print(synth_time)

            # Write controller to relevant location if it synthesized
            filename = 'ProcessedMoves2/Goal' + str(x_goal_loc) + str(y_goal_loc) + '/G' + str(x_goal_loc) + '_' + \
                       str(y_goal_loc) + current_state + '.py'  #'_W' + str(current_horizon) +
            if ctrl_final_sync is not None:
                dumpsmach.write_python_case(filename, ctrl_final_sync)

        # All other horizons from the first one (>= W2)
        else:
            print('Synthesis for ' + current_state)

            # Start timer for synthesis
            start_time = time.time()

            # Generate progress to any area of transition region that aren't in the w region (inner layer)
            spec_inter = '(('
            for locat2 in transition_part[current_horizon]:
                if locat2 not in w_part[current_horizon]:
                    locations = 'Pos' + str(locat2[0]) + str(locat2[1]) + 'Ori' + str(locat2[2])
                    spec_inter = spec_inter + 'loc = "' + locations + '")||('
            # Finish tail end of progress spec (false is there to wrap up string generated by loop)
            spec_inter = spec_inter + 'False))'
            sys_auto_prog |= {spec_inter}

            # Create the GR spec for all the generated env and sys specs
            specs_inter = spec.GRSpec(env_auto_vars, sys_auto_vars, env_auto_init, sys_auto_init,
                                      env_auto_safe, sys_auto_safe, env_auto_prog, sys_auto_prog)

            # Synthesizer attributes
            specs_inter.moore = True
            option = 'omega'
            specs_inter.qinit = '\E \A'
            # synthesizer should find initial system values that satisfy
            # `env_init /\ sys_init` and work, for every environment variable
            # initial values that satisfy `env_init`.

            # SYNTHESIZE!!!
            ctrl_inter = synth.synthesize(option, specs_inter, sys=sys_auto, ignore_sys_init=True)

            # Failure results in adjustments to the horizon related to current I.C.
            if ctrl_inter is None:
                print('Failed to synthesize ' + current_state + ', moving to next w_part and transition_part')
                w_part[current_horizon + 1].append(locat)
                transition_part[current_horizon + 1].append(locat)
                w_part[current_horizon].remove(locat)
                transition_part[current_horizon].remove(locat)

            # Stopwatch and print
            synth_time = time.time() - start_time
            print(synth_time)

            # Write controller to relevant location if it synthesized
            filename = 'ProcessedMoves2/Goal' + str(x_goal_loc) + str(y_goal_loc) + '/G' + str(x_goal_loc) + '_' + \
                       str(y_goal_loc) + current_state + '.py'  #'_W' + str(current_horizon) +
            if ctrl_inter is not None:
                dumpsmach.write_python_case(filename, ctrl_inter)




'''
Function Name: create_and_synthesize_specs_single_goal
Purpose: Generate finite horizons around a single goal and synthesize the correct controllers for each initial condition
         within each horizon
'''


def create_and_synthesize_specs_single_goal(x_goal_loc, y_goal_loc, dimension_x, dimension_y, obs_loc):
    # These exceptions are used for preventing synthesis around goals where obstacles or edges are
    if x_goal_loc == 1 or x_goal_loc == dimension_x or y_goal_loc == 1 or y_goal_loc == dimension_y:
        return
    if (x_goal_loc, y_goal_loc) in obs_loc:
        return
    # Use this to force continuation from last goal synthesized
    #if x_goal_loc < 5 or y_goal_loc < 8:
    #    return

    # Construct the finite automata for each section surrounding goal,
    # These are at max 4 on each diagonal for the transition space, while
    # the W space consists of 3 beyond each,

    # Create w regions transition space for all w regions around current goal
    w_part = list()
    transition_part = list()
    w_part.append(None)
    transition_part.append(None)
    create_spec_space(x_goal_loc, y_goal_loc, w_part, transition_part)

    # Generate w file
    with open('Goal'+str(x_goal_loc) + '_' + str(y_goal_loc)+'.csv', 'wb') as csvfile:
        writer = csv.writer(csvfile, delimiter = ',')
        # Cycle through all horizons, starting at the first one closest to goal, and synthesize the controllers
        for current_horizon, tran1 in enumerate(transition_part):
            if current_horizon != 0:
                create_w_specs_all_init_cond(current_horizon, x_goal_loc, y_goal_loc,
                                             w_part, transition_part, dimension_x, dimension_y)
                string_ext = list()
                #print(w_part[current_horizon])
                for item in w_part[current_horizon]:
                    str_add = 'Pos' + str(item[0]) + '_' + str(item[1]) + 'Ori' + str(item[2])
                    #input('hello')
                    string_ext.append(str_add)
                    #print(string_ext)
                writer.writerow(string_ext)




'''
Primary program loop
'''

# Main loop, generating all synthesized controllers.
for x_goal in range(1, dim_x + 1):
    for y_goal in range(1, dim_y + 1):
        create_and_synthesize_specs_single_goal(x_goal, y_goal, dim_x, dim_y, obstacle_location)
