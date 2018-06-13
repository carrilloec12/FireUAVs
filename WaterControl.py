#!/usr/bin/env python
"""Example using transition system dynamics.

This example illustrates the use of TuLiP to synthesize a reactive
controller for system whose dynamics are described by a discrete
transition system.
"""
# RMM, 20 Jul 2013
#
# Note: This code is commented to allow components to be extracted into
# the tutorial that is part of the users manual.  Comments containing
# strings of the form @label@ are used for this purpose.

# @import_section@
# Import the packages that we need
from __future__ import print_function

import logging

from tulip import transys, spec, synth, dumpsmach
# @import_section_end@


logging.basicConfig(level=logging.WARNING)
logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
logging.getLogger('tulip.synth').setLevel(logging.WARNING)
logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)


#
# System dynamics
#
# The system is modeled as a discrete transition system in which the
# robot can be located anyplace on a 2x3 grid of cells.  Transitions
# between adjacent cells are allowed, which we model as a transition
# system in this example (it would also be possible to do this via a
# formula)
#
# We label the states using the following picture
#
#     +----+----+----+
#     | X3 | X4 | X5 |
#     +----+----+----+
#     | X0 | X1 | X2 |
#     +----+----+----+
#

# @system_dynamics_section@
# Create a finite transition system
sys_water = transys.FTS()
sys_water_vars = set()


# Define the states of the system
sys_water.states.add_from(['Water100','Water60', 'Water20','Water0'])
sys_water.states.initial.add('Water100')    # start in state X0
#sys_water.states.initial.add('Water60')
#sys_water.states.initial.add('Water20')
#sys_water.states.initial.add('Water0')

# Define the allowable transitions

sys_water.transitions.add_comb({'Water100'}, {'Water60','Water100'})
sys_water.transitions.add_comb({'Water60'}, {'Water100', 'Water20','Water60'})
sys_water.transitions.add_comb({'Water20'}, {'Water100', 'Water0','Water20'})
sys_water.transitions.add_comb({'Water0'}, {'Water100', 'Water0'})
# @system_dynamics_section_end@

#
# @environ_section@
env_vars = {'Base','GoalPos','Goal'}
env_init = set()                # empty set
env_prog = {'Base','GoalPos','Goal','!Base','!GoalPos','!Goal'}
env_safe = {'Base->!GoalPos','GoalPos->!Base'}                # empty set
# @environ_section_end@

#sys_vars = #{'X0reach'}          # infer the rest from TS
sys_init = set()#{'X0reach'}
sys_prog = set()#{'home'}             # []<>home
#sys_safe = {'(X (X0reach) <-> lot) || (X0reach && !park)'}
#sys_prog |= {'X0reach'}

sys_water_safe = set()

# Water control requirements

WaterDownSpec = ('((!(loc = "Water0")) && (X(Goal) && X(GoalPos)))&&(!(X(Base)))->(' +
'(!(X(loc = "Water100")))&&(X(loc = "Water60")<->(loc = "Water100"))&&(X(loc = "Water20")<->(loc = "Water60"))'
+ '&&(X(loc = "Water0")<->(loc = "Water20")))')

Home_spec = ('(X(Base))->((!(X(loc = "Water0")))&&((X(loc = "Water100")))'+
'&&(!(X(loc = "Water60")))&&(!(X(loc = "Water20"))))')


WaterKeepSpec = ('(!(X(Goal) && X(GoalPos)))&&(!(X(Base)))->' +
'((X(loc = "Water0")<->(loc = "Water0"))&&'
+'(X(loc = "Water100")<->(loc = "Water100"))&&(X(loc = "Water60")<->(loc = "Water60"))&&(X(loc = "Water20")<->(loc = "Water20")))')
WaterKeepSpec2 = ('(((loc = "Water0") && (X(Goal) && X(GoalPos)))&&(!(X(Base))))->' +
'((X(loc = "Water0")<->(loc = "Water0"))&&'
+'(X(loc = "Water100")<->(loc = "Water100"))&&(X(loc = "Water60")<->(loc = "Water60"))&&(X(loc = "Water20")<->(loc = "Water20")))')

sys_water_safe |= {WaterDownSpec,Home_spec,WaterKeepSpec,WaterKeepSpec2}


# @specs_create_section@
# Create the specification
specs = spec.GRSpec(env_vars, sys_water_vars, env_init, sys_init,
                    env_safe, sys_water_safe, env_prog, sys_prog)
# @specs_create_section_end@

#
# Controller synthesis
#
# At this point we can synthesize the controller using one of the available
# methods.
#
# @synthesize@
# Moore machines
# controller reads `env_vars, sys_vars`, but not next `env_vars` values
specs.moore = False
# synthesizer should find initial system values that satisfy
# `env_init /\ sys_init` and work, for every environment variable
# initial values that satisfy `env_init`.
specs.qinit = '\E \A'
ctrl = synth.synthesize(specs, sys=sys_water)
assert ctrl is not None, 'unrealizable'
# @synthesize_end@

#
# Generate a graphical representation of the controller for viewing,
# or a textual representation if pydot is missing.
#
# @plot_print@
if not ctrl.save('discrete.png'):
    print(ctrl)
# @plot_print_end@
Filename = 'WaterControl_controller.py'
dumpsmach.write_python_case(Filename,ctrl)
