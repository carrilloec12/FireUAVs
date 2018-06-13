'''
These two classes contain the environment as a whole and all individual cells in the environment. The update rules
regarding the cells still need to be added from Matlab code
'''


class Env(object):
    def __init__(self):
        self.cells = {}

    def add_cell(self, key, *args):
        self.cells[key] = Cell(*args)

    def update_cells(self, params):
        for i in self.cells:
            self.cells[i].cell_env_update(params)

    def update_cells_agent_action(self, params, fleet):
        for i in self.cells:
            self.cells[i].cell_agent_update(fleet, params)


class Cell(object):
    def __init__(self, obs, fuel, fire, fireupdate, water_accum):
        self.water_accum = water_accum
        self.obstacle = obs
        self.fuel = fuel
        self.fire = fire
        self.fireupdate = fireupdate

    def cell_env_update(self, params):

        return 'uhm...'  # TODO add in update rules for the cell given parameters

    def cell_agent_update(self, fleet, params):

        return 'uhm...'  # TODO add in update rules given fleet locations and actions