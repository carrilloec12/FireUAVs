'''
These two classes contain the environment as a whole and all individual cells in the environment. The update rules
regarding the cells still need to be added from Matlab code
'''


class Env(object):
    def __init__(self):
        self.cells = {}

    def add_cell(self, key, *args):
        self.cells[key] = Cell(*args)

    def update_cells(self, params, time_step):
        for i in self.cells:
            if (self.cells[i].fire > 0 and i[0] != 1 and i[0] != params.width and i[1] != 1 and
                    i[1] != params.height and self.cells[i].obstacle != 1 and self.cells[i].ext != 1):
                self.cells[i].firetime = self.cells[i].firetime + time_step
                if self.cells[i].fireupdate <= self.cells[i].firetime:
                    fuel_drop = self.cells[i].fuel - self.cells[i].fire
                    self.cells[i].fuel = 0 if fuel_drop < 0 else fuel_drop
                    fire_rise = self.cells[i].fire + 1
                    self.cells[i].fire = params.max_fire_intensity if fire_rise > params.max_fire_intensity else \
                        fire_rise
                    self.cells[i].water_accum = 0.0
                    self.cells[i].firetime = 0.0
                    self.cells[i].fireupdate = params.fire_update_times[self.cells[i].fire]
                    self.cells[i].update_cells = True
                else:
                    self.cells[i].update_cells = False

                if self.cells[i].fuel == 0:
                    self.cells[i].fire = 0
                    self.cells[i].firetime = 0.0
                    self.cells[i].fireupdate = params.fire_update_times[self.cells[i].fire]
                    self.cells[i].ext = 1

            else:
                self.cells[i].fire = 0
                self.cells[i].firetime = 0.0
                self.cells[i].update_cells = False

        for i in self.cells:
            if self.cells[i].update_cells is True:
                update_keys = [(i[0] + 1, i[1]), (i[0] - 1, i[1]), (i[0], i[1] + 1), (i[0], i[1] - 1)]
                #print(update_keys)
                #print(i)
                #input('wait...')
                for r in update_keys:
                    #print(r)
                    #print(r[0] != 0.0 and r[0] != params.width and r[1] != 0.0 and
                    #r[1] != params.height and self.cells[r].obstacle != 1 and self.cells[r].fire < self.cells[i].fire
                    #and self.cells[r].ext == 0)
                    #input('wait...')
                    if (r[0] != 1 and r[0] != params.width and r[1] != 1 and
                    r[1] != params.height and self.cells[r].obstacle != 1 and self.cells[r].fire < self.cells[i].fire
                    and self.cells[r].ext == 0):
                        #input('wait...')
                        fire_rise = self.cells[i].fire
                        self.cells[r].fire = params.max_fire_intensity if fire_rise > params.max_fire_intensity else \
                            fire_rise
                        self.cells[r].water_accum = 0.0
                        self.cells[r].firetime = 0.0
                        self.cells[r].fireupdate = params.fire_update_times[self.cells[r].fire]

    def update_cells_agent_action(self, params, fleet):
        for i in self.cells:
            #self.cells[i].cell_agent_update(fleet, params)
            l = 1


class Cell(object):
    def __init__(self, params, obs, fuel, fire, fireupdate, water_accum):
        self.water_accum = water_accum
        self.obstacle = obs
        self.fuel = fuel
        self.fire = fire
        self.firetime = 0.0
        self.fireupdate = params.fire_update_times[self.fire]
        self.ext = 0
        self.update_cells = False

    def cell_env_update(self, params):

        return 'uhm...'  # TODO add in update rules for the cell given parameters

    def cell_agent_update(self, water_accum):
        self.water_accum = water_accum
