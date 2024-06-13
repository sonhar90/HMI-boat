from abc import ABC, abstractmethod

class Propeller:
    def __init__(self, config, physical_params):

        self.diameter          = config['diameter']
        self.max_rpm           = config['max_rpm']
        self.min_rpm           = config['min_rpm']
        self.rpm_time_constant = config['rpm_time_constant']
        self.Kt0               = config['Kt0']
        self.type              = config['type']
        
        self.rho               = physical_params['rho_water']
        self.rps               = 0.0
        self.wake_factor       = config['wake_factor']

    @ abstractmethod
    def get_thrust(self, *args, **kwargs):
        raise NotImplementedError("Subclass must implement abstract method")
        
    @ abstractmethod
    def get_Kt(self, rps, Va, pitch = None):
       raise NotImplementedError("Subclass must implement abstract method")
       