from abc import ABC, abstractmethod
from .fixed_pitch_propeller import FixedPitchPropeller
from ngc_interfaces.msg import ThrusterSignals
from .controllable_pitch_propeller import ControllablePitchPropeller 

class Thruster:
    def __init__(self, config, physical_params):

        self.id       = config['id']
        self.position = config['position']
        self.type     = config['type']
        self.active   = False

        self.rho = physical_params['rho_water']

        # Initialize setpoint data
        self.setpoints             = ThrusterSignals()
        self.setpoints.thruster_id = self.id
        self.setpoints.rps         = 0.0
        self.setpoints.pitch       = 0.0
        self.setpoints.azimuth_deg = 0.0
        self.setpoints.active      = False
        self.setpoints.error       = False

        # Propeller 
        if config['propeller']['type'] == 'cpp':
            self.propeller = ControllablePitchPropeller(config['propeller'],physical_params)
        elif config['propeller']['type'] == 'fpp':
            self.propeller = FixedPitchPropeller(config['propeller'],physical_params)
        else:
            # Should throw exception
            raise Exception('Propeller type exception in constructor')
    
    @abstractmethod
    def get_force(self, *args, **kwargs):
        raise NotImplementedError("Subclass must implement abstract method")
    
    