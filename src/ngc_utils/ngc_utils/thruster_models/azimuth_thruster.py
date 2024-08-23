from .thrusters_base import Thruster
import ngc_utils.math_utils as mu
import math

class AzimuthThruster(Thruster):

    def __init__(self, config, physical_params):
        super().__init__(config, physical_params)

        self.azimuth_angle_rad = 0.0
      
    def get_force(self, rps, nu, azimuthAngleRad, pitch = None):

        rps = mu.saturate(rps,self.propeller.min_rpm,self.propeller.max_rpm)

        # It is assmumed that the hull has a rightening effect so that the vessel sway speed can be neglected
        Va     = (1.0 - self.propeller.wake_factor)* nu[0]
        Thrust = self.propeller.get_thrust(rps, Va, pitch)

        Fx = Thrust*math.cos(azimuthAngleRad)
        Fy = Thrust*math.sin(azimuthAngleRad)
        
        return Fx, Fy
        