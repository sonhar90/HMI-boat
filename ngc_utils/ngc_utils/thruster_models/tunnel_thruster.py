from .thrusters_base import Thruster
import ngc_utils.math_utils as mu

class TunnelThruster(Thruster):

    def __init__(self, config, physical_params):
        super().__init__(config, physical_params)

    def get_force(self, rps, nu, pitch = None):

        Fx = 0.0
        
        rps = mu.saturate(rps,self.propeller.min_rpm,self.propeller.max_rpm)
        
        # Use a rection of 15% for the water flow into the tunnel 
        water_inflow = 0.85*(nu[1] + self.position[0] * nu[2]) 

        # Special handling to avoid Kt0 to be unsymmetric due to assumptions in the proeller class
        if water_inflow < 0 and rps < 0:
            normalized_thrust = -self.propeller.get_thrust(abs(rps),abs(water_inflow), pitch)
            
        else:
            normalized_thrust = self.propeller.get_thrust(rps,water_inflow, pitch)
      
        # Reducing the effectiveness of the thrust as a function of forward velocity
        velocity_reduction_factor = mu.saturate((8 - abs(nu[0]))/8,0.2,1)

        Fy = normalized_thrust*velocity_reduction_factor

        return Fx, Fy
        