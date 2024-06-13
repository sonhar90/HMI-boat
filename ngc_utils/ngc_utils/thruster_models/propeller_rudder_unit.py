from .thrusters_base import Thruster
import ngc_utils.math_utils as mu
import math

class PropellerRudderUnit(Thruster):

    def __init__(self, config, physical_params):
        super().__init__(config, physical_params)

        self.rudder_angle_rad = 0.0

        self.height                  = config['rudder']['height']                        # the rudder height in meters
        self.width                   = config['rudder']['width']                         # Average width
        self.max_rudder_angle_deg    = config['rudder']['max_rudder_angle_deg']
        self.stall_angle_deg         = config['rudder']['stall_angle_deg']
        self.CL_stall                = config['rudder']['CL_stall']
        self.CD_zero_deg             = config['rudder']['CD_zero_deg']
        self.prop_to_rudder_distance = config['rudder']['prop_to_rudder_distance']        # distance from center of propeller to center of lift to rudder
        self.rudder_time_constant    = config['rudder']['rudder_time_constant']           # sec

    def get_force(self, rps, nu, rudderAngleRad, pitch = None):

        rudder_drag = 0
        rudder_lift = 0

        rps = mu.saturate(rps,self.propeller.min_rpm,self.propeller.max_rpm)
        rudderAngleRad = mu.saturate(rudderAngleRad, -math.radians(self.max_rudder_angle_deg), math.radians(self.max_rudder_angle_deg))

        # It is assmumed that the hull has a rightening effect so that the vessel sway speed can be neglected
        Va     = (1.0 - self.propeller.wake_factor)* nu[0]
        Thrust = self.propeller.get_thrust(rps, Va, pitch)

        if Va < 0:
            Va = 0.0

        if Va**2 + (8*Thrust/(self.rho*math.pi*self.propeller.diameter**2)) > 0:
        
            # Water velocity over the rudder
            V_inf = math.sqrt(Va**2 + (8.0*Thrust/(self.rho*math.pi*self.propeller.diameter**2)))
            r_0   = 0.5*self.propeller.diameter
            r_inf = r_0*math.sqrt(0.5*(1 + (Va/V_inf)))
            r_x   = r_0*(0.14*(r_inf/r_0)**3 + (r_inf/r_0)*(self.prop_to_rudder_distance/r_0)**(3.0/2.0))/(0.14*(r_inf/r_0)**3 + (self.prop_to_rudder_distance/r_0)**(3.0/2.0))
            V_x   = V_inf*(r_inf/r_x)**2

            area = self.height * self.width
            C_d  = self.CD_zero_deg  +  4.0*(1.2 - self.CD_zero_deg)/(math.pi**2)*rudderAngleRad**2     # Assume drag coefficient is 1.2 at 90 degrees
            
            if abs(rudderAngleRad) < math.radians(self.stall_angle_deg):
                coef = self.CL_stall / math.radians(self.stall_angle_deg)
                C_l  = coef*rudderAngleRad
            elif abs(rudderAngleRad) < math.radians(self.stall_angle_deg) + 0.05:                       # Adding in a 0.05 radian flat zone at the lift curve top
                C_l  = self.CL_stall
            else:
                coef = mu.saturate(self.CL_stall - ((2.0*self.CL_stall / math.radians(self.stall_angle_deg))*(abs(rudderAngleRad) - (math.radians(self.stall_angle_deg) + 0.05))),0,self.CL_stall)
                C_l  = coef*rudderAngleRad
            
            rudder_drag = 0.5*C_d*self.rho*area*V_x**2
            rudder_lift = 0.5*C_l*self.rho*area*V_x**2
        
        Fx = Thrust - rudder_drag
        Fy = rudder_lift
        
        return Fx, Fy
        