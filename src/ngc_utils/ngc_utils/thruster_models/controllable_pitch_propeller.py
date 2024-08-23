from .propeller_base import Propeller
import ngc_utils.math_utils as mu

class ControllablePitchPropeller(Propeller):
    
    def __init__(self, config, physical_params):
        super().__init__(config, physical_params)

        self.max_pitch           = config['max_pitch']
        self.min_pitch           = config['min_pitch']
        self.pitch_time_constant = config['pitch_time_constant']

        self.pitch = 0.0

    def get_thrust(self, rps, Va, pitch = None):
        
        if pitch == None:
            raise ValueError('Pitch cannot be None')

        return 0.5*self.get_Kt(rps, Va, pitch)*self.rho*rps**2*self.diameter**4 
    
    def get_Kt(self, rps, Va, pitch = None):
        
        if abs(rps) < mu.eps or pitch == None:
            return 0
        else:
        
            J = Va / rps * self.diameter

            # Note, this is a simplification that serves to reduce Kt when the pitch is negative
            pitch_normalized = pitch / self.max_pitch

            # This is a simplification 
            if Va <  0 and rps > 0:
                return self.Kt0*pitch_normalized

            # This is a simplification 
            elif Va > 0 and rps < 0:
                return -self.Kt0*pitch_normalized

            elif Va < 0 and rps < 0:
                
                J = Va / rps * self.diameter

                # Compute simplified J fall off factor based on Kt0 value and some assumptions
                zero_crossing_J = mu.saturate(1.4*pitch_normalized,0.5,1.4)
                Kj              = self.Kt0*pitch_normalized / zero_crossing_J

                # Compute Kt
                return -self.Kt0*pitch_normalized + Kj*J

            else:

                J = Va / rps * self.diameter    

                # Compute simplified J fall off factor based on Kt0 value and some assumptions
                zero_crossing_J = mu.saturate(1.4*pitch_normalized,0.5,1.4)
                Kj              = self.Kt0*pitch_normalized / zero_crossing_J

                # Compute Kt
                return self.Kt0*pitch_normalized - Kj*J
            



        
                

               
                
                 

            