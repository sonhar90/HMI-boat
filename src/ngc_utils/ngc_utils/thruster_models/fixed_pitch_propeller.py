from .propeller_base import Propeller
import ngc_utils.math_utils as mu

class FixedPitchPropeller(Propeller):
  
    def __init__(self, config, physical_params):
        super().__init__(config, physical_params)
    
    
    def get_thrust(self, rps, Va, pitch = None):
    
        return 0.5*self.get_Kt(rps, Va)*self.rho*rps**2*self.diameter**4 
   

    def get_Kt(self, rps, Va, pitch = None):
        
        if abs(rps) < mu.eps or pitch != None:
            return 0
        else:
            
            # This is a simplification 
            if Va <  0 and rps > 0:
                
                return self.Kt0

            # This is a simplification 
            elif Va > 0 and rps < 0:
                
                return -self.Kt0

            elif Va < 0 and rps < 0:
                
                J = Va / rps * self.diameter

                # Compute Kt - assumed J zero crossing is 1.4
                return -self.Kt0 + (self.Kt0/1.4)*J

            else:
                
                J = Va / rps * self.diameter    

                # Compute Kt - assumed J zero crossing is 1.4
                return self.Kt0 - (self.Kt0/1.4)*J