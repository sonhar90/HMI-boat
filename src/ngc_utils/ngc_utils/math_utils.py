import math
import numpy as np

eps = 0.00000000001

def mapToPiPi(angle_in_radians):
    # Wrap the angle to the range [-pi, pi]
    wrapped_angle = (angle_in_radians + math.pi) % (2 * math.pi) - math.pi
    
    # This adjustment is necessary to handle the case when angle + pi == 2 * pi
    if wrapped_angle <= -math.pi:
        wrapped_angle += 2 * math.pi
    
    return wrapped_angle

def RotationMatrix(phi,theta,psi):
        
        # Computes the Euler angle rotation matrix R in SO(3) using the zyx convention        
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cth  = np.cos(theta)
        sth  = np.sin(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)
        
        R = np.array([
            [ cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth ],
            [ spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi ],
            [ -sth,      cth*sphi,                 cth*cphi ] ])

        return R

def TranslationMatrix(phi,theta):
        
        # Computes the Euler angle attitude transformation matrix T using the zyx convention
            
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cth  = np.cos(theta)
        sth  = np.sin(theta)    

        try: 
            T = np.array([
                [ 1,  sphi*sth/cth,  cphi*sth/cth ],
                [ 0,  cphi,          -sphi],
                [ 0,  sphi/cth,      cphi/cth] ])
            
        except ZeroDivisionError:  
            print ("Tzyx is singular for theta = +-90 degrees." )
            
        return T

def skew_symetric_matrix(a):
        
        return np.array([[0, -a[2], a[1]],
                         [a[2], 0, -a[0]], 
                         [-a[1], a[0], 0]])

def saturate(value, min, max):
      if value >= max:
            return max
      elif value <= min:
            return min
      else:
            return value