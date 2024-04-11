#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
#from geometry_msgs.msg import Twist,Pose, Point, Vector3
#from std_msgs.msg import Float64MultiArray
import numpy as np
from builtin_interfaces.msg import Time
from ngc_interfaces.msg import Nu, NuDot, Ned, Tau
import math
from abc import ABC, abstractmethod
from ngc_thruster_system.thrust_torque_curve import ThrustTorqueCurve
np.set_printoptions(formatter={'float_kind': "{:.2f}".format})
"""
This is the Thruster classes that must correspond with the types in the thruster.yaml file
"""

class Thruster():
    def __init__(self,name:str,  thrust_torque_curve:str, prop_diameter):
        super().__init__()
        self.roh = 1024 #kg/m3 #TODO move this to the utils file
        self.thrust_torque_curve = getattr(ThrustTorqueCurve(),thrust_torque_curve)
        self.prop_diameter = prop_diameter
        self.rps = 0
        self.pitch = 0
        self.angle = 0
        self.rps_upper = 0
        self.rps_lower = 0
        self.pitch_upper = 0
        self.pitch_lower = 0
        self.angle_upper = 0
        self.angle_lower = 0
        self.position = [0,0,0] #defines the positon from ceter of origin (CO)
        self.thrust_config_mtrx = np.array([0,0,0,0,0,0,0]) #generalized in 6DOF, to handle forces in surge, sway , heav, pitch, roll, yaw 
        self.debug = True
    
    @abstractmethod
    def get_tau(self, nu, n, pitch, angle = None):
        return NotImplementedError

    @abstractmethod
    def tau_callback(self):
        return NotImplementedError
 
class FixedPitchPropeller(Thruster):
    def __init__(self, name: str, thrust_torque_curve: str, prop_diameter):
       super().__init__(name, thrust_torque_curve, prop_diameter)
    
    def get_tau(self, nu, n, pitch, angle = None):
        A = np.array([0,0,0])
        return A

class ControllablePitchPropellerAndRudder(Thruster):
    def __init__(self, name: str, thrust_torque_curve: str, prop_diameter):
      super().__init__(name, thrust_torque_curve, prop_diameter)
      self.rudder_height = 0.0
      self.rudder_length = 0.0
      self.rudder_thikness = 0.0
      self.rudder_block_coeff = 0.0
      self.distance_propeller_rudder = 0.0
      
    def get_tau(self, nu, n, pitch, angle = None):
        #The following model and variables are based on the propellor-rudder models in: 
        #[1] https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2584935/Third_paper_submitted_manuscript.pdf?sequence=2 
        #[2] Fossen 2021 at page 238
        #[3] H. Yasukawa, Y.Yoshimura, 2015 Introduction of MMG standard method for ship maneuvering prediction:
        #[4] Brix, Verlag, 1993 Manouvering Technichal Manual
        #[5] Bertram, 2000, Practical Ship Hydrodynamics
        roh = self.roh
        b = self.rudder_height
        l = self.rudder_length
        A_R = b*l #area of rudder
        ar = b**2 /(A_R) #aspect ratio
        #CN = 6.13 *ar /(ar +2.25)
        x_p = x_R = self.position[0] # longitudial position of propeller relative to midship
        U = math.sqrt(nu.u**2 + nu.v**2)
        print(f"abs velocity: {U}")
        C_B = self.rudder_block_coeff # guessed values since the rudder is thick at one end a thin at the other Block coefficient (how much of the box volume is the rudder displacing?)
        L_pp = 11.0 #length between perpendiculars #make this a variable in the ship model config file
        epsilon = 1.09 #typically 1.09 according to fossen at page 238
        r = nu.r # yaw rate
        beta =  math.atan2(nu.v, nu.u) #hull drift angle at mid ship
        
        ###### calculate the proppeller thrust force ######
        nabla = C_B * (self.rudder_height*self.rudder_length*self.rudder_thikness) # Displacment volume of the rudder
        
        a_H = 40*nabla/(L_pp**3)  #
        wp0 = 0.5 * C_B - 0.05 # wake coefficeients in straight motion for centerline screws 
        #TODO: We must identify the wake coefficient for a double propeller
        t = 0.6*wp0
        u= nu.u #surge velocity
        D = self.prop_diameter
        eta = D/b #ratio between propeller diameter and rudder height
        k = 0.5 # experimental constant is often put 0.5 according to Fossen at p. 238
        beta_p = beta -x_p*r #propeller inflow angle
        wp =  wp0 *  math.exp(-4* beta_p**2)    # For ships with one propeller typical values are between 0.2 and 0.45 according to https://www.quora.com/What-is-the-wake-fraction-of-a-propeller-in-a-ship 
        y_R = 1 #straighetening effect from the propeler slip stream (the propeller and rudder are very close and it is therefore set to 1)
        beta_R = beta * x_R # effective inflow angle to rudder        
        f_alpha = (6.13*A_R)/(A_R +2.25) #rudder profile
        t_R = 0.45-0-25*C_B
        x_H = -(0.4 +0.1*C_B)
        
        #TODO: Investigate if it is a problem that this canot be a negative number.
        if n ==0.0 :
            J = 0.0
        else: 
            J = u*(1-wp)/(n*D)

        tau_prop_x = (1-t) * roh *n**2 * D**4 *self.thrust_torque_curve(J=J, pitch= pitch)
        
        #alternative formulation (unknown source) : tau_prop_x = roh * D**4 *(math.pi * 0.2**2 * 0.5) * self.thrust_torque_curve(pitch = pitch, J= J) * n**2
        
        #TODO: simplify this so we have one expression for the rudder velocity 
        ###### calculate the rudder steering forces ######
        if n == 0.0: #if the propeller is at stand still
            u_R = epsilon *u*(1-wp) #The velocity over the rudder is only due to the surge speed of the vessel
        elif u == 0.0: #if the surge speed is zero, the only velocity over the rudder is caused by the propeller
            #TODO: simplify this
            #the follwoing model is derived by using a combination of the soruces: [3](Appendix. A),[4](p.84), [5](p.190)
            UR0 = 0 #the surge speed is zero in this case
            x= self. distance_propeller_rudder
            r_0 = self.prop_diameter/2 #radius of slipstream at infinite distance behind the propeller
            du_inf = math.sqrt(8*tau_prop_x/(roh*math.pi*self.prop_diameter**2)) #the slip stream velocity due to pressure increase over propeller
            r_inf = r_0 *math.sqrt(0.5*(1+(UR0/du_inf))) #the slip stream radius far away from the propeller (source Brix p.84)
            r_x = r_0 *((0.14*(r_inf/r_0)**3 +(r_inf/r_0) + (x/r_0)**(1.5))/(0.14*(r_inf/r_0)**3+ (x/r_0)**(1.5))) 
            u_RP = du_inf *(r_inf/r_x)**2 #corrected slipstream velocity at prop position
            u_R = math.sqrt(eta * u_RP**2 +(1- eta) *UR0**2)

        else: #if the vessle is moving and the propeller is running
            u_R = epsilon *u*(1-wp) * math.sqrt(eta*(1+ k * (math.sqrt(1 + (8*self.thrust_torque_curve(pitch = pitch,J= J)/(math.pi *J**2)))-1)**2 +(1-eta))) 
            print(f"vel rudder: {u_R}")
        #TODO: Go through this and evaluate if it is necessary 
        """v_R = U*y_R *beta_R
        
        if v_R != 0 and u_R != 0:
            a_R = angle - math.atan(u_R/v_R)
        else:
            a_R =  angle
        
        U_R = math.sqrt(u_R**2 + v_R**2)"""
        U_R = u_R
        a_R = angle
        #compute the rudder Normal force based on the relative rudder angle of attack and velocities
        tau_R_N= 0.5 *roh *A_R*U_R**2 *f_alpha*math.sin(a_R)
        X_R = -(1-t_R)*tau_R_N *math.sin(a_R)
        Y_R = -(1+a_H)*tau_R_N *math.cos(a_R)
        N_R = -(x_R +a_H*x_H)*tau_R_N *math.cos(a_R)
        if self.debug:
            print(f"rudder angle: {angle}")
            print(f"angle of attack (considering nu.u and nu.v): {a_R}")
            print(f"thrust form prop: {tau_prop_x}")
            print(f"vel over rudder: {u_R}")
            print(f"normal force on rudder: {tau_R_N}")
            print(f"X_R: {X_R}")
            print(f"")

        tau = np.array([math.copysign(tau_prop_x, n)+X_R, Y_R, N_R])

        return tau

class Azimuth(Thruster):
    def __init__(self, name: str, thrust_torque_curve: str, prop_diameter):
       super().__init__(name, thrust_torque_curve, prop_diameter)

    def get_tau(self, nu, n, pitch, angle = None):
        A = np.array([0,0,0])
        return A

class Tunnel(Thruster):
    def __init__(self, name: str, thrust_torque_curve: str, prop_diameter):
       super().__init__(name, thrust_torque_curve, prop_diameter)

    def get_tau(self, nu, n, pitch, angle = None):
        A = np.array([0,0,0])
        return A


def main(args = None):
 pass

if __name__ == '__main__':
    main()


