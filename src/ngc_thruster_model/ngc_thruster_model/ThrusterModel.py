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

np.set_printoptions(formatter={'float_kind': "{:.2f}".format})

"""
Source: Handbook of marine craft hydrodynamic and motion control (HMHMC) Fossen. 2021
"""
class ThrusterModel(Node):
    def __init__(self):
        super().__init__("ngc_thruster_model")

    #TODO: legge til launch
    #TODO sette opp setup.py
    

    def get_tau_prop(self):
        #This is the propultion force in surge
        roh = 1026
        b = 0.50 #rudder height
        l = 0.40 #rudder lenght
        t = 0.05 #rudder thikness?

        A_R = b*l #area of rudder
        ar = b**2 /(A_R) #aspect ratio
        CN = 6.13 *ar /(ar +2.25)

        #MMG std. method for ship maneuvering prediction procedure to model the speed ofviscus flow over the rudder
        epsilon = 1.09 #typically 1.09 according to fossen at page 238

        ##### see this paper: https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2584935/Third_paper_submitted_manuscript.pdf?sequence=2 
        #L = 12 #length of rudder (m)
        #B = 3.5 # beam of rudder (m)
        #T = 0.9 # draft of rudder (m)
        x_p = 5.0 # longitudial position of propeller relative to midship
        r = 0 # yaw rate
        beta =  0 #hull drift angle at mid ship
        beta_R = 0 # effective inflow angle to rudder
        beta_p = beta -x_p*r #propeller inflow angle
        C_B = 0.5 # guessed values since the rudder is thick at one end a thin at the other Block coefficient (how much of the box volume is the rudder displacing?)
        nabla = C_B * (l*b*t) # Displacment volume of the rudder
        wp0 = 0.5 * C_B - 0.05 # for centerline screws #TODO: We must identify the wake coefficient for a double propeller
        a_H = 40*nabla/(l**3)  #
        wp =  wp0 *  math.exp(-4* beta_p**2)    # For ships with one propeller typical values are between 0.2 and 0.45 according to https://www.quora.com/What-is-the-wake-fraction-of-a-propeller-in-a-ship 
        #####


        def Kt(pitch, J):
            """
            Basert på wagner B-series 3-blade AE/AO =0.5, P/D (pitch) 0.5-1.4. 
            The values in the diagram are linearzed.
            """
            Kt0 = 0.38
            dKt = 0.35
            return Kt0 * pitch - dKt *J
        
        u= 30 * 0.514 #surge velocity (from 30 knots to ms) 
        D = 0.40 # propeller diameter
        n =  1200/60 # rps ca 1000 rpm i vanlig drift
        eta = D/b #ratio between propeller diameter and rudder height
        k = 0.5 # experimental constant is often put 0.5 according to fossen at p. 238
        pitch = 1.4
        J = u/(n*D)
        print(f"J: {J}, Kt: {Kt(pitch=pitch, J=J)}")
        u_R = epsilon * u *(1-wp) * math.sqrt(eta*(1+k * (math.sqrt(1 + (8*Kt(pitch,J)/(math.pi *J**2))-1))**2 +(1-eta)))
        print(f"Vel over rudder: {u_R}")
        tau_R_x= -0.5 *(0.28*C_B +0.55)*roh *u_R**2*A_R*CN *math.sin(0)**2
        tau_prop_x = roh * D**4 *(math.pi * 0.2**2 * 0.5) * Kt(pitch,J) * n**2
        print(f"tau_prop*2 - tau_rudder*2: {tau_prop_x *2} + {tau_R_x *2 } = {round((tau_prop_x-tau_R_x)*2 * 1/1000, 2)} kN") 
        tau = np.array([tau_prop_x, 0, 0])
        
        return tau

def main(args=None):
    rclpy.init(args=args) #initiate the ROS2 comunication
    node = ThrusterModel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()