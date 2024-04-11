#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
#from geometry_msgs.msg import Twist,Pose, Point, Vector3
#from std_msgs.msg import Float64MultiArray
import numpy as np
from builtin_interfaces.msg import Time
from ngc_interfaces.msg import Nu, NuDot, Ned, Tau

np.set_printoptions(formatter={'float_kind': "{:.2f}".format})

"""
Source: Handbook of marine craft hydrodynamic and motion control (HMHMC) Fossen. 2021
"""
class Simulator(Node):
    def __init__(self):
        super().__init__("ngc_sim")
        self.counter_ = 0
        self.eta = np.array([0,0,0,0,0,0], dtype= float)
        self.nu = np.array([0,0,0,0,0,0], dtype= float)

        self.nu_dot  = np.array([0,0,0,0,0,0], dtype= float)

        self.eta_pub= self.create_publisher(Ned, "eta_sim", 1)
        self.nu_pub = self.create_publisher(Nu, "nu_sim", 1)
        self.nu_dot_pub = self.create_publisher(NuDot, "nu_dot_sim", 1)
        self.tau_sub = self.create_subscription(Tau, "tau_prop",  self.force_cmd_callback, 10)
        self.timer_= self.create_timer(0.01, self.integrator_callback)
        self.prev_timestamp = None
        self.tau = Tau()
        self.declare_parameters(
            namespace = "",
            parameters= [
            ('m', rclpy.Parameter.Type.DOUBLE),
            ('l', rclpy.Parameter.Type.DOUBLE),
            ('b', rclpy.Parameter.Type.DOUBLE),
            ('draft', rclpy.Parameter.Type.DOUBLE),
            ('roh', rclpy.Parameter.Type.DOUBLE),
            ('payload', rclpy.Parameter.Type.DOUBLE),
            ('r_g', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ('R44', rclpy.Parameter.Type.DOUBLE),
            ('R55', rclpy.Parameter.Type.DOUBLE),
            ('R66', rclpy.Parameter.Type.DOUBLE),
            ('A_wp', rclpy.Parameter.Type.DOUBLE),
            ('GMT', rclpy.Parameter.Type.DOUBLE),
            ('GML', rclpy.Parameter.Type.DOUBLE),
            ('LCF', rclpy.Parameter.Type.DOUBLE),
            ('r_p', rclpy.Parameter.Type.DOUBLE),
            ('Xu', rclpy.Parameter.Type.DOUBLE),
            ('Xuu', rclpy.Parameter.Type.DOUBLE),
            ('Xuuu', rclpy.Parameter.Type.DOUBLE),
            ('Xurr', rclpy.Parameter.Type.DOUBLE),
            ('Yv', rclpy.Parameter.Type.DOUBLE),
            ('Zw', rclpy.Parameter.Type.DOUBLE),
            ('Kp', rclpy.Parameter.Type.DOUBLE),
            ('Mq', rclpy.Parameter.Type.DOUBLE),
            ('Nr', rclpy.Parameter.Type.DOUBLE),
            ('Nur', rclpy.Parameter.Type.DOUBLE),
            ('Nuur', rclpy.Parameter.Type.DOUBLE),
            ('Xudot', rclpy.Parameter.Type.DOUBLE),
            ('Yvdot', rclpy.Parameter.Type.DOUBLE),
            ('Zwdot', rclpy.Parameter.Type.DOUBLE),
            ('Kpdot', rclpy.Parameter.Type.DOUBLE),
            ('Mqdot', rclpy.Parameter.Type.DOUBLE),
            ('Nrdot', rclpy.Parameter.Type.DOUBLE)
        ])
        
        self.m = self.get_parameter('m').get_parameter_value().double_value
        self.l = self.get_parameter('l').get_parameter_value().double_value
        self.b = self.get_parameter('b').get_parameter_value().double_value
        self.draft = self.get_parameter('draft').get_parameter_value().double_value
        self.roh = self.get_parameter('roh').get_parameter_value().double_value
        self.payload = self.get_parameter('payload').get_parameter_value().double_value
        self.r_g = self.get_parameter('r_g').get_parameter_value().double_array_value
        
        #inertia dyadics coefficient to compute Ig
        self.R44 = self.get_parameter('R44').get_parameter_value().double_value
        self.R55 = self.get_parameter('R55').get_parameter_value().double_value
        self.R66 = self.get_parameter('R66').get_parameter_value().double_value

        #hydrostatic--> restoring coefficients/spring stifness - > resulting in the spring stiffnes matrix G
        self.A_wp = self.get_parameter('A_wp').get_parameter_value().double_value
        self.GMT = self.get_parameter('GMT').get_parameter_value().double_value
        self.GML = self.get_parameter('GML').get_parameter_value().double_value
        self.LCF = self.get_parameter('LCF').get_parameter_value().double_value
        self.r_p = self.get_parameter('r_p').get_parameter_value().double_value
        self.nabla = (self.m + self.payload)/self.roh
        
        #linear damping terms
        self.Xu = self.get_parameter('Xu').get_parameter_value().double_value
        self.Yv = self.get_parameter('Yv').get_parameter_value().double_value
        self.Zw = self.get_parameter('Zw').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Mq = self.get_parameter('Mq').get_parameter_value().double_value
        self.Nr = self.get_parameter('Nr').get_parameter_value().double_value
        
        #D matrix definition
        self.D = np.diag([self.Xu, self.Yv, self.Zw, self.Kp, self.Mq, self.Nr])

        #non linear damping terms
        self.Xuu = self.get_parameter('Xuu').get_parameter_value().double_value
        self.Xuuu = self.get_parameter('Xuuu').get_parameter_value().double_value
        self.Xurr = self.get_parameter('Xurr').get_parameter_value().double_value
        self.Nuur = self.get_parameter('Nuur').get_parameter_value().double_value

        self.Nur = self.get_parameter('Nur').get_parameter_value().double_value

        #linear added mass term 
        self.Xudot = self.get_parameter('Xudot').get_parameter_value().double_value
        self.Yvdot = self.get_parameter('Yvdot').get_parameter_value().double_value
        self.Zwdot = self.get_parameter('Zwdot').get_parameter_value().double_value
        self.Kpdot = self.get_parameter('Kpdot').get_parameter_value().double_value
        self.Mqdot = self.get_parameter('Mqdot').get_parameter_value().double_value
        self.Nrdot = self.get_parameter('Nrdot').get_parameter_value().double_value
        
        self.Ig = self.m * np.diag([self.R44**2, self.R55**2, self.R66**2]) # inertia dyadic among center of gravity (CG) se page 65. HMHMC 2021   

        #rigid-body system inertia matrix MRB
        self.MRB_CO = self.__rigid_body_matrix()
        self.MA = np.diag([self.Xudot, self.Yvdot, self.Zwdot, self.Kpdot, self.Mqdot, self.Nrdot])
        self.M = self.MRB_CO + self.MA
        
        #print initialization information to the end-user
        self.get_logger().info(f"M:{self.M}")
        self.get_logger().info(f"MRB_CO:{self.MRB_CO}")   
        self.get_logger().info(f"MA:{self.MA}") 
        self.get_logger().info(f"M:{self.M}") 
        self.get_logger().info(f"CRB:{self.m2c(self.MRB_CO, [0,0,0,0,0,1])}")    
        self.n = 0
        self.get_logger().info("The simulator node has started")
    
    def force_cmd_callback(self, msg:Tau):
        self.tau = msg #takes the twist message and stores it
        

    def integrator_callback(self):
        eta_message = Ned()
        nu_message = Nu()
        nu_dot_message = NuDot()

        now= self.get_clock().now().to_msg() 
        if self.prev_timestamp is None:
            self.prev_timestamp = now # set the first timestep 
        dt = (now.sec - self.prev_timestamp.sec) + (now.nanosec -self.prev_timestamp.nanosec)*1e-9 # 1e-9 #convert to seconds 
        X = self.tau.surge_x
        Y = self.tau.sway_y
        Z = 0
        K = 0
        M = 0
        N = self.tau.yaw_n
        tau_prop = np.array([X,Y,Z,K,M,N])
        #print(f"tau_prop: {tau_prop}")
        tau_wind = np.array([0,0,0,0,0,0])
        tau_wave = np.array([0,0,0,0,0,0])
        #TODO vurder om self.eta og nu er intuitivt her, eller om dynamics bare tar det direkte fra classen

        self.eta, self.nu , self.nu_dot= self.dynamics(eta=self.eta, nu=self.nu, tau_prop=tau_prop, tau_wind=tau_wind, tau_wave=tau_wave, dt = dt)

        
        #construct the messages
        eta_message.x, eta_message.y, eta_message.z, eta_message.psi = self.eta[0], self.eta[1],self.eta[2], self.eta[5] 
        nu_message.u, nu_message.v, nu_message.w, nu_message.p, nu_message.q, nu_message.r = map(lambda x:x, self.nu)

        nu_dot_message.u_dot, nu_dot_message.v_dot, nu_dot_message.w_dot, nu_dot_message.p_dot, nu_dot_message.q_dot, nu_dot_message.r_dot = map(lambda x:x, self.nu_dot)

        self.prev_timestamp = now
        
        #publish the messages
        self.eta_pub.publish(eta_message)
        self.nu_pub.publish(nu_message)
        self.nu_dot_pub.publish(nu_dot_message)

    def dynamics(self, eta:np.array, nu:np.array, tau_prop:np.array, tau_wind:np.array, tau_wave:np.array, dt:float):
        """
        
        The dynamics is modeled with a linear damping 
        M = M+MA
        C(v) = CRB(v)+ CA(v)
        tau = tau_prop + tau_wind + tau_wave
        xdot = [etadot,nudot].T = [R(psi)v, M⁻¹(tau-C(v)v - D(v)v)
        """

        tau = tau_prop #+tau_wave+tau_wind # the extarnal forces acing on the vessel 
        Dcf= self.crossFlowDrag(L = self.l, B= self.b, T=self.draft, nu_r= nu) #returns a negative daming force in yaw and sway 
        D = self.D
        #TODO add squared and cubed surge damping
        #TODO maybe add the roll-infliction onn surge and yaw damping, as in Andrew Rosses PhD thesis 
        M = self.M
        Minv = np.linalg.inv(M)
        CRB = self.m2c(self.MRB_CO, nu= nu) 
        CA = 0*self.m2c(self.MA, nu=nu) #CA is destabelising the model!
        C = CRB + CA
        ex_forces = Dcf - C @ nu - D @ nu - self.Dv(nu=nu) @ nu
        nudot = Minv @ (tau_prop + ex_forces ) 

              
        #forward Euler integration [k+1]
        nu = nu + nudot * dt
        eta = self.attitudeEuler(eta=eta, nu=nu, sampleTime=dt)
        
        #print(f"nu_dot: {nudot}")
        #print(f"nu: {nu}")
        #print(f"eta: {eta}")
        return eta, nu, nudot

    def Dv(self, nu:np.array):
        """Returns the D(nu) matrix.
        NB!Must be multplyed with nu

        Args:
            nu (np.array): _description_

        Returns:
            Dv (np.array): Nonlinear damping matrix (without crossflow)
        """

        u,v,w,p,q,r = map(lambda x: x, nu) #run exctract_value_funct on each element in the vector
        
        Dv = np.zeros((6,6))
        Xuu= self.Xuu * abs(u)
        Xuuu= self.Xuuu * abs(u)**2

        Xurr = self.Xurr * abs(u)*abs(r) # modeling surge damping when turning
        Nur = self.Nur * abs(u) # modeling yaw damping when speed in surge is present #
        Nuur = self.Nuur * abs(u)**2 # modeling yaw damping when speed in surge is present
        
        Dv[0,0] = Xuu + Xuuu
        Dv[5,0] = Xurr
        Dv[0,5] = Nuur
        return Dv
        
    def R_b2ned(self, psi:float):
        """
        rotate the vector from body to NED
        """
        return np.array([[np.cos(psi), -np.sin(psi),0],
                         [np.sin(psi), np.cos(psi), 0],
                         [0,0,1]])

    def Rzyx(self, phi,theta,psi):#(Author: Fossen, 2021)
        """
        R = Rzyx(phi,theta,psi) computes the Euler angle rotation matrix R in SO(3)
        using the zyx convention
        """
        
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

    def Tzyx(self, phi,theta):#(Author: Fossen, 2021)
        """
        T = Tzyx(phi,theta) computes the Euler angle attitude
        transformation matrix T using the zyx convention
        """
        
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

    def attitudeEuler(self, eta,nu,sampleTime):#(Author: Fossen, 2021)
        """
        eta = attitudeEuler(eta,nu,sampleTime) computes the generalized 
        position/Euler angles eta[k+1]
        """
    
        p_dot   = np.matmul( self.Rzyx(eta[3], eta[4], eta[5]), nu[0:3] )
        v_dot   = np.matmul( self.Tzyx(eta[3], eta[4]), nu[3:6] )

        # Forward Euler integration
        eta[0:3] = eta[0:3] + sampleTime * p_dot
        eta[3:6] = eta[3:6] + sampleTime * v_dot
        eta[5] = eta[5]  % (2*np.pi) #map the value between 0 and 2pi where 0 is north

        return eta

    def skew(self, a):
        """
        return the skew symetric matrix
        """
        return np.array([[0, -a[2], a[1]],
                         [a[2], 0, -a[0]], 
                         [-a[1], a[0], 0]])
    
    def __transform_center_of_gravity_to_center_of_origin(self):
        """
        Transform from center of gravity to center of origin. 
        """

        return  np.vstack([np.hstack([np.eye(3, dtype= float), self.skew(self.r_g).T]),
                            np.hstack([np.zeros((3,3), dtype=float), np.eye(3,dtype=float)])])    

    def __rigid_body_matrix(self):
        """
        Computes the system inertia matrix transformed from center of gravity to center of origin
        returns: MRB_CO
        """
        H = self.__transform_center_of_gravity_to_center_of_origin()
        MRB_CG = np.vstack([np.hstack([self.m*np.eye(3, dtype=float), np.zeros((3,3), dtype=float)]),
                            np.hstack([np.zeros((3,3), dtype= float), self.Ig])])
        
        MRB_CO = H.T @ MRB_CG @ H # eq. 3.29 in HMHMC
        return MRB_CO

    def Smtrx(self, a):#(Author: Fossen, 2021)
        """
        S = Smtrx(a) computes the 3x3 vector skew-symmetric matrix S(a) = -S(a)'.
        The cross product satisfies: a x b = S(a)b. 
        """
    
        S = np.array([ 
            [ 0, -a[2], a[1] ],
            [ a[2],   0,     -a[0] ],
            [-a[1],   a[0],   0 ]  ])

        return S
    
    def m2c(self, M, nu):#(Author: Fossen, 2021)
        """
        C = m2c(M,nu) computes the Coriolis and centripetal matrix C from the
        mass matrix M and generalized velocity vector nu (Fossen 2021, Ch. 3)
        """

        M = 0.5 * (M + M.T)     # systematization of the inertia matrix

        if (len(nu) == 6):      #  6-DOF model
        
            M11 = M[0:3,0:3]
            M12 = M[0:3,3:6] 
            M21 = M12.T
            M22 = M[3:6,3:6] 
        
            nu1 = nu[0:3]
            nu2 = nu[3:6]
            dt_dnu1 = np.matmul(M11,nu1) + np.matmul(M12,nu2)
            dt_dnu2 = np.matmul(M21,nu1) + np.matmul(M22,nu2)

            #C  = [  zeros(3,3)      -Smtrx(dt_dnu1)
            #      -Smtrx(dt_dnu1)  -Smtrx(dt_dnu2) ]
            C = np.zeros( (6,6) )    
            C[0:3,3:6] = -self.Smtrx(dt_dnu1)
            C[3:6,0:3] = -self.Smtrx(dt_dnu1)
            C[3:6,3:6] = -self.Smtrx(dt_dnu2)
                
        else:   # 3-DOF model (surge, sway and yaw)
            #C = [ 0             0            -M(2,2)*nu(2)-M(2,3)*nu(3)
            #      0             0             M(1,1)*nu(1)
            #      M(2,2)*nu(2)+M(2,3)*nu(3)  -M(1,1)*nu(1)          0  ]    
            C = np.zeros( (3,3) ) 
            C[0,2] = -M[1,1] * nu[1] - M[1,2] * nu[2]
            C[1,2] =  M[0,0] * nu[0] 
            C[2,0] = -C[0,2]       
            C[2,1] = -C[1,2]
            
        return C
    
    def crossFlowDrag(self, L:float,B:float,T:float,nu_r:np.array): #(Author: Fossen, 2021)
        """Calculates the nonlinear damping force.

        Args:
            L (float): Length of the boat
            B (float): Beam
            T (float): Drat
            nu_r (np.array): nu

        Returns:
            numpy.array : Tau_crossflow
        """
        def Hoerner(B,T):#(Author: Fossen, 2021)
            """
            CY_2D = Hoerner(B,T)
            Hoerner computes the 2D Hoerner cross-flow form coeff. as a function of beam 
            B and draft T.The data is digitized and interpolation is used to compute 
            other data point than those in the table
            """
            
            # DATA = [B/2T  C_D]
            DATA1 = np.array([
                0.0109,0.1766,0.3530,0.4519,0.4728,0.4929,0.4933,0.5585,0.6464,0.8336,
                0.9880,1.3081,1.6392,1.8600,2.3129,2.6000,3.0088,3.4508, 3.7379,4.0031 
                ])
            DATA2 = np.array([
                1.9661,1.9657,1.8976,1.7872,1.5837,1.2786,1.2108,1.0836,0.9986,0.8796,
                0.8284,0.7599,0.6914,0.6571,0.6307,0.5962,0.5868,0.5859,0.5599,0.5593 
                ])

            CY_2D = np.interp( B / (2 * T), DATA1, DATA2 )
            
            return CY_2D
        
        rho = 1026               # density of water
        n = 20                   # number of strips

        dx = L/20             
        Cd_2D = Hoerner(B,T)    # 2D drag coefficient based on Hoerner's curve

        Yh = 0
        Nh = 0
        xL = -L/2
        
        for i in range(0,n+1):
            v_r = nu_r[1]             # relative sway velocity
            r = nu_r[5]               # yaw rate
            Ucf = abs(v_r + xL * r) * (v_r + xL * r)
            Yh = Yh - 0.5 * rho * T * Cd_2D * Ucf * dx         # sway force
            Nh = Nh - 0.5 * rho * T * Cd_2D * xL * Ucf * dx    # yaw moment
            xL += dx
            
        tau_crossflow = np.array([0, Yh, 0, 0, 0, Nh],float)

        

        return tau_crossflow

def main(args=None):
    rclpy.init(args=args) #initiate the ROS2 comunication
    node = Simulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()