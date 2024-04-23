import numpy as np
import ngc_utils.math_utils as mu

class VesselModel:
    def __init__(self, config):
        self.vessel_name = config['vessel']['name']
        self.dimensions = config['vessel']['dimensions']
        self.mass = config['vessel']['mass'] + config['vessel']['payload'] 
        self.second_moment_of_inertia = config['vessel']['second_moment_of_inertia']
        self.hydrodynamic_coefficients = config['vessel']['hydrodynamic_coefficients']
        self.hydrostatic_coefficients = config['vessel']['hydrostatic_coefficients']
        self.aerodynamic_coefficients = config['vessel']['aerodynamic_coefficients']

        # Calculate the 6 DOF inverse mass matrix
        self.Ig = self.mass * np.diag([self.second_moment_of_inertia['R44']**2, self.second_moment_of_inertia['R55']**2, self.second_moment_of_inertia['R66']**2])

        H = self.transform_center_of_gravity_to_center_of_origin()
        self.MRB_CG = np.vstack([np.hstack([self.mass*np.eye(3, dtype=float), np.zeros((3,3), dtype=float)]),
                            np.hstack([np.zeros((3,3), dtype= float), self.Ig])])
        
        self.MRB_CO = H.T @ self.MRB_CG @ H # eq. 3.29 in HMHMC
        self.MA = np.diag([self.hydrodynamic_coefficients['added_mass']['X_udot'], self.hydrodynamic_coefficients['added_mass']['Y_vdot'], self.hydrodynamic_coefficients['added_mass']['Z_wdot'], self.hydrodynamic_coefficients['added_mass']['K_pdot'], self.hydrodynamic_coefficients['added_mass']['M_qdot'], self.hydrodynamic_coefficients['added_mass']['N_rdot']])
        self.M = self.MRB_CO + self.MA
    
        self.MassInv6Dof = np.linalg.inv(self.M) 

    def display_info(self):
        print(f"Ship Dimensions: Length {self.dimensions['length']}m, Width {self.dimensions['width']}m, Draft {self.dimensions['draft']}m")
        print(f"Mass: {self.mass} kg")

    def get_hydrodynamic_damping_X(self, nu_r):
        
        X_uu = self.hydrodynamic_coefficients['nonlinear_damping']['X_uu']
        X_ur = self.hydrodynamic_coefficients['nonlinear_damping']['X_ur']
        X_uv = self.hydrodynamic_coefficients['nonlinear_damping']['X_uv']
        
        u = nu_r[0]
        v = nu_r[1]
        r = nu_r[5]

        return - X_uu*abs(u)*u - X_ur*abs(u)*r - X_uv*abs(u)*v 
    
    def get_hydrodynamic_damping_Y(self, nu_r):
        
        L = self.dimensions['length']
        B = self.dimensions['width']
        T = self.dimensions['draft']

        YN_crossflow = self.crossFlowDrag(L,B,T,nu_r)

        return YN_crossflow[1]
    
    def get_hydrodynamic_damping_Z(self, nu_r):
        
        Z_w = self.hydrodynamic_coefficients['linear_damping']['Z_w']
        
        w = nu_r[2]
 
        return - Z_w*w
    
    def get_hydrodynamic_damping_K(self, nu_r):
        
        K_p = self.hydrodynamic_coefficients['linear_damping']['K_p']
        
        p = nu_r[3]
 
        return - K_p*p
    
    def get_hydrodynamic_damping_M(self, nu_r):
        
        M_q = self.hydrodynamic_coefficients['linear_damping']['M_q']
        
        q = nu_r[4]
 
        return - M_q*q
    
    def get_hydrodynamic_damping_N(self, nu_r):
    
        L = self.dimensions['length']
        B = self.dimensions['width']
        T = self.dimensions['draft']

        YN_crossflow = self.crossFlowDrag(L,B,T,nu_r)

        return YN_crossflow[5]


    def get_6dof_hydrodynamic_damping(self, nu_r):

        damping = np.array([0,0,0,0,0,0], dtype= float)

        damping[0] = self.get_hydrodynamic_damping_X(nu_r)
        damping[1] = self.get_hydrodynamic_damping_Y(nu_r)
        damping[2] = self.get_hydrodynamic_damping_Z(nu_r)
        damping[3] = self.get_hydrodynamic_damping_K(nu_r)
        damping[4] = self.get_hydrodynamic_damping_M(nu_r)
        damping[5] = self.get_hydrodynamic_damping_N(nu_r)

        return damping
    
    def transform_center_of_gravity_to_center_of_origin(self):
        return  np.vstack([np.hstack([np.eye(3, dtype= float), mu.skew_symetric_matrix(self.dimensions['r_g']).T]),
                        np.hstack([np.zeros((3,3), dtype=float), np.eye(3,dtype=float)])])
    
    
    def Hoerner(self, B,T):
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
    
    def crossFlowDrag(self, L:float,B:float,T:float,nu_r:np.array): 
        """Calculates the nonlinear damping force.

        Args:
            L (float): Length of the boat
            B (float): Beam
            T (float): Drat
            nu_r (np.array): nu

        Returns:
            numpy.array : Tau_crossflow
        """
        
        rho = self.simulation_physical_params['rho_water']
        n = 20                          # number of strips

        dx = L/20             

        Cd_2D = self.Hoerner(B,T)       # 2D drag coefficient based on Hoerner's curve

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
    
    def get_6DOF_coreolis_centripital_forces(self,nu):
    
        """
        This omputes the Coriolis and centripetal forces from the
        mass matrix M and generalized velocity vector nu
        """

        M = 0.5 * (self.M + self.M.T)    

        M11 = M[0:3,0:3]
        M12 = M[0:3,3:6] 
        M21 = M12.T
        M22 = M[3:6,3:6] 
    
        nu1 = nu[0:3]
        nu2 = nu[3:6]
        dt_dnu1 = np.matmul(M11,nu1) + np.matmul(M12,nu2)
        dt_dnu2 = np.matmul(M21,nu1) + np.matmul(M22,nu2)

        C = np.zeros( (6,6) )    
        C[0:3,3:6] = -mu.skew_symetric_matrix(dt_dnu1)
        C[3:6,0:3] = -mu.skew_symetric_matrix(dt_dnu1)
        C[3:6,3:6] = -mu.skew_symetric_matrix(dt_dnu2)

        return C @ nu
     
        """ 3 DOF computation
            C = np.zeros( (3,3) ) 
            C[0,2] = -M[1,1] * nu[1] - M[1,2] * nu[2]
            C[1,2] =  M[0,0] * nu[0] 
            C[2,0] = -C[0,2]       
            C[2,1] = -C[1,2]
        """
        
    def set_physical_parameters(self, params):

        self.simulation_physical_params = params

    def get_6dof_restoring_forces(self, eta):

        restoring = np.array([0,0,0,0,0,0], dtype= float)

        rho   = self.simulation_physical_params['rho_water']
        g     = self.simulation_physical_params['gravity']
        nabla = self.mass / rho 

        # Heave
        restoring[2] = self.hydrostatic_coefficients['A_wp'] * eta[2]

        #  Roll
        restoring[3] = nabla * self.hydrostatic_coefficients['GM_T'] * eta[3]

        # Pitch
        restoring[4] = nabla * self.hydrostatic_coefficients['GM_L'] * eta[4]

        return rho*g*restoring

    def interpolate_coefficients(self, direction, coefficients, directions):
        # Convert directions to a numpy array if not already one
        directions = np.array(directions, dtype=float)
        
        # Find indices where directions are less than or equal to the given direction
        lower_indices = np.where(directions <= direction)[0]
        if lower_indices.size == 0:
            # No directions are less than or equal to 'direction'; take the smallest direction
            lower_index = 0
        else:
            lower_index = np.max(lower_indices)

        # Find indices where directions are greater than or equal to the given direction
        upper_indices = np.where(directions >= direction)[0]
        if upper_indices.size == 0:
            # No directions are greater than or equal to 'direction'; take the largest direction
            upper_index = len(directions) - 1
        else:
            upper_index = np.min(upper_indices)

        if lower_index == upper_index:
            # The direction matches exactly or all directions are the same
            return coefficients[lower_index]

        # Linear interpolation of the coefficients
        lower_direction = directions[lower_index]
        upper_direction = directions[upper_index]
        lower_coeff = coefficients[lower_index]
        upper_coeff = coefficients[upper_index]

        # Compute the interpolated coefficient
        interpolated_coeff = lower_coeff + (upper_coeff - lower_coeff) * ((direction - lower_direction) / (upper_direction - lower_direction))

        return interpolated_coeff

    def get_6Dof_wind_forces(self, eta, nu,  wind_direction, wind_speed):
        
        # Extract aerodynamic coefficients from the configuration
        directions = np.radians(np.array(self.aerodynamic_coefficients['relative_directions']))
        C_X = np.array(self.aerodynamic_coefficients['C_X'])
        C_Y = np.array(self.aerodynamic_coefficients['C_Y'])
        C_K = np.array(self.aerodynamic_coefficients['C_K'])
        C_N = np.array(self.aerodynamic_coefficients['C_N'])

        relative_wind_speed, relative_wind_direction = self.get_relative_wind_speed_and_direction(eta, nu, wind_direction, wind_speed)

        wind_sign = np.sign(relative_wind_direction)
        relative_wind_direction = np.abs(relative_wind_direction)

        # Interpolate coefficients based on the given wind direction
        CX = self.interpolate_coefficients(relative_wind_direction, C_X, directions)
        CY = self.interpolate_coefficients(relative_wind_direction, C_Y, directions)
        CK = self.interpolate_coefficients(relative_wind_direction, C_K, directions)
        CN = self.interpolate_coefficients(relative_wind_direction, C_N, directions)

        # Calculate forces and moments using the wind speed
        # Assuming the wind force is proportional to the square of the wind speed
        rho_air = self.simulation_physical_params['rho_air']
        A_frontal = self.aerodynamic_coefficients['A_frontal'] 
        A_lateral = self.aerodynamic_coefficients['A_lateral'] 
        
        Fx = 0.5 * rho_air * CX * A_frontal * relative_wind_speed**2
        Fy = wind_sign * 0.5 * rho_air * CY * A_lateral * relative_wind_speed**2
        Mk = wind_sign * 0.5 * rho_air * CK * A_lateral * relative_wind_speed**2 * (A_lateral / self.dimensions['length'])
        Mn = wind_sign * 0.5 * rho_air * CN * A_lateral * relative_wind_speed**2 * self.dimensions['length'] 

        return np.array([Fx, Fy, 0, Mk, 0, Mn],float)
    
    def get_relative_wind_speed_and_direction(self, eta, nu, wind_direction, wind_speed):
        
        nu_wind = wind_speed*np.array([np.cos(np.radians(wind_direction) - eta[5]), np.sin(np.radians(wind_direction) - eta[5]), 0],float)
        
        u_rw = nu[0] - nu_wind[0]
        v_rw = nu[1] - nu_wind[1]
        
        relative_wind_direction = mu.mapToPiPi(-np.arctan2(v_rw,u_rw))
        relative_wind_speed = np.sqrt(u_rw**2 + v_rw**2)

        return relative_wind_speed, relative_wind_direction