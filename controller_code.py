import rospy

class DroneIn3D:
    
    def __init__(self):
        self.X=np.array([
            # x, y, z, phi, theta, psi, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            # x_dot, y_dot, z_dot, p, q, r
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0])       
        self.g = 9.81

    def R(self):
    
    r_x = np.array([[1, 0, 0],
                    [0, cos(self.X[3]), -sin(self.X[3])],
                    [0, sin(self.X[3]), cos(self.X[3])]])

    r_y = np.array([[cos(self.X[4]), 0, sin(self.X[4])],
                    [0, 1, 0],
                    [-sin(self.X[4]), 0, cos(self.X[4])]])

    r_z = np.array([[cos(self.X[5]), -sin(self.X[5]), 0],
                    [sin(self.X[5]), cos(self.X[5]), 0],
                    [0,0,1]])

    r_yx = np.matmul(r_y, r_x)
    return np.matmul(r_z, r_yx)


class Controller:
    
    def __init__(self,
                z_k_p=1.0,
                z_k_d=1.0,
                x_k_p=1.0,
                x_k_d=1.0,
                y_k_p=1.0,
                y_k_d=1.0,
                k_p_roll=1.0,
                k_p_pitch=1.0,
                k_p_yaw=1.0,
                k_p_p=1.0,
                k_p_q=1.0,
                k_p_r=1.0):
        
        
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.x_k_p = x_k_p
        self.x_k_d = x_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r
       
        self.g = 9.81
    
    def altitude_controller(self,
                       z_target,
                       z_dot_target,
                       z_dot_dot_target,
                       z_actual,
                       z_dot_actual,
                       rot_mat):
    
        def pd(kp, kd, error, error_dot, target):
            p_term = kp * error
            d_term = kd * error_dot
            return p_term + d_term + target
        
        u_1_bar = pd(self.z_k_p, self.z_k_d, 
                    error     = z_target - z_actual, 
                    error_dot = z_dot_target - z_dot_actual,
                    target    = z_dot_dot_target)

        b_z = rot_mat[2,2]
        c=(u_1_bar - self.g)/b_z
        return c

    def lateral_controller(self,
                      x_target,
                      x_dot_target,
                      x_dot_dot_target,
                      x_actual,
                      x_dot_actual,
                      y_target,
                      y_dot_target,
                      y_dot_dot_target,
                      y_actual,
                      y_dot_actual,
                      c):
    
        def pd(kp, kd, error, error_dot, target):
            # Proportional and differential control terms
            p_term = kp * error
            d_term = kd * error_dot
            
            # Control command (with feed-forward term)
            return p_term + d_term + target
        
        # Determine errors
        x_err = x_target - x_actual
        y_err = y_target - y_actual
        x_err_dot = x_dot_target - x_dot_actual
        y_err_dot = y_dot_target - y_dot_actual
        
        # Apply the PD controller
        x_dot_dot_command = pd(self.x_k_p, self.x_k_d, x_err, x_err_dot, x_dot_dot_target)
        y_dot_dot_command = pd(self.y_k_p, self.y_k_d, y_err, y_err_dot, y_dot_dot_target)

        # Determine controlled values by normalizing with the collective thrust
        b_x_c = x_dot_dot_command / c
        b_y_c = y_dot_dot_command / c
        return b_x_c, b_y_c

    def roll_pitch_controller(self,
                          b_x_c_target,
                          b_y_c_target,
                          rot_mat):
    
        def p(kp, error):
            return kp * error
        
        b_x = rot_mat[0,2]
        b_y = rot_mat[1,2]
        
        b_x_commanded_dot = p(self.k_p_roll, error=b_x_c_target - b_x)
        b_y_commanded_dot = p(self.k_p_pitch, error=b_y_c_target - b_y)

        rot_mat1 = np.array([[rot_mat[1,0], -rot_mat[0,0]], 
                            [rot_mat[1,1], -rot_mat[0,1]]]) / rot_mat[2,2]

        rot_rate = np.matmul(rot_mat1, np.array([b_x_commanded_dot, b_y_commanded_dot]).T)
        p_c = rot_rate[0]
        q_c = rot_rate[1]

        return p_c, q_c

    
    def yaw_controller(self,
                   psi_target,
                   psi_actual):
    
        def p(kp, error):
            return kp * error

        return p(self.k_p_yaw, error=psi_target - psi_actual)
if __name__ == '__main__':
    drone = DroneIn3D()    
    control_system = Controller(z_k_p=2.0,z_k_d=1.0,x_k_p=6.0,x_k_d=4.0,y_k_p=6.0,y_k_d=4.0,k_p_roll=8.0,k_p_pitch=8.0,
    k_p_yaw=8.0,k_p_p=20.0,k_p_q=20.0,k_p_r=20.0)   
    for i in range(0,z_path.shape[0]):

        rot_mat = drone.R()

        c = control_system.altitude_controller(z_path[i],
                                            z_dot_path[i],
                                            z_dot_dot_path[i],
                                            drone.X[2],
                                            drone.X[8],
                                            rot_mat)

        b_x_c, b_y_c = control_system.lateral_controller(x_path[i],
                                                        x_dot_path[i],
                                                        x_dot_dot_path[i],
                                                        drone.X[0],
                                                        drone.X[6],
                                                        y_path[i],
                                                        y_dot_path[i],
                                                        y_dot_dot_path[i],
                                                        drone.X[1],
                                                        drone.X[7],
                                                        c) 

        for j in range(inner_loop_relative_to_outer_loop):
            
            rot_mat = drone.R()
            p_c, q_c = self.roll_pitch_controller(b_x_c,
                                                b_y_c,
                                                rot_mat)
            
            r_c = self.yaw_controller(psi_path[i], 
                                    drone.psi)
            
