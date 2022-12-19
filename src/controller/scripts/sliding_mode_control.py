# !/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():

    def __init__(self):

        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)

        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False

        global omega
        omega = 0

        rospy.on_shutdown(self.save_data)

    def traj_coeff(self, t0, tf, pi, pf):

        # Calculating the coefficients
        A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5], [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4,],[0, 0, 2, 6*t0, 12*t0**2, 20*t0**3,],[1, tf, tf**2, tf**3, tf**4, tf**5],[0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],[0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

        # Velocity and Acceleration at each Waypoint is 0 i.e pi_dot, pi_ddot, pf_dot, pf_ddot = 0
        b = np.array([pi, 0, 0, pf, 0, 0])

        coeff = np.linalg.solve(A,b)

        return coeff
        

    def traj_evaluate(self):
        
        # Obtaining the coefficients for each co-ordinates of Waypoint
        if self.t<= 5: #P0 (0,0,0) to P1 (0,0,1)
            coeff_x = self.traj_coeff(0,5,0,0)
            coeff_y = self.traj_coeff(0,5,0,0)
            coeff_z = self.traj_coeff(0,5,0,1)

        elif self.t<= 20:  #P1 (0,0,1) to P2 (1,0,1)
            coeff_x = self.traj_coeff(5,20,0,1)
            coeff_y = self.traj_coeff(5,20,0,0)
            coeff_z = self.traj_coeff(5,20,1,1)

        elif self.t<= 35:  #P2 (1,0,1) to P3 (1,1,1)
            coeff_x = self.traj_coeff(20,35,1,1)
            coeff_y = self.traj_coeff(20,35,0,1)
            coeff_z = self.traj_coeff(20,35,1,1)

        elif self.t<= 50: #P3 (1,1,1) to P4 (0,1,1)
            coeff_x = self.traj_coeff(35,50,1,0)
            coeff_y = self.traj_coeff(35,50,1,1)
            coeff_z = self.traj_coeff(35,50,1,1)

        elif self.t<= 65: #P4 (0,1,1) to P5 (0,0,1)
            coeff_x = self.traj_coeff(50,65,0,0)
            coeff_y = self.traj_coeff(50,65,1,0)
            coeff_z = self.traj_coeff(50,65,1,1)

        else: # Zero input after 65 secs
            coeff_x = np.zeros(6)
            coeff_y = np.zeros(6)
            coeff_z = np.zeros(6)
            
        return np.array([coeff_x, coeff_y, coeff_z])

    def traj_gen(self):
        
        # Trajectory generation from obtained coefficient and time matrix 'T'
        coeff = self.traj_evaluate()
        T = np.transpose(np.array([[1, self.t, self.t**2, self.t**3, self.t**4, self.t**5],  [0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4], [0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3]]))

        traj_x = coeff[0] @ T
        xd = traj_x[0]
        xd_dot = traj_x[1]
        xd_ddot = traj_x[2]

        traj_y = coeff[1] @ T
        yd = traj_y[0]
        yd_dot = traj_y[1]
        yd_ddot = traj_y[2]

        traj_z = coeff[2] @ T
        zd = traj_z[0]
        zd_dot = traj_z[1]
        zd_ddot = traj_z[2]

        return xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot
    
    def sign_function(self, S):
        #sign function to restrict the value to -1, 0, 1 depending upon the value of S
        if S > 0:
            sgn = +1
        elif S < 0:
            sgn = -1
        else:
            sgn = 0
        return sgn

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):

        # obtain the desired values by evaluating the corresponding trajectories
        xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot = self.traj_gen()

        m = 27*(10**-3)
        l = 46*(10**-3)
        Ix = 16.57171*(10**-6)
        Iy = 16.57171*(10**-6)
        Iz = 29.261652*(10**-6)
        Ip = 12.65625 *(10**-8)
        kf = 1.28192*(10**-8)
        km = 5.964552*(10**-3)
        ak = 1/(4*kf)
        bk = sqrt(2)*ak/l        
        g = 9.8

        global omega

        #Tuned Parameters
        Kp = 30
        Kd = 5
        #    K1  K2  K3   K4
        K = [2, 140, 140, 5]
        lam = [2 , 12.5, 12.5, 5]
        
        # Current Position
        x = xyz[0,0]
        y = xyz[1,0]
        z = xyz[2,0]

        # Current Velocity 
        x_dot = xyz_dot[0,0]
        y_dot = xyz_dot[1,0]
        z_dot = xyz_dot[2,0]

        # Current RPY angle
        phi = rpy[0,0]
        theta = rpy[1,0]
        shi = rpy[2,0]

        # Current RPY velocity
        phi_dot = rpy_dot[0,0]
        theta_dot = rpy_dot[1,0]
        shi_dot = rpy_dot[2,0]

        print("------------------------- Time: ", round(self.t,2),"-------------------------------")

        # Z Input
        u1 = m * (zd_ddot + g - lam[0] * (z_dot - zd_dot)  - K[0] * self.sign_function((z_dot - zd_dot) + lam[0] * (z - zd))) * (1/(np.cos(phi)*np.cos(theta)))
        
        #Calculating the force in x and y direction
        Fx = m*(-Kp*(x-xd) - Kd*(x_dot-xd_dot) + xd_ddot)
        Fy = m*(-Kp*(y-yd) - Kd*(y_dot-yd_dot) + yd_ddot)
        
        # Restricting the forces between (-1, 1) to resolve math domain error
        force_x = max(min(Fx / u1,1),-1)
        force_y = max(min(Fy / u1,1),-1)
        theta_d = asin(force_x)
        phi_d   = asin(-force_y)

        # Phi 
        u2 = Ix*((-theta_dot*shi_dot*((Iy-Ix)/Ix))+Ip*omega*theta_dot/Ix-lam[1]*(phi_dot)-K[1]*self.sign_function(phi_dot + lam[1]*np.arctan2(np.sin(phi-phi_d),np.cos(phi-phi_d))))

        # Theta
        u3 = Iy*((-phi_dot*shi_dot*((Iz-Ix)/Iy))+Ip*omega*phi_dot/Ix-lam[2]*(theta_dot)-K[2]*self.sign_function(theta_dot + lam[2] * np.arctan2(np.sin(theta - theta_d),np.cos(theta - theta_d))))
         
        # Shi
        u4 = Iz*(((-phi_dot*theta_dot*((Ix-Iy)/Iz))-lam[3]*(shi_dot)-K[3]*self.sign_function(shi_dot + lam[3] * np.arctan2(np.sin(shi),np.cos(shi)))))

        u = np.array([u1,u2,u3,u4])

        # Allocation Matrix
        alloc = np.array([[ak,    -bk,    -bk,    -ak/km],[ak,     -bk,    bk,     ak/km],[ak,     bk,     bk,     -ak/km],[ak,     bk,     -bk,    ak/km]])

        # Rotor velocity calculations
        w = (alloc @ u)**(1/2)
        
        # Restricting the velocity below max value 2618
        for i in range(0,len(w)):
            w[i] = min(max(w[i],0),2618)
        
        omega = w[0] - w[1] + w[2] - w[3]
        motor_vel = w
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]]

        self.motor_speed_pub.publish(motor_speed)
              
    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])], [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])

        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

    # save the actual trajectory data
    def save_data(self):
        #Caution: Change the directory
        with open("/home/purvang/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control/src/controller/scripts/log.pkl", "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self. z_series], fp)


if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")