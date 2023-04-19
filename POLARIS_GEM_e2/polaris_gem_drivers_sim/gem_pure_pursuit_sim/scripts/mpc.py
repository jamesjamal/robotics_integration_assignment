#!/usr/bin/env python3

#================================================================
# File name: mpc_controller.py                                                                  
# Description: MPC controller for GEM vehicle in Gazebo                                                              
# Author: [Your Name]                                                          
# Date created: [Date]                                                                
# Date last modified: [Date]                                                          
# Version: 1.0                                                                    
# Usage: rosrun gem_mpc_controller mpc_controller.py                                                                    
# Python version: 3.8                                                             
#================================================================

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
from scipy.optimize import minimize

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

class MPC(object):
    
    def __init__(self):

        self.rate         = rospy.Rate(20)

        self.dt           = 0.1  # time step
        self.n            = 8    # number of states (x, y, yaw, v, cte, epsi, delta, a)
        self.m            = 2    # number of control inputs (delta, a)
        self.horizon      = 10   # time horizon
        self.w_cte        = 5000 # weight of cross-track error
        self.w_epsi       = 1000 # weight of orientation error
        self.w_delta      = 1    # weight of steering angle
        self.w_a          = 0.1  # weight of acceleration
        self.Lf           = 1.75 # wheelbase length
        self.max_delta    = math.radians(40) # max steering angle
        self.max_a        = 1.0  # max acceleration
        self.min_a        = -2.0 # min acceleration

        self.read_waypoints() # read waypoints

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0 
        self.ackermann_msg.steering_angle          = 0.0

        self.ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)


    # import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/wps.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_yaw = [float(point[2]) for point in path_points]


    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2), 3)


    # computes the orientation error between the vehicle and the desired path
    def epsi(self, psi, psides):
        e = psi - psides
        while e > math.pi:
            e -= 2 * math.pi
        while e < -math.pi:
            e += 2 * math.pi
        return round(e, 3)
        
        
        
        # computes the cross-track error between the vehicle and the desired path
    def cte(self, x, y, psi):
        min_dist = float('inf')
        min_index = 0
        for i in range(len(self.path_points_x)):
            d = self.dist((x, y), (self.path_points_x[i], self.path_points_y[i]))
            if d < min_dist:
                min_dist = d
                min_index = i
        if min_index < len(self.path_points_x) - 1:
            dx = self.path_points_x[min_index+1] - self.path_points_x[min_index]
            dy = self.path_points_y[min_index+1] - self.path_points_y[min_index]
            cte = (y - self.path_points_y[min_index]) - (dy/dx) * (x - self.path_points_x[min_index])
        else:
            cte = y - self.path_points_y[min_index]

        psi_des = math.atan2(dy, dx)
        epsi = self.epsi(psi, psi_des)

        return round(cte, 3), round(epsi, 3)
        
        
     


