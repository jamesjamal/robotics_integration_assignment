#!/usr/bin/env python3

#================================================================
# File name: class_mpc.py                                                                  
# Description: mpc  controller for GEM vehicle in Gazebo                                                              
# Author: James
# Email: jamespmnayak@gmail.com                                                                     
                                                      
# Version: 0.1                                                                    
                                                     
# Python version: 3.8                                                             
#================================================================

# Python Headers
import os 
import csv
import math
import numpy as np
import cvxpy as cp
from numpy import linalg as la
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import sys
sys.path.append("../../PathPlanning/CubicSpline/")
import cubic_spline_planner
import random
from operator import add

import sys
import time

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState


import warnings
warnings.filterwarnings('ignore')

# Define the State class

road = False
save_simulation = False 
path_planning = True

Nu = 2 #number of control inputs

DT=0.2
dt=0.2
W_track = 1.2 #wheel track in metre
W_base = 1.75 #wheel base in metre


H = 5 # Horizon length
    
accept_dist = 100 #acceptable destination distance
accept_stop_v = 0.05 #acceptable stopping velocity


max_speed = 15              # m/s
max_reverse_speed = 5       # m/s
max_steer_angle = np.pi / 4     #max steering angle
max_steer_rate = np.pi / 6      #max steering speed
max_acc = 2                     #maximum acceleration m/s^2

W1 = np.array([1.01, 1.01])  # input weightage
W2 = np.array([1.0, 1.0, 1.5, 1.5])  # state error weightage
W3 = np.array([0.01, 0.01])  # rate of input change weightage
W4 = W2  # state error weightage
class State:
    def __init__(self,x_state,y_state,yaw,velocity):
        self.x = x_state
        self.y = y_state
        self.yaw = yaw
        self.v = velocity
        
    
    #Update state variable for given acceleration and steering angle
    def update_state(self, acc, steer_angle):

        self.x = self.x + self.v * np.cos(self.yaw) * dt
        self.y = self.y + self.v * np.sin(self.yaw) * dt
        self.yaw = self.yaw + (self.v / W_base) * np.tan (steer_angle) * dt
        self.v = self.v + acc * dt

    def state_to_vec(self):
        state_vec=np.array([self.x, self.y, self.yaw, self.v])
        return state_vec
    #end  

# Define the MPC class
class MPC:

    
    def __init__(self):
        self.ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)
        self.odom_sub = rospy.Subscriber('/gem/base_footprint/odom', Odometry, self.odom_callback)
        
        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle= 0.0
        self.rate = rospy.Rate(10) # 10 Hz
        self.H = 5 #Horizon 
        self.N_search = 5 # search for closest point in the next N_search points on the path
        self.simulation_time_limit = 100 #seconds
        self.Nx = 4 #number of states
        self.desired_speed = 5           # m/s
        self.dt=0.2
        self.W_base =1.75
        self.gem_pos_x =0
        self.gem_pos_y= 0
        self.gem_pos_yaw =0
        self.path_gem_x=[0]
        self.path_gem_y=[0]
        self.ct_error=[0]
        self.t=[0]



    def odom_callback(self, msg):
        self.gem_pos_x =msg.pose.pose.position.x
        self.gem_pos_y =msg.pose.pose.position.y
     # This function will be called every time a new message is received
        orientation_quat = msg.pose.pose.orientation
        orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        #print(self.gem_pos_x)
        self.gem_pos_yaw = yaw
        

    # Define the dynamic model function
    def dynamic_model(self, velocity, yaw, steer):
        A=np.array([[1.0 , 0.0 , - self.dt * velocity * math.sin(yaw), self.dt * math.cos(yaw) ],\
                    [0.0 , 1.0 , self.dt * velocity * math.cos(yaw),  self.dt * math.sin(yaw)],\
                    [0.0 , 0.0 , 1.0                , self.dt * math.tan(steer) / self.W_base],\
                    [0.0 , 0.0 , 0.0 , 1.0]])

        B=np.array([[0.0 , 0.0 ],\
                    [0.0 , 0.0 ],\
                    [0.0 , self.dt * velocity / (self.W_base * math.cos(steer) ** 2) ],\
                    [self.dt  , 0.0]])

        C=np.array([self.dt * velocity * math.sin(yaw) * yaw,\
                     - self.dt * velocity * math.cos(yaw) * yaw ,\
                    - self.dt * velocity * steer / (self.W_base * math.cos(steer) ** 2) ,\
                    0.0])
        return A, B, C

    # Define the function to calculate the predicted trajectory
    def calc_predicted_trajectory(self, acc,steer,cur_state_vec):
        traj_pred = np.zeros((self.Nx,self.H+1))
        traj_pred[:,0]=cur_state_vec.T
        pred_state=State(cur_state_vec[0], cur_state_vec[1], cur_state_vec[2], cur_state_vec[3])
        for i in range(self.H):
            pred_state.update_state(acc[i], steer[i])
            temp_state=pred_state.state_to_vec()
            traj_pred[:,i+1]=temp_state.T
        return traj_pred
            
    def get_straight_course(self, dl):
        ax = [0.0, 30.0]
        ay = [0.0, 1.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

        return cx, cy, cyaw
        
    def get_right_turn(self,dl):
        ax = [0.0, 10.0, 15.0, 20.0, 20.0, 20.0, 20.0]
        ay = [0.0, 1.0, 0.0, 0.0, 5.0, 10.0, 20.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw

        
    def stop_planning(self,path_x,path_y):
        dist_to_dest = (path_x[-1]-gx)**2+(path_y[-1]-gy)**2
        if dist_to_dest < accept_dist:
            return True
        else:
            return False
            
            


    
            
            
    def initialize_obstacles(self,NUM_OF_OBSTACLES):
        

        coordinateX=[]
        coordinateY=[]
        velX =[]
        velY =[]
    
        for i in range(1,NUM_OF_OBSTACLES):
            coordinateX.append(random.randrange(10, gx, 1))
            coordinateY.append(random.randrange(10, gy, 1))
            velX.append((np.random.random()/40)*(-1)**i)
            velY.append((np.random.random()/40)*(-1)**i)
        
        ox = coordinateX  
        oy = coordinateY 
        return ox,oy,velX,velY
        
        
    def get_closest_point_on_path(self,path_x, path_y, cur_state_vec, point):
        next_x = path_x[point:point+self.N_search]
        next_y = path_y[point:point+self.N_search]
        diff_x = next_x-cur_state_vec[0]
        diff_y = next_y-cur_state_vec[1]
        dist_sq = (diff_x)**2 + (diff_y)**2
        min_d = min(dist_sq)
        temp=np.argwhere(dist_sq==min_d)
        target_pt = int(temp[0,0]) + point
        return target_pt  
        
        
    def cal_desired_trajectory(self, cur_state_vec, path_x, path_y, dist_step, path_yaw, target_pt):
        traj_des = np.zeros((self.Nx,self.H+1))
        steer_des = np.zeros((1,self.H+1))
        distance = 0
        total_pts = len(path_x)

        target_pt = self.get_closest_point_on_path(path_x, path_y, cur_state_vec, target_pt)

        traj_des[0,0] = path_x[target_pt]
        traj_des[1,0] = path_y[target_pt]
        traj_des[2,0] = path_yaw[target_pt]
        traj_des[3,0] = self.desired_speed

        for i in range(self.H):
            distance += abs(cur_state_vec[3]) * self.dt
            pts_travelled = int(round(distance/dist_step))

            if (target_pt+pts_travelled)<total_pts:
                traj_des[0,i+1] = path_x[target_pt + pts_travelled]
                traj_des[1,i+1] = path_y[target_pt + pts_travelled]
                traj_des[2,i+1] = path_yaw[target_pt + pts_travelled]
                if (target_pt+pts_travelled) == total_pts - 1:
                    traj_des[3,i+1] = 0.0
                else:
                    traj_des[3,i+1] = self.desired_speed
            else:
                traj_des[0,i+1] = path_x[-1]
                traj_des[1,i+1] = path_y[-1]
                traj_des[2,i+1] = path_yaw[-1]
                traj_des[3,i+1] = 0.0
        if traj_des[3,1] == 0.0:
            traj_des[3,0] = 0.0
        return traj_des, target_pt, steer_des
        
        
    def destination_check(self,state, goal, target_pt, length_path):
        a=0
        dist_to_dest = (state.x-goal[0])**2+(state.y-goal[1])**2
        if dist_to_dest < accept_dist:
            a += 1
        if state.v < abs(accept_stop_v):
            a += 1
        if abs(target_pt - length_path) < 5:
            a += 1
        if a==3:
            return True
        return False
        
        
        
    def run_MPC(self,traj_des, cur_state_vec, mpc_acc, mpc_steer, steer_des, goal):

        for iter in range(3):
            traj_pred = self.calc_predicted_trajectory(mpc_acc,mpc_steer,cur_state_vec)
            x = cp.Variable([self.Nx, H+1])
            u = cp.Variable([Nu, H])

            cost = 0.0
            constraints = []
            for i in range(H):
                cost += cp.sum(W1 * cp.square(u[:, i]))                                   # input weightage
                cost += cp.sum(W2 * cp.square(traj_des[:, i] - x[:, i]))                  # state error weightage
            #cost += cp.sum(W2 * cp.square([goal[0],goal[1],0,0] - x[:, i]))                  # terminal cost
                if i < (H - 1):
                    cost += cp.sum(W3 * cp.square(u[:, i+1] - u[:, i]))                    # rate of input change weightage
                    constraints += [cp.abs(u[1, i+1] - u[1, i]) <= max_steer_rate * dt]
            
                A,B,C = self.dynamic_model(traj_pred[3,i], traj_pred[2,i], mpc_steer[i])
                constraints += [x[:, i+1] == A * x[:, i] + B * u[:, i] + C]
        
        
            cost += cp.sum(W4 * cp.square(traj_des[:, H] - x[:, H]))                      # final state error weightage
        #cost += cp.sum(10 * cp.square([goal[0],goal[1]] - x[:2, H]))                  # terminal cost
       
            constraints += [x[:, 0] == cur_state_vec]
            constraints += [x[3, :] <= max_speed]
            constraints += [x[3, :] >= -max_reverse_speed]
            constraints += [u[1, :] <= max_steer_angle]
            constraints += [u[1, :] >= -max_steer_angle]
            constraints += [u[0, :] <= max_acc]
            constraints += [u[0, :] >= -3*max_acc] 

            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve()

            mpc_x = x.value[0, :]
            mpc_y = x.value[1, :]
            mpc_yaw = x.value[2, :]
            mpc_v = x.value[3, :]
            mpc_acc = u.value[0, :]
            mpc_steer = u.value[1, :]
            lyap_val=0
            lap_u=0
            lap_x=0
            lap_du=0
            for i in range(H):
                lyap_val += np.sum(W1 * np.square(u.value[:, i]))                                   # input weightage
                lap_u += np.sum(W1 * np.square(u.value[:, i]))
                lyap_val += np.sum(W2 * np.square(traj_des[:, i] - x.value[:, i]))                  # state error weightage
                lap_x += np.sum(W2 * np.square(traj_des[:, i] - x.value[:, i]))
                if i < (H - 1):
                    lyap_val += np.sum(W3 * np.square(u.value[:, i+1] - u.value[:, i]))                    # rate of input change weightage
                    lap_du += np.sum(W3 * np.square(u.value[:, i+1] - u.value[:, i]))
            lyap_val += np.sum(W4 * np.square(traj_des[:, H] - x.value[:, H]))
            lap_x += np.sum(W4 * np.square(traj_des[:, H] - x.value[:, H]))
        #yap_val += np.sum(W2 * np.square(x.value[:, 1]))
            
            aaaa=5
        return mpc_x, mpc_y, mpc_yaw, mpc_v, mpc_acc, mpc_steer, lyap_val, lap_u, lap_x, lap_du
        
        
        
    def run_controller(self,path_x, path_y, path_yaw,  \
                    dist_step, initial_state, goal,\
                    ox, oy, velX, velY, path_planning):
        current_state = initial_state
        imgct=0
    #Initialize variables to store actual state values of car
        x = [current_state.x]
        y = [current_state.y]
        yaw = [current_state.yaw]
        vel = [current_state.v]
        t = [0]
        steer = [0]
        acc = [0]
        lyap=[0]
        lu=[0]
        lx=[0]
        ldu=[0]
        

        mpc_acc = np.zeros(self.H)
        mpc_steer = np.zeros(self.H)
        cur_state_vec=current_state.state_to_vec()
        target_pt = self.get_closest_point_on_path(path_x, path_y, cur_state_vec, 0)
        while t[-1] <= self.simulation_time_limit:

            imgct +=1
            print(imgct)
            cur_state_vec = current_state.state_to_vec()
        
            traj_des, target_pt, steer_des = self.cal_desired_trajectory(cur_state_vec, path_x, path_y,dist_step, path_yaw,target_pt)
        
            mpc_x, mpc_y, mpc_yaw, mpc_v, mpc_acc, mpc_steer, lyap_val, lap_u, lap_x, lap_du = self.run_MPC(traj_des, cur_state_vec, mpc_acc, mpc_steer, steer_des, goal)  


            current_state.update_state(mpc_acc[0], mpc_steer[0])
        
            #print("the steering angle "  +  str(round(mpc_steer[0],3)))
            vehicle_angle= round(mpc_steer[0],3)
            self.ackermann_msg.speed = current_state.v
            self.ackermann_msg.steering_angle= vehicle_angle
            self.ackermann_pub.publish(self.ackermann_msg)
            
            L= round(np.sqrt((self.gem_pos_x-path_x[target_pt])**2 + (self.gem_pos_y-path_y[target_pt] )**2))
            
            alpha   = mpc_yaw[0] - (self.gem_pos_yaw)
            self.ct_error.append(round(np.sin(alpha) * L, 3))
            
            
            print("cross track error : " + str(round(np.sin(alpha) * 0.08, 3)))
#            print(mpc_yaw[0])
            print(target_pt)
            
            time = t[-1] + self.dt
            self.t.append(time)
            x.append(self.gem_pos_x)
            y.append(self.gem_pos_y)
            yaw.append(self.gem_pos_yaw)
            vel.append(current_state.v)
            steer.append(mpc_steer[0])
            acc.append(mpc_acc[0])
            lyap.append(lyap_val)
            lu.append(lap_u)
            lx.append(lap_x)
            ldu.append(lap_du)
            
            self.path_gem_x.append(self.gem_pos_x)
            self.path_gem_y.append(self.gem_pos_y)

            if self.destination_check(current_state, goal, target_pt, len(path_x)):
                print("Reached destination close the graph")
                break

            ox=np.add(ox,velX)
            oy=np.add(oy,velY)

            plt.cla()
            plt.plot(mpc_x, mpc_y, "xr", label="MPC")
            plt.plot(path_x, path_y, "-r", label="course")
            plt.plot(ox,oy,'ok')
            plt.plot(x, y, "ob", label="trajectory")

            plt.plot(traj_des[0, :], traj_des[1, :], "xk", label="xref")
            plt.plot(path_x[target_pt], path_y[target_pt], "xg", label="target")
#        plot_car(current_state.x, current_state.y, current_state.yaw, mpc_steer[0])
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                    + ", speed[m/s]:" + str(round(current_state.v , 2)))


 

        return t, x, y, yaw, vel, steer, acc, lyap, lu, lx, ldu
    #end       

            
    def main_only_mpc(self):
        print("I am gonna follow a stright line ")
        dist_step= 1

        path_planning = False
        path_x,path_y,path_yaw = self.get_straight_course(dist_step)

        
        ox,oy,velX,velY = self.initialize_obstacles(0)
        initial_state= State(self.gem_pos_x , self.gem_pos_y, 0.01, 0.0)
        goal = np.array([path_x[-1], path_y[-1]])

        t, x, y, yaw, vel, steer, acc, lyap, lu, lx, ldu = self.run_controller(path_x, path_y, path_yaw, \
                                                    dist_step, initial_state, goal, \
                                                    ox, oy, velX, velY, path_planning)
                                                    
#        print("lenght of gem_x and gem_y : " + str(len(path_gem_x)) + " " + str(len(path_gem_y)))
                                                    
        plt.close("all")
        plt.subplots()
        plt.plot(path_x, path_y, "-r", label="spline")
        plt.plot(self.path_gem_x, self.path_gem_y, "--g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.show()
        
        
        plt.subplots()
        plt.plot(self.t, self.ct_error, "-b", label="cross track error")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("Time [s]")
        plt.ylabel("yaw")
        plt.legend()
        plt.show()
                                                    
                                                    

                               
if __name__ == '__main__':
    rospy.init_node('mpc_controller')
    mpc = MPC()
    mpc.main_only_mpc()
    rospy.spin()
