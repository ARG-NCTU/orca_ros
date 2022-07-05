#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
import rvo2
import random
from dynamic_reconfigure.server import Server
import yaml
import math
import csv
import pickle as pkl
import numpy as np
import os
import tf
import random
import time

objs_dict = {
    "red_totem": 0.5,
    "green_totem": 0.5,
    "yellow_totem": 0.5,
    "black_totem": 0.5,
    "blue_totem": 0.5,
    "ball3": 0.3,
    "ball5": 0.5,
    "ball7": 0.7,
}

class WAMVORCA():
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        # initiallize boat status
        self.wamv_odom = Odometry()
        self.yaw = 0

        self.obs_pos_list = []
        self.obs_vertics_list = []
        with open("/home/argrobotx/robotx-2022/duckiepond-devices/obs_pos.yaml") as file:
            total_obs_list = yaml.safe_load(file)

        for key, value in total_obs_list.items():
            for pos_key, pos_value in value.items():
                self.obs_pos_list.append(pos_value)
        for i in range(len(self.obs_pos_list)):
            self.obs_vertics_list.append([self.obs_pos_list[i][0]+2, self.obs_pos_list[i][1]+2])
            self.obs_vertics_list.append([self.obs_pos_list[i][0]+2, self.obs_pos_list[i][1]-2])
            self.obs_vertics_list.append([self.obs_pos_list[i][0]-2, self.obs_pos_list[i][1]+2])
            self.obs_vertics_list.append([self.obs_pos_list[i][0]-2, self.obs_pos_list[i][1]-2])
        
        self.test_obs_list = [(-551, 215), (-551,216), (-551,217), (-551,218), (-551,219),(-551,220), \
        (-551,221), (-551,222),(-551,223),(-551,224),(-551,225),(-551,226),(-551,227), (-551,228), \
        (-551,229), (-551,230), (-551,231),(-551,232), (-551,233), (-551,234), (-551,235)]
        self.sub_odom = rospy.Subscriber(
            "odom_in", Odometry, self.cb_odom, queue_size=1)
        self.sub_goal = rospy.Subscriber(
            "goal_in", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)

        self.pub_cmd = rospy.Publisher("cmd_out", Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.start_orca)
        self.sim = rvo2.PyRVOSimulator(1/60., 1.5, 10, 1.5, 2, 3.5, 1)
        self.a0 = self.sim.addAgent((-536, 222.7))
        self.sim.setAgentPrefVelocity(self.a0, (1, 1))
        self.radius = 3.5
        self.dis_tre = 5
        self.sim.setAgentRadius(self.a0, self.radius)
        self.sim.addObstacle(self.obs_pos_list)
        # self.sim.addObstacle(self.obs_vertics_list)
        # self.sim.addObstacle(self.test_obs_list)
        self.sim.processObstacles()
        self.auto = 0
        self.goal = [[0, 0] for i in range(1)]
        self.robot_position = [[0, 0]]
        self.velocity = [[0, 0]for i in range(1)]
        
        self.v_max = [1 for i in range(1)]


    def compute_V_des(self, X, goal, V_max):
        V_des = []
        for i in range(len(X)):
            dif_x = [goal[i][k]-X[i][k] for k in range(2)]
            norm = self.get_distance(dif_x, [0, 0])
            norm_dif_x = [dif_x[k]*V_max[i]/norm for k in range(2)]
            V_des.append(norm_dif_x[:])
            if self.reach(X[i], goal[i], 0.1):
                V_des[i][0] = 0
                V_des[i][1] = 0
        return V_des

    def reach(self, p1, p2, bound=0.5):
        if self.get_distance(p1,p2)< bound:
            return True
        else:
            return False

    def start_orca(self, event):
        if self.goal[0][0] == 0 and self.goal[0][1] == 0:
            return 
        if self.auto == 0:
            return
        self.update_all()
        self.velocity = self.compute_V_des(self.robot_position, self.goal, self.v_max)
        for i in range(1):
            self.sim.setAgentPosition(self.a0, (self.robot_position[i][0], self.robot_position[i][1]))
            self.sim.setAgentPrefVelocity(self.a0, (self.velocity[i][0], self.velocity[i][1]))
            print("vx: ", self.velocity[i][0], "vy: ", self.velocity[i][1])
            self.sim.doStep()
            velocity_get = self.sim.getAgentVelocity(self.a0)
            print("velocity get: vx: ", velocity_get[0], "vy: ", velocity_get[1])
            obs_num = self.sim.getAgentNumObstacleNeighbors(self.a0)
            print("obs num: ", obs_num)


            dis, angle = self.process_ang_dis(velocity_get[0], velocity_get[1], self.yaw)
            # positions = ['(%5.3f, %5.3f)' % self.sim.getAgentPosition(self.a0)]
            # print("orca pos:", positions)
            # print("dis: ", dis, "angle: ", angle)

            goal_distance = self.get_distance(self.robot_position[i], self.goal[i])
            if(goal_distance < self.dis_tre):
                print("goal reach")
                # self.sim.setAgentPrefVelocity(self.a0, (0, 0))
                self.goal[i] = [0, 0]
                cmd = Twist()
                cmd.linear.x = 0
                cmd.angular.z = 0
                self.pub_cmd.publish(cmd)
                return

            cmd = Twist()
            cmd.linear.x = dis * 0.35
            cmd.angular.z = angle * 0.95

        self.pub_cmd.publish(cmd)
        

    def cb_joy(self, msg):
        start_button = 7
        back_button = 6

        if (msg.buttons[start_button] == 1) and not self.auto:
            self.auto = 1
            rospy.loginfo('go auto')
        elif msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            rospy.loginfo('go manual')

    def process_ang_dis(self, vx, vy, yaw):
        dest_yaw = math.atan2(vy, vx)

        angle = dest_yaw - yaw
        if angle > np.pi:
            angle = angle-2*np.pi

        if angle < -np.pi:
            angle = angle+2*np.pi

        angle = angle/np.pi

        dis = math.sqrt(vx**2+vy**2)

        dis = max(min(dis, 1), -1)
        angle = max(min(angle, 1), -1)
        return dis, angle
    
    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def update_all(self):
        self.robot_position = []

        # update position
        pos = [self.wamv_odom.pose.pose.position.x,
               self.wamv_odom.pose.pose.position.y]
        self.robot_position.append(pos)

        # update orientation
        quaternion = (self.wamv_odom.pose.pose.orientation.x,
                      self.wamv_odom.pose.pose.orientation.y,
                      self.wamv_odom.pose.pose.orientation.z,
                      self.wamv_odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def cb_odom(self, msg):
        self.wamv_odom = msg
    
    def cb_goal(self, msg):
        self.goal[0] = np.array([
            msg.pose.position.x, msg.pose.position.y])


if __name__ == "__main__":
    rospy.init_node("WamvORCA")
    wamvORCA = WAMVORCA()
    rospy.spin()
