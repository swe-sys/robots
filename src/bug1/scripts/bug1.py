#!/usr/bin/env python3

import math
from math import *
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class bug1:
    def __init__(self):
        self.vel_msg = Twist()
        self.goal = Point(6,3,0)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.obstacle_detector = rospy.Subscriber('/scan',LaserScan, self.obstacle_detector)
        self.get_odom = rospy.Subscriber('/odom', Odometry, self.odom_updater )
        self.mode = 0  # 0 is mode reach to goal
        self.ranges = LaserScan()
        self.gotoclst = False
        rospy.sleep(2)
        self.timer = rospy.Timer(rospy.Duration(0.15), self.main,oneshot=True)
        self.hit = None
    
    def odom_updater(self,odom):
        self.odom = odom
        self.x = odom.pose.pose.position.x 
        self.y = odom.pose.pose.position.y
        self.rot_q = odom.pose.pose.orientation
        euler = euler_from_quaternion([self.rot_q.x , self.rot_q.y , self.rot_q.z , self.rot_q.w ])
        
        self.yaw = euler[2]
        if self.yaw < 0:
            self.yaw += 2 * np.pi    

    def go2goal(self):
        rotation_angle = math.atan2((self.goal.y-self.y),(self.goal.x-self.x))
        # print(self.yaw, rotation_angle,"print1")
        if rotation_angle < 0:
            rotation_angle = rotation_angle + math.pi
        # print(self.yaw, rotation_angle,"print@")
        if abs(self.yaw - rotation_angle) > 0.2:
            self.vel_msg.angular.z = 0.2
            self.vel_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(self.vel_msg)
        elif self.hit_an_obstacle():
            print("No path exists")
            self.circum_navigate()   
        else:
            self.vel_msg.linear.x = 0.15
            self.vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.vel_msg)        

    def obstacle_detector(self,msg):
        self.ranges = msg.ranges

    def hit_an_obstacle(self):
        if self.ranges[0] < 0.5 or self.ranges[30] < 0.5 or self.ranges[330] < 0.5:
            if self.mode != 2:
                self.mode = 1
            if self.hit is None:
                self.hit = Point(self.x,self.y,0.0)
                self.old = rospy.get_time()
            return True
        else:
            return False

    def circum_navigate(self):
        """funct to follow wall boundary
        Turn Right by default or rotate on CCW fashion"""
        print("circum_navi")
        deg = 50
        dst = 0.5
        # while True:
        if min(self.ranges[0:deg]) <= dst or min(self.ranges[(359-deg):]) <= dst: # front wall  
            self.vel_msg.angular.z = -0.2
            self.vel_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(self.vel_msg)       
        elif min(self.ranges[deg:120]) < 1.1*dst: # left wall 
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = 0.2
            self.cmd_vel_pub.publish(self.vel_msg)
        #elif abs(dist([self.x,self.y],[self.goal.x,self.goal.y]) - min(self.odom_counter)) < 0.17:
            #self.go2goal()      
        else:
            self.vel_msg.angular.z = 0.17
            self.vel_msg.linear.x = 0.02
            self.cmd_vel_pub.publish(self.vel_msg)       


    def main(self,event):
        r = rospy.Rate(10)
        self.odom_counter = []
        while dist([self.x,self.y],[self.goal.x, self.goal.y]) > 0.5:            
            print(self.mode)            
            if self.mode == 0:
                self.go2goal()
            print(self.hit_an_obstacle(),"hit and obstacle hai")
            if self.mode == 1:
                self.circum_navigate()
                self.odom_counter.append(dist([self.x,self.y],[self.goal.x,self.goal.y]))
                if self.hit is not None:
                    print("inside the instance",self.hit,self.old)
                    # if self.in_between(self.hit,Point(self.x,self.y,0.0),self.old): #checks if 1 round is complete
                    if dist([self.x,self.y],[self.hit.x,self.hit.y]) < 1.0 and rospy.get_time() > self.old + 30:
                        self.mode = 2  # go to closest point in round 2
                        print("Now its Cycle 2")
            if self.mode == 2:
                if abs(dist([self.x,self.y],[self.goal.x,self.goal.y]) - min(self.odom_counter)) < 0.5 or self.gotoclst:
                    self.gotoclst = True
                    self.go2goal()                                       
                else:
                    self.circum_navigate()                    
            r.sleep()        
        self.vel_msg.angular.z = 0.0
        self.vel_msg.linear.x = 0.0
        # if  (self.mode == 3):
        #     print("Bug1 Fails")
        # else:
        #     print("Goal Reached")
        self.cmd_vel_pub.publish(self.vel_msg)
   
if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')     
    rospy.loginfo('BUGzzzzzzzzzz')      
    bug1()    
    rospy.spin()

