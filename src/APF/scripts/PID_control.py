#!/usr/bin/env python3

import math
import numpy as np
from geometry_msgs.msg import Pose2D,  Twist
from nav_msgs.msg import Odometry
import rospy
from tf.transformations import euler_from_quaternion

pi = np.pi
class PID_control():
    def __init__(self,PID_name,kv = 1.2 , kalpha = 1.1,publisher_name='/cmd_vel', odom_name='/odom'):
        self.name = PID_name
        self.kv = kv
        self.ka = kalpha
        self.vmax = 0.22
        self.wmax = pi/2.5
        self.current_time = rospy.get_time()
        self.pub_cmd_vel = rospy.Publisher(publisher_name,Twist,queue_size=1)
        self.bot_location = Pose2D()
        self.bot_vel = Twist()
        rospy.Subscriber(odom_name, Odometry, self.callback_odom)


    def callback_odom(self, data):
        self.bot_location.x = data.pose.pose.position.x
        self.bot_location.y = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        if yaw < 0:
            yaw += 2 * pi
        self.bot_location.theta = yaw
        self.bot_vel.linear.x = data.twist.twist.linear.x
        self.bot_vel.linear.y = data.twist.twist.linear.y
        self.bot_vel.angular.z = data.twist.twist.angular.z

    def E_alpha_i(self,goal_pose,cur_pose):

        alpha_i = math.atan2((goal_pose.y-cur_pose.y),(goal_pose.x - cur_pose.x))
        # print(alpha_i)
        if alpha_i < 0:
            alpha_i += 2*pi
        return alpha_i - cur_pose.theta
    
    def Dist(self,goal_pose,cur_pose):
        return math.sqrt((goal_pose.x - cur_pose.x)**2 + (goal_pose.y - cur_pose.y)**2)

    def Velocity_tracking_law(self,position):
        # print(goal_pose)
        """ Tracking law
        V = kv*cos(e_alpha)*D_i
        W_i = ka*e_alpha + alpha_dot"""

        goal_pose = Pose2D(position[0],position[1],0.0)

        e_alpha = self.E_alpha_i(goal_pose, self.bot_location)
        if e_alpha < -pi:
            e_alpha += 2*pi
        elif e_alpha > pi:
            e_alpha -= 2*pi

        D_i = self.Dist(goal_pose, self.bot_location)

        V = self.kv*math.cos(e_alpha)*D_i

        W_i = self.ka*e_alpha + goal_pose.theta

        if V>self.vmax:
            V=self.vmax
        # if W_i > self.wmax:
        #     W_i = V/0.2
        cmd_vel = Twist()
        cmd_vel.linear.x = V
        cmd_vel.angular.z = W_i
        print(cmd_vel)
        return cmd_vel
    
    def simon_go(self,x,y):
        v_des = np.arctan2(y-self.bot_location.y,x-self.bot_location.x)
        param_points.x = 0.2*np.cos(v_des) + self.bot_location.x
        param_points.y = 0.2*np.sin(v_des) + self.bot_location.y
        param_points.theta = 0
        
        cmd_vel = self.Velocity_tracking_law([param_points.x,param_points.y])
        # print(param_points,cmd_vel)
        self.pub_cmd_vel.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('PID_control')
    PID = PID_control("circle",publisher_name="/cmd_vel",odom_name="/odom")
    r=rospy.Rate(50)
    param_points = Pose2D()
    rad = 2
    theta = -pi
    while not rospy.is_shutdown():
        
        # PID.simon_go(-2,0.)
        PID.pub_cmd_vel.publish(PID.Velocity_tracking_law([2.,5.]))  
        # print(PID.bot_location)
           
        r.sleep()
