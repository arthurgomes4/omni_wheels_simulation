#! /usr/bin/python3

import rospy
import math as m
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class cmdVelocityNode:

    def __init__(self, node_name):
        
        # initialize the node
        rospy.init_node(node_name)

        # load params
        self.load()

        # compute H0 elements, these are required for twist to wheel speed conversion
        self.compute_H0_elements()

        # create the wheel velocity command publishers
        self.vel_command_pubs = []
        for topic in self.wheel_controller_command_topics:
            self.vel_command_pubs.append(rospy.Publisher(topic, Float64, queue_size=1))

        # subscribe to the main command velocity topic
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)

        rospy.loginfo('cmd_vel_node setup complete. Waiting for msgs on '+str(self.cmd_vel_topic))
        
        rospy.spin()

        rospy.loginfo('shutting down node')

    def load(self):

        # load parameters from the yaml file
        self.cmd_vel_topic = rospy.get_param('cmd_vel_topic','/cmd_vel')
        self.number_of_wheels = rospy.get_param('number_of_wheels',None)
        self.wheel_radius = rospy.get_param('wheel_radius',None)
        self.wheel_driving_angles = rospy.get_param('wheel_driving_angles',None)
        self.wheel_x_coords = rospy.get_param('wheel_x_coords',None)
        self.wheel_y_coords = rospy.get_param('wheel_y_coords',None)
        self.wheel_controller_command_topics = rospy.get_param('wheel_controller_command_topics',None)

        # check if all have been entered
        k = 0
        if self.number_of_wheels == None:
            rospy.logerr('parameter <number_of_wheels> not given')
            k = 1
        if self.wheel_radius == None:
            rospy.logerr('parameter <wheel_radius> not given')
            k = 1
        if self.wheel_driving_angles == None:
            rospy.logerr('parameter <wheel_driving_angles> not given')
            k = 1
        if self.wheel_x_coords == None:
            rospy.logerr('parameter <wheel_x_coords> not given')
            k = 1
        if self.wheel_y_coords == None:
            rospy.logerr('parameter <wheel_y_coords> not given')
            k = 1
        if self.wheel_controller_command_topics == None:
            rospy.logerr('parameter <wheel_controller_command_topics> not given')
            k = 1
        
        if k:
            exit(1)

    def cmd_vel_callback(self, vel_msg):
        
        # extract Vx, Vy, Wz from the twist message
        x_vel, y_vel, z_ang = vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z
        wheel_speeds = []

        # rospy.loginfo('Vx: '+str(x_vel)+' Vy: '+str(y_vel)+' Wz: '+str(z_ang))
        
        for i in range(self.number_of_wheels):

            speed = Float64()

            # calculate the required speed for wheel i
            speed.data = self.x_coeff[i]*x_vel + self.y_coeff[i]*y_vel + self.w_coeff[i]*z_ang

            # publish the wheel speed
            wheel_speeds.append(speed)

        for i in range(self.number_of_wheels):
            self.vel_command_pubs[i].publish(wheel_speeds[i])

    def compute_H0_elements(self):
 
        self.w_coeff = []
        self.x_coeff = []
        self.y_coeff = []
        for i in range(self.number_of_wheels):

            self.w_coeff.append((self.wheel_x_coords[i]*m.sin(self.wheel_driving_angles[i]) - self.wheel_y_coords[i]*m.cos(self.wheel_driving_angles[i]))/self.wheel_radius)

            self.x_coeff.append(m.cos(self.wheel_driving_angles[i])/self.wheel_radius)

            self.y_coeff.append(m.sin(self.wheel_driving_angles[i])/self.wheel_radius)

if __name__ == '__main__':
    cmdVelocityNode('cmd_vel_node')
        