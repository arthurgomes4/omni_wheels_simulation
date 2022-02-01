#! /usr/bin/env python

import rospy 
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class odometryNode:

    def __init__(self, node_name):

        rospy.init_node(node_name)

        self.load()

        self.compute_inverse_H0_matrix()

        rospy.Subscriber(self.wheel_state_topic, JointState, self.joint_state_callback)
        self.odom_publisher = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)

        rospy.loginfo('odom_node setup complete.')
        rospy.spin()
        rospy.loginfo('odom_node shutting down')

    def load(self):

        # load parameters from the yaml file
        self.cmd_vel_topic = rospy.get_param('cmd_vel_topic','/cmd_vel')
        self.number_of_wheels = rospy.get_param('number_of_wheels',None)
        self.wheel_radius = rospy.get_param('wheel_radius',None)
        self.wheel_driving_angles = rospy.get_param('wheel_driving_angles',None)
        self.wheel_x_coords = rospy.get_param('wheel_x_coords',None)
        self.wheel_y_coords = rospy.get_param('wheel_y_coords',None)

        self.robot_start_config = rospy.get_param('robot_start_config', [0,0,0])
        self.wheel_state_topic = rospy.get_param('wheel_state_topic', None)

        self.odom_topic = rospy.get_param('odom_topic','/odom')

        self.prev_time_stamp = None

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
        if self.wheel_state_topic == None:
            rospy.logerr('parameter <wheel_state_topic> not given')
            k = 1

        if k:
            exit(1)


    def compute_inverse_H0_matrix(self):
        
        self.compute_H0_elements()

        temp = np.array([[ a for a in self.w_coeff],
                         [ a for a in self.x_coeff],
                         [ a for a in self.y_coeff]])

        H0 = np.transpose(temp)

        self.H0_inv = np.linalg.pinv(H0,rcond=1e-6)

    def compute_H0_elements(self):
 
        self.w_coeff = []
        self.x_coeff = []
        self.y_coeff = []
        for i in range(self.number_of_wheels):

            self.w_coeff.append((self.wheel_x_coords[i]*np.sin(self.wheel_driving_angles[i]) - self.wheel_y_coords[i]*np.cos(self.wheel_driving_angles[i]))/self.wheel_radius)

            self.x_coeff.append(np.cos(self.wheel_driving_angles[i])/self.wheel_radius)

            self.y_coeff.append(np.sin(self.wheel_driving_angles[i])/self.wheel_radius)

    def stamp_to_seconds(self, stamp):
        return float(stamp.secs) + float(stamp.nsecs)/1000000000

    def joint_state_callback(self, joint_state_msg):
        
        if self.prev_time_stamp == None:
            self.prev_time_stamp = self.stamp_to_seconds(joint_state_msg.header.stamp)
            return

        current_time_stamp = self.stamp_to_seconds(joint_state_msg.header.stamp)
        wheel_speeds = np.array(joint_state_msg.velocity)
        chassis_velocity = np.matmul(self.H0_inv, np.transpose(wheel_speeds))

        print('=================')
        print(current_time_stamp)
        print('-----')
        print(wheel_speeds)
        print('-----')
        print(chassis_velocity)

        pass

if __name__ == '__main__':
    odometryNode('odom_node')

