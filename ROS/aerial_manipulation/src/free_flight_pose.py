#!/usr/bin/env python

'''
SUBSCRIBES TO:
/raven/msf_core/odometry:     Pose of Quadrotor COM (in ground frame)   variable name: quadrotor_pose_gf
/contact_mode:          Contact mode as defined in force_to_mode.py
/desired_pose_gf:       Desired Pose of the quadrotor (for transition) (in ground frame)

PUBLISHES:
/raven/command/pose:      Sample Desired Pose of Quadrotor COM to Desired Point only if contact_mode==False (CHECK NODE ARGUMENTS)

'''

import rospy

import math

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import RollPitchYawrateThrust
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Quaternion

import sys


contact_mode = False #initialize mode to free-flight
trial_pose_gf = PoseStamped()
quadrotor_pose_gf = Odometry()

def callback_mode(msg):
    global contact_mode
    contact_mode = msg.data


def callback_quadrotor_pose_gf(msg):
    global quadrotor_pose_gf
    quadrotor_pose_gf = msg

def make_trial_pose_gf(x_input,y_input,z_input):
    global trial_pose_gf
    global quadrotor_pose_gf

    trial_pose_gf.header = quadrotor_pose_gf.header
    trial_pose_gf.header.frame_id = 'trial_pose'
    trial_pose_gf.pose.position.x = float(x_input)
    trial_pose_gf.pose.position.y = float(y_input)
    trial_pose_gf.pose.position.z = float(z_input)
    trial_pose_gf.pose.orientation.x = 0.0
    trial_pose_gf.pose.orientation.y = 0.0
    trial_pose_gf.pose.orientation.z = 0.0
    trial_pose_gf.pose.orientation.w = 1.0
    #print(float(z_input))



def main(x_input,y_input,z_input):
    global trial_pose_gf
    global quadrotor_pose_gf
    global contact_mode

    rospy.init_node('free_flight_pose')
    rospy.Subscriber("/contact_mode",Bool,callback_mode)                              #("subscribed topic name",message class, callback)  The callback records the subscribed data and processes it
    rospy.Subscriber("/raven/msf_core/odometry",Odometry,callback_quadrotor_pose_gf)
    pubtp = rospy.Publisher('/raven/command/pose',PoseStamped, queue_size=10)          #("name of published topic". message class, queue size)

    while not rospy.is_shutdown() and not contact_mode: #disable second part of conditional only for free-flight testing
        #do stuff here
        #replace trial pose with desired pose for actual test
        make_trial_pose_gf(x_input,y_input,z_input)

        #publish
        pubtp.publish(trial_pose_gf)                                   #argument is the variable that you want to publish, should conform with message class eg:string
    #common for all nodes
    rospy.spin()



#common for all nodes
if __name__ == '__main__':
    try:
        main(sys.argv[1],sys.argv[2],sys.argv[3])
    except rospy.ROSInterruptException:
        pass
