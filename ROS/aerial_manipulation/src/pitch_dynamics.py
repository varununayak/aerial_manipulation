#!/usr/bin/env python

'''
SUBSCRIBES TO:
/raven/msf_core/odometry:     Pose of Quadrotor COM in ground frame: to check whether reference has been achieved
/desired_pose_gf:             Pose of Desired state of Quadrotor COM (in ground frame)
/contact_mode:                To check whether the system is in manipulation mode OR free-flight mode

PUBLISHES:
/raven/command/pose:    Pitch angle for manipulation while holding altitude (for mav_control)------- ONLY IF contact_mode==TRUE AND reference_achieved()==TRUE
                        control input variable name is manipulation_pose (PoseStamped)


The pitch reference angle is calculated using the non-linear passivity based PD controller. Do all this only if it is in contact mode i.e. contact_mode == True

Ensure that the altitude is held

TODO: reference_achieved checks whether the drone arrives at particular state within an error boundary.
It must reach required position for manipulation along with zero yaw, zero roll, all rates zero and some finite pre-defined pitch angle say 10 degrees.

'''

import rospy
import math

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import RollPitchYawrateThrust
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Quaternion


#initializations
contact_mode = False #initialize mode to free-flight
desired_pose_gf = PoseStamped() #the desired pose of the quadrotor just before manipulation (as of now, pitch angle is zero)
quadrotor_pose_gf = Odometry() #the pose of the quadrotor - published in /raven/msf_core/odometry
manipulation_pose = PoseStamped() #the pose of the quadrotor during manipulation


#callbacks for all subscribed topics

def callback_mode(msg):
    global contact_mode
    contact_mode = msg.data


def callback_desired_pose_gf(msg):
    global desired_pose_gf
    desired_pose_gf = msg

def callback_quadcom_pose_gf(msg):
    global quadrotor_pose_gf
    quadrotor_pose_gf = msg


def generate_manipulation_pose():
    global manipulation_pose
    global desired_pose_gf

    pitch_angle_manipulation = 14*math.pi/180 #14 degrees pitch angle for manipulation

    manipulation_pose.header = desired_pose_gf.header
    manipulation_pose.header.frame_id  = "wf"
    manipulation_pose.position.x = desired_pose_gf.position.x
    manipulation_pose.position.y = desired_pose_gf.position.y
    manipulation_pose.position.z = desired_pose_gf.position.z
    manipulation_pose.orientation.x = 0
    manipulation_pose.orientation.y = 1*math.sin(pitch_angle_manipulation/2)
    manipulation_pose.orientation.z = 0
    manipulation_pose.orientation.w = math.cos(pitch_angle_manipulation/2)



def reference_achieved():
    #this function returns True if quadrotor COM pose is equal to the desired pose (with some allowable eerr
    global quadrotor_pose_gf
    global desired_pose_gf

    #Euclidean norm of error for position and orientation
    errorsquar_pos = (quadrotor_pose_gf.pose.pose.position.x - desired_pose_gf.pose.position.x)^2 +   (quadrotor_pose_gf.pose.pose.position.y - desired_pose_gf.pose.position.y)^2 +  (quadrotor_pose_gf.pose.pose.position.z - desired_pose_gf.pose.position.z)^2
    errorsquar_ori = (quadrotor_pose_gf.pose.pose.orientation.x - desired_pose_gf.pose.orientation.x)^2 + (quadrotor_pose_gf.pose.pose.orientation.y - desired_pose_gf.pose.orientation.y)^2 + (quadrotor_pose_gf.pose.pose.orientation.z - desired_pose_gf.pose.orientation.z)^2 + (quadrotor_pose_gf.pose.pose.orientation.w - desired_pose_gf.pose.orientation.w)^2

    if (errorsquar_pos < 0.5) and (errorsquar_ori < 0.08):#update values
        return True
    else:
        return False


def main():
    global quadrotor_pose_gf
    global desired_pose_gf
    global manipulation_pose
    global contact_mode

    rospy.init_node('pitch_dynamics')
    rospy.Subscriber("/contact_mode",Bool,callback_mode)                              #("subscribed topic name",message class, callback)  The callback records the subscribed data and processes it
    rospy.Subscriber("/raven/msf_core/odometry",Odometry,callback_quadcom_pose_gf)
    rospy.Subscriber("/desired_pose_gf",PoseStamped,callback_desired_pose_gf)
    pubci=rospy.Publisher('/raven/command/pose',PoseStamped,queue_size=10) #control input (pose reference for mav_control) during manipulatio mode is the manipulation_pose



    while not rospy.is_shutdown() and contact_mode and reference_achieved():
        #publish only if drone is in the reference zone -- reference_achieved() == True
        #mav_control message type for pitch reference and altitude holding
        generate_manipulation_pose()

        pubci.publish(manipulation_pose)
    #common for all nodes
    rospy.spin()



#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
