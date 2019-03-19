#!/usr/bin/env python

'''

PUBLISHES:
/wall_position_qf:      Position of Wall Point expressed in quadrotor COM frame(for IK) (Vector3)
/desired_pose_gf:       Pose of Desired State   in ground frame(for mav_control): This is the pose to be achieved just before manipulation (transition)
/raven/msf_core/odometry:     Pose of Quadrotor COM (in ground frame)   variable name: quadrotor_pose_gf

Subscribes to: VICON data


TODO: Transform desired pose according to wall pose (instead of keeping it aligned with ground frame) and publish it as a Pose() message type.


'''

import rospy
import numpy as np
import tf
import PyKDL as kdl
import math


#import more message definitions here
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseStamped



from geometry_msgs.msg import Vector3

wall_position_qf = np.array([0.0,0.0,0.0])
wall_position_gf_rel =  np.array([0.0,0.0,0.0])

quadrotor_pose_gf = [TransformStamped().header,'0',0,0,0,0,0,0,0]  #initialize TransformStamped of quadrotor in ground frame
wall_position_gf = [TransformStamped().header,'0',0,0,0,0,0,0,0]  #initialize TransformStamped of wall in ground frame
odom = Odometry()
desired_pose_gf = PoseStamped()

#write any required functions here
def callback_quadcom_pose_gf(msg):
    global quadrotor_pose_gf
    quadrotor_pose_gf = [msg.header,  msg.child_frame_id  ,msg.transform.translation.x  , msg.transform.translation.y, msg.transform.translation.z  , msg.transform.rotation.x,  msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

def callback_wall_position_gf(msg):
    global wall_position_gf
    wall_position_gf =gf = [msg.header,  msg.child_frame_id  ,msg.transform.translation.x  , msg.transform.translation.y  ,msg.transform.translation.z  , msg.transform.rotation.x,  msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

def generate_odometry_pose_with_covariance():
    global quadrotor_pose_gf
    global odom

    odom.header.stamp = quadrotor_pose_gf[0].stamp
    odom.header.frame_id = quadrotor_pose_gf[0].frame_id # i.e. '/odom
    odom.child_frame_id = quadrotor_pose_gf[1] # i.e. '/base_footprint'
    odom.pose.pose.position = Point(quadrotor_pose_gf[2],  quadrotor_pose_gf[3], quadrotor_pose_gf[4])
    odom.pose.pose.orientation = Quaternion(quadrotor_pose_gf[5],quadrotor_pose_gf[6],quadrotor_pose_gf[7],quadrotor_pose_gf[8])
    p_cov = np.array([0.0]*36).reshape(6,6) #zero covariance for VICON data
    odom.pose.covariance = tuple(p_cov.ravel().tolist())




def transform_wall_position():
    global wall_position_gf
    global quadrotor_pose_gf
    global wall_position_qf

    #Rotating and Translating the wall position vector to get it in quadrotor frame expressed in quadrotor frame
    wall_position_gf_rel = np.array([wall_position_gf[2], wall_position_gf[3], wall_position_gf[4]]) - np.array([quadrotor_pose_gf[2],  quadrotor_pose_gf[3], quadrotor_pose_gf[4]])
    q = [quadrotor_pose_gf[8], quadrotor_pose_gf[5], quadrotor_pose_gf[6], quadrotor_pose_gf[7]]
    R = np.array([[1-2*(q[2]**2)-2*q[3]**2,2*q[1]*q[2]-2*q[3]*q[0],2*q[1]*q[3]+2*q[2]*q[0]],[2*q[1]*q[2]+2*q[3]*q[0],1-2*q[1]**2-2*q[3]**2,2*q[3]*q[2]-2*q[0]*q[1]],[2*q[1]*q[3]-2*q[2]*q[0],2*q[2]*q[3]+2*q[1]*q[0],1-2*q[1]**2-2*q[2]**2]])
    wall_position_qf = np.dot(R,wall_position_gf_rel)

    #Getting the wall pose and translating it along its own x axis (negatively by some amount) and its own z axis (negatively by some amount) and adding a pitch angle for desired pose

def generate_desired_pose():
    global desired_pose_gf
    global wall_position_gf

    dx= 0.495 #x-distance from wall just before manipulation
    dz= 0.363 #z-distance from wall just before manipulation

    pitch_reference_angle = 10*math.pi/180

    desired_pose_gf.header = wall_position_gf[0]

    #wall must be parallel to yz plane
    desired_pose_gf.pose.position.x = wall_position_gf[2] - dx
    desired_pose_gf.pose.position.y = wall_position_gf[3]
    desired_pose_gf.pose.position.z = wall_position_gf[4] - dz
    desired_pose_gf.pose.orientation.x = 0
    desired_pose_gf.pose.orientation.y = 1*math.sin(pitch_reference_angle/2)
    desired_pose_gf.pose.orientation.z = 0
    desired_pose_gf.pose.orientation.w = math.cos(pitch_reference_angle/2)




def main():
    global quadrotor_pose_gf, wall_position_gf, wall_position_qf, desired_pose_gf

    rospy.init_node('pose_publisher_vicon')
    rospy.Subscriber("/vicon/m100_varun/m100_varun",TransformStamped,callback_quadcom_pose_gf)               #/vicon/m100_varun/m100_varun
    rospy.Subscriber("/vicon/manipulated_wall/manipulated_wall",TransformStamped,callback_wall_position_gf)
    #pub = rospy.Publisher('', , queue_size=10)          #("name of published topic". message class, queue size)
    pubqcom = rospy.Publisher('/raven/msf_core/odometry',Odometry,queue_size=10)
    pubwpqf = rospy.Publisher('/wall_position_qf',Vector3,queue_size=10)
    pubdpgf = rospy.Publisher('/desired_pose_gf',PoseStamped,queue_size=10)



    while not rospy.is_shutdown():
        #do stuff here

        #for expressing the wall position in quadrotor frame and generate desired pose with respect to wall pose
        transform_wall_position() #transforms wall position from VICON frame to Quadrotor Frame for IK
        generate_desired_pose()
        generate_odometry_pose_with_covariance()


        #publish
        pubqcom.publish(odom) #publish odometry data to raven
        pubwpqf.publish(wall_position_qf[0],wall_position_qf[1],wall_position_qf[2]) #for the use of the inverse kinematics
        pubdpgf.publish(desired_pose_gf)
    #common for all nodes
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
