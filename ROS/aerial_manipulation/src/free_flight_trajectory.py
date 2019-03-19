#!/usr/bin/env python

'''
SUBSCRIBES TO:
/raven/msf_core/odometry:     Pose of Quadrotor COM (in ground frame)   variable name: quadrotor_pose_gf
/contact_mode:          Contact mode as defined in force_to_mode.py
/desired_pose_gf:       Desired Pose of the quadrotor (for transition) (in ground frame)

PUBLISHES:
/raven/command/trajectory:       Tracking of Quadrotor COM to Desired Point only if contact_mode==False

'''

import rospy

import math

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import RollPitchYawrateThrust
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from std_msgs.msg import Duration




#initializing
contact_mode = False #initialize mode to free-flight
quadrotor_pose_gf = Odometry()
desired_pose_gf = PoseStamped()

point1 = MultiDOFJointTrajectoryPoint()
point2 = MultiDOFJointTrajectoryPoint()

point1transform = Transform()
point2transform = Transform()
point1velocity = Twist()
point2velocity = Twist()
point1acceleration = Twist()
point2acceleration = Twist()

free_flight_trajectory_gf = MultiDOFJointTrajectory()


def callback_mode(msg):
    global contact_mode
    contact_mode = msg.data


def callback_quadcom_pose_gf(msg):
    global quadrotor_pose_gf
    quadrotor_pose_gf = msg

def callback_desired_pose_gf(msg):
    global desired_pose_gf
    desired_pose_gf = msg



def generate_trajectory():
    #this function generates two points on the trajectory required just before manipulation
    global desired_pose_gf,point1,point2,point1transform,point2transform,point1velocity,point2velocity,point1acceleration,point2acceleration, free_flight_trajectory_gf,quadrotor_pose_gf

    #generate MultiDOFJointTrajectoryPoints (two)

    #define point1 desired state
    point1transform.translation.x = desired_pose_gf.pose.position.x - 2.5
    point1transform.translation.y = 0.0
    point1transform.translation.z = desired_pose_gf.pose.position.z
    point1transform.rotation.x = 0.0
    point1transform.rotation.y = 0.0
    point1transform.rotation.z = 0.0
    point1transform.rotation.w = 1.0

    point1velocity.linear.x = 0.0
    point1velocity.linear.y = 0.0
    point1velocity.linear.z = 0.0
    point1velocity.angular.x = 0.0
    point1velocity.angular.y = 0.0
    point1velocity.angular.z = 0.0

    point1acceleration.linear.x = 0.0
    point1acceleration.linear.y = 0.0
    point1acceleration.linear.z = 0.0
    point1acceleration.angular.x = 0.0
    point1acceleration.angular.y = 0.0
    point1acceleration.angular.z = 0.0

    point1.time_from_start = rospy.rostime.Duration(10.0) #it will try to move to this state 5 seconds after initialization


    #define point2 desired state
    point2transform.translation.x = desired_pose_gf.pose.position.x
    point2transform.translation.y = desired_pose_gf.pose.position.y
    point2transform.translation.z = desired_pose_gf.pose.position.z
    point2transform.rotation.x = desired_pose_gf.pose.orientation.x
    point2transform.rotation.y = desired_pose_gf.pose.orientation.y
    point2transform.rotation.z = desired_pose_gf.pose.orientation.z
    point2transform.rotation.w = desired_pose_gf.pose.orientation.w

    point2velocity.linear.x = 0.0
    point2velocity.linear.y = 0.0
    point2velocity.linear.z = 0.0
    point2velocity.angular.x = 0.0
    point2velocity.angular.y = 1.0
    point2velocity.angular.z = 0.0

    point2acceleration.linear.x = 0.0
    point2acceleration.linear.y = 0.0
    point2acceleration.linear.z = 0.0
    point2acceleration.angular.x = 0.0
    point2acceleration.angular.y = 0.0
    point2acceleration.angular.z = 0.0

    point2.time_from_start = rospy.rostime.Duration(20.0) #it will try to move to this state 15 seconds after initialization


    #Appending the required trajectory states into the transforms array
    point1.transforms.append(point1transform)
    point2.transforms.append(point2transform)
    point1.velocities.append(point1velocity)
    point2.velocities.append(point2velocity)
    point1.accelerations.append(point1acceleration)
    point2.accelerations.append(point2acceleration)


    #use the point definitions to create the free-flight trajectory
    free_flight_trajectory_gf.header = quadrotor_pose_gf.header
    free_flight_trajectory_gf.header.frame_id = "free_flight_trajectory"

    free_flight_trajectory_gf.joint_names = ['middle','desired']
    free_flight_trajectory_gf.points = [point1,point2]




def main():
    global trial_pose_gf, quadrotor_pose_gf, contact_mode

    rospy.init_node('free_flight_trajectory')
    rospy.Subscriber("/contact_mode",Bool,callback_mode)                              #("subscribed topic name",message class, callback)  The callback records the subscribed data and processes it
    rospy.Subscriber("/raven/msf_core/odometry",Odometry,callback_quadcom_pose_gf)
    rospy.Subscriber("/desired_pose_gf",PoseStamped,callback_desired_pose_gf)
    pubtp = rospy.Publisher('/raven/command/trajectory',MultiDOFJointTrajectory, queue_size=10)          #("name of published topic". message class, queue size)
    generate_trajectory() #trajectory generated only once but published continuously in while loop

    while not rospy.is_shutdown() and not contact_mode:
        #replace trial pose with desired pose for actual test


        #publish
        pubtp.publish(free_flight_trajectory_gf)                                   #argument is the variable that you want to publish, should conform with message class eg:string
    #common for all nodes
    rospy.spin()



#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
