#!/usr/bin/env python



'''
SUBSCRIBES TO:
/wall_position_qf:      Position vector of the point on the wall to be manipulated (in the quadrotor COM frame) (Vector3)


PUBLISHES:
/man_config:    Three float values theta1, theta2 and theta3  in degrees based on inverse_kinematics()



The theta values are calculated using the inverse kinematics. The default values are theta1=0; theta2=30; theta3=-25

TIP: Exaggerate the last link length a little so that the wall point comes into the workspace sooner and a certain pressure is applied (say, by 2-3cm)

TODO: Nothing
'''

import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

#import more message definitions here


wp_position=[0.0,0.0,0.0]

man_config=[0.00,30.00,-25.00] #initialize manipulator configuration in degrees




#write any required functions here

def callback_wpp(msg):
    global wp_position
    wp_position= [msg.x,msg.y,msg.z]


def inverse_kinematics():
    global man_config
    global wp_position

    rel_position = wp_position #make sure rel_position is expressed in frame attached to mb i.e vector from manipulator base frame to wall point in manipulator base frame
    # If the point (x,y,z) is outside the workspace the variable holds its previous valid value
    x=rel_position[0]
    y=rel_position[1]
    z=rel_position[2]

    #find theta1, theta2 and theta3
    theta1 = math.atan2(y,x) #the y and x here are in 3 dimensions

    #getparams
    dl1=0.010 #offset due to servo 1
    dl2=0.011 #offset due to servo 2
    l1=0.0956 #first link length
    l2=0.3165 #second link length
    l3=0.189  + 0.03 #third link length (exaggeration by 30 mm)

    dx=0.0228 #offset of the base of the manipulator from quadrotor COG
    dz=0.0797

    '''
    #UNCOMMENT THIS SECTION FOR 3-DOF
    X=np.dot( [ [math.cos(theta1),math.sin(theta1),0,-dl1]  ,  [0,0,1,-l1]  ,  [math.sin(theta1),-math.cos(theta1),0,0]  ,  [0,0,0,1] ] , [x,y,z,1])  #for 3DOF
    bring into 2 dimensions, for 3DOF only
    x=X[0]
    y=X[1]
    '''

    #COMMENT THIS PART OUT FOR 3-DOF
    x=x-(dl2+dx)
    y=z-(dz+l1)



    theta3= - math.acos((x**2+y**2-l2**2-l3**2)/(2*l2*l3))
    phi= math.asin((math.sin(theta3)*l3)/(math.sqrt(x**2+y**2)))  #intermediate variable
    theta2= math.atan2(y,x)-phi #the y and x here are in the 2 dimension analysis

    #conversion to degrees as the driver uses degrees
    theta1deg= theta1*180/(math.pi)
    theta2deg= theta2*180/(math.pi)
    theta3deg= theta3*180/(math.pi)

    #store in man_config topic
    man_config=[theta1deg,theta2deg,theta3deg]



def main():
    global man_config
    rospy.init_node('ik_publish_theta')
    rospy.Subscriber("/wall_position_qf",Vector3,callback_wpp)
    pub = rospy.Publisher('man_config',Vector3 , queue_size=10)


    while not rospy.is_shutdown():
        #do stuff here
        inverse_kinematics()
        pub.publish(man_config[0],man_config[1],man_config[2])   #publishing theta1 theta2 and theta3 used by the theta_to_serial node

    #common for all nodes
    rospy.spin()






#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
