#!/usr/bin/env python

'''
SUBSCRIBES TO:
/force: value of force published by the force_from_serial node (float64)

PUBLISHES:
/contact_mode:  as described below (Bool)-->

contact_mode == False => Trajectory Tracking Mode
contact_mode == True => Manipulation Mode

The mode selection will use Schmitt Trigger switching method for force.


TODO: Nothing

'''

import rospy

#from std_msgs.msg import

from std_msgs.msg import Bool
from std_msgs.msg import Float64


contact_mode = False #False = free-flight mode, this is the initialization of the mode
force = 0.0
#callbacks


def callback_force(msg):
    global force
    force = float(msg.data)


#write any required functions here

def calculate_mode():
    global force
    return (True and force_schmitt_trigger())     #insert any other condition if required instead of "True" in the conditional


def force_schmitt_trigger():
    F_upperlimit= 3.8       #Schmitt Trigger Upper Limit
    F_lowerlimit=2.5        #Schmitt Trigger Lower Limit
    global force
    global contact_mode
    #Schmitt Trigger Implementation 
    if ((not contact_mode) and force>F_upperlimit):
        return True
    elif contact_mode and force<F_lowerlimit:
        return False
    elif contact_mode and force>F_lowerlimit:
        return True
    else:
        return False




def main():
    global force
    global contact_mode
    rospy.init_node('force_to_mode')
    pub = rospy.Publisher('contact_mode',Bool, queue_size=10)          #("name of published topic". message class, queue size)
    rospy.Subscriber("force",Float64,callback_force)

    while not rospy.is_shutdown():
        #do all stuff here
        contact_mode = calculate_mode()
        #publish

        pub.publish(contact_mode)                                   #argument is the variable that you want to publish, should conform with message class eg:string


    #common for all nodes
    rospy.spin()



#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
