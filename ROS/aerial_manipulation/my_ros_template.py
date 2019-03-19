#!/usr/bin/env python

'''
GENERIC ROS NODE TEMPLATE FOR PYTHON
'''

import rospy

###             import any other libraries here
#import math
#import numpy as np

###             import any message definitions here
#from std_msgs.msg import String

###             write any required functions here
#def myfunction():




###             main function
def main():
    rospy.init_node('pitch_dynamics')
    rospy.Subscriber("",,)                              #("subscribed topic name",message class, callback)  The callback records the subscribed data and processes it
    pub = rospy.Publisher('', , queue_size=10)          #("name of published topic". message class, queue size)
    rate = rospy.rate(10)                               #frequency of publishing (optional)


    while not rospy.is_shutdown():
        #do stuff here

        #publish
        pub.publish()                                   #argument is the variable that you want to publish, should conform with message class eg:string
        rate.sleep()                                    #sleep in order to publish at desired rate

    #common for all nodes
    rospy.spin()                                        #so that ROS knows that this node should keep running



#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
