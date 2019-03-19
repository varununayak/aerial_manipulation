#!/usr/bin/env python

'''
SUBSCRIBES TO:
/man_config:    Theta values published by the inverse kinematics engine.

WRITES:
writes the three theta values to the serial port of the NUC (connected to the Arduino) in the format "<theta1,theta2,theta3>". This is a string of information. Eg: Write "<24.52,21.01,54.16>".

The theta values are in DEGREES.

'''
import serial
import rospy
#import  message definitions here
from geometry_msgs.msg import Vector3



#write any required functions here
def callback(msg):
    theta1=msg.x
    theta2=msg.y
    theta3=msg.z
    output= "<" + str(theta1) + "," + str(theta2) + ","+ str(theta3) + ">"
    ard_device = '/dev/ttyUSB1'

    #write to serial port here
    ser = serial.Serial(ard_device, 115200, timeout=0.5) #ard_device is "/dev/tty/USBX" wherever the arduino is connected
    ser.write(output)



def main():

    rospy.init_node('theta_to_serial')
    rospy.Subscriber("man_config",Vector3,callback)                              #("subscribed topic name",message class, callback)  The callback records the subscribed data and processes it
    rospy.spin()




#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
