#!/usr/bin/env python


'''

READS:
Parses data from the serial port connected to the Arduino. The data contains the value of the force experienced at the end-effector
Format available on serial port: "<F_value>"


PUBLISHES:
/force:     Value of force in Newtons (float64)



TODO: Nothing

'''
import serial
import rospy

from std_msgs.msg import String
from std_msgs.msg import Float64


#--------------------------------------------------------#
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1) #opening takes time so do it only once #setbaudrate on Arduino as given
s='0' #initializing the value of the serial-read force

def read_force_serial():
    global s
    ##read force value from serial port here
    line = ser.readline()   # read a '\n' terminated line
    x = ser.read() # read one byte at beginning
    if x=='<':
        s = ser.read(4)        # read up the three bytes that captures the float value (timeout
    read_force = float(s)
    return float(s)




def main():
    force=0.0  #initialize  force value
    rospy.init_node('force_from_serial')
    pub = rospy.Publisher('force',Float64, queue_size=10)          #("name of published topic". message class, queue size)
    ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)



    while not rospy.is_shutdown():
        #do all stuff here
        force = read_force_serial()         

        pub.publish(force)                                   #argument is the variable that you want to publish, should conform with message class eg:string
        #rate.sleep()

    #common for all nodes
    rospy.spin()



#common for all nodes
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
