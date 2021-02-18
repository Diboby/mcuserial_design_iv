#!/opt/bin/python
#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String

def serial_connection():
    rospy.init_node('serial_connection', anonymous = True)
    rate = rospy.Rate(10)
    syxnotita = 0
    euros = 0
    mesi_gwnia = 0
    while not rospy.is_shutdown():
        data = [syxnotita, euros, mesi_gwnia]
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        print (ser.name)
        time.sleep(2)
        while ser.isOpen():
            print ("Port Open")
            for i in range(5):
                print ('Allo Design 4 !')
                ser.write("Allo Design 4 !\n")
                time.sleep(1)
                rate.sleep()
        ser.close()

serial_connection()
