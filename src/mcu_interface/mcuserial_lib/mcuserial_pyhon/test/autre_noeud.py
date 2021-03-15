import sys
import time
import rospy
import Queue
import threading
import multiprocessing

from mcuserial_msgs.srv import alim_serial_com_srv, RequestParam

from mcuserial_python import SerialClient
from serial import SerialException
from mcuserial_msgs.msg import TopicInfo, dataTemplate


if __name__ == "__main__":
    rospy.init_node("mcuserial_node_test2")
    rospy.loginfo("ROS <--> MCU Serial Python Node Test")
    rospy.wait_for_service('alim_serial_com')
    print("alim_serial_com service available")

    try:
        alim_serial_service = rospy.ServiceProxy('alim_serial_com', alim_serial_com_srv)
        while not rospy.is_shutdown():
            # TODO UT
            # TODO WHEN WRITING DATA, MINIMUM IS ALWAYS 4 BYTES, BECAUSE REGISTERS (TO VALIDATE NOW)
            '''print('Enter function number')
            x = int(input())
            print('Enter device id')
            y = int(input())
            print('Enter command data')
            z = int(input())'''
            resp = alim_serial_service(5, [200, 201], [1, 1])
            print(resp)
            sleep(3)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)