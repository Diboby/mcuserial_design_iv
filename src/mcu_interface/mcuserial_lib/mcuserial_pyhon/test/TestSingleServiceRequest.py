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
    rospy.init_node("mcuserial_node_test")
    rospy.loginfo("ROS <--> MCU Serial Python Node Test")
    rospy.wait_for_service('alim_serial_com')
    print("alim_serial_com service available")

    try:
        alim_serial_service = rospy.ServiceProxy('alim_serial_com', alim_serial_com_srv)
        timee = 0
        if not rospy.is_shutdown():
            bef = time.time()
            resp = alim_serial_service(3, [], [])
            aft = time.time()
            print(aft - bef, resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)