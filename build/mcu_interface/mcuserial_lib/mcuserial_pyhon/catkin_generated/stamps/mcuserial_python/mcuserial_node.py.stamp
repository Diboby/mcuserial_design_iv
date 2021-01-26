#!/usr/bin/env python

import rospy
from mcuserial_python import SerialClient
from serial import SerialException
from time import sleep
import multiprocessing

import sys

if __name__=="__main__":
    rospy.init_node("mcuserial_node")
    rospy.loginfo("ROS --> MCU Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','9600'))

    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 4 :
        exit(0)
    if len(sys.argv) >= 2 :
        port_name  = sys.argv[1]
    if len(sys.argv) == 3 :
        baud = int(sys.argv[2])

    
    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
        try:
            mcuSerialInterface = SerialClient(port_name, baud)
            #Publisher and subcriber creating
            mcuSerialInterface.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue
        except:
            rospy.logwarn("Unexpected Error: %s", sys.exc_info()[0])
            mcuSerialInterface.port.close()
            sleep(1.0)
            continue
