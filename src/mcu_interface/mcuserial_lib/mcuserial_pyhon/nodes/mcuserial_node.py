#!/usr/bin/env python

import rospy
from mcuserial_python import SerialClient
from serial import SerialException
from time import sleep
import multiprocessing
import threading

from ToBeRemoved import *
from mcuserial_msgs.msg import TopicInfo, dataTemplate

import sys

def sendMessage(ThrEvent, serialClient, index):
    ThrEvent = ThrEvent
    msg = buildMessage()
    msg = concatenateMessage(msg) + " " + str(index)
   
    i = 0
    while not rospy.is_shutdown() and not ThrEvent.is_set() :
        serialClient.send(msg)        
        i += 1
        print("number of message pushlished : {}".format(i))
        sleep(2)


if __name__=="__main__":
    rospy.init_node("mcuserial_node")
    rospy.loginfo("ROS <--> MCU Serial Python Node")

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
            mcuSerialInterface = SerialClient(port_name, baud, 10)
            ThrEvent = threading.Event()
            waThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread A"))
            waThread.daemon = True
            waThread.start()


            wbThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread B"))
            wbThread.daemon = True
            wbThread.start()

            wcThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread C"))
            wcThread.daemon = True
            wcThread.start()

            wdThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread D"))
            wdThread.daemon = True
            wdThread.start()

            weThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread E"))
            weThread.daemon = True
            weThread.start()


            weThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread F"))
            weThread.daemon = True
            weThread.start()


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
            ThrEvent.set()
            mcuSerialInterface.port.close()
            sleep(1.0)
            continue

