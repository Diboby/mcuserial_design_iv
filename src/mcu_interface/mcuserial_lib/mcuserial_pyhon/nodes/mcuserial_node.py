#!/usr/bin/env python

import rospy
from mcuserial_python import SerialClient
from serial import SerialException
from time import sleep
import multiprocessing

from mcuserial_msgs.msg import TopicInfo, dataTemplate

import sys

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

    topicInfoSuscriber = TopicInfo()
    topicInfoSuscriber.topic_id = 100
    topicInfoSuscriber.topic_name = "rosToMcu"
    topicInfoSuscriber.message_type = "mcuserial_msgs/dataTemplate"
    topicInfoSuscriber.md5sum = dataTemplate._md5sum
    topicInfoSuscriber.buffer_size = 20

    topicInforPublisher = TopicInfo()
    topicInforPublisher.topic_id = 200
    topicInforPublisher.topic_name = "mcuToRos"
    topicInforPublisher.message_type = "mcuserial_msgs/dataTemplate"
    topicInforPublisher.md5sum = dataTemplate._md5sum
    topicInforPublisher.buffer_size = 20

    
    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
        try:
            mcuSerialInterface = SerialClient(port_name, baud)
            #Publisher and subcriber creating
            mcuSerialInterface.callbacks[TopicInfo.ID_SUBSCRIBER](topicInfoSuscriber)
            mcuSerialInterface.callbacks[TopicInfo.ID_PUBLISHER](topicInforPublisher)
            
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
