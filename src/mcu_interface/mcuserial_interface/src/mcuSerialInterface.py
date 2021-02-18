import rospy
from time import sleep
import multiprocessing
import sys
from mcuros_msgs.McuToRosMsg import * 
from mcuserial_python import SerialClient
from mcuserial_msgs.msg import TopicInfo, dataTemplate


class mcuSerialInterface :

    def __init__(self, port, baudrate, timeout=0.5):
        self.message = mcuMessage()
        self.serialClient = SerialClient(port, baudrate, timeout)

    def sendNullMessage(self):
        pass

    def sendMessage(self, mcuMessage) :
        pass

def callback():
    pass

def buildMessage():
    message = dataTemplate()

    message.header = 119
    message.dataSize = 5
    message.function = 3
    message.offset = 2
    message.count = 174
    message.data = [45, 13, 78, 34, 21]
    message.crc16 = 3

    return message


if __name__ == "__main__" :

    rospy.init_node("test_node")

    publisher = rospy.Publisher("rosToMcu", dataTemplate, queue_size=10)
    msgT = buildMessage()

    msg = McuToRosMsg()

    mcuSerialInterface = SerialClient('/dev/ttyUSB0', 9600, 10000)

    i = 0
    while not rospy.is_shutdown() :
        #publisher.publish(msg)
        print(id(mcuSerialInterface.general_write_queue))
        mcuSerialInterface.send_internal_data(msg)
        i += 1
        print("number of message pushlished : {}".format(i))
        sleep(2)
        


