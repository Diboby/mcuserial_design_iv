import rospy
from time import sleep
import multiprocessing
import sys
from mcuros_msgs import McuToRosMsg
from mcuserial_python import SerialClient
from mcuserial_msgs.msg import TopicInfo, dataTemplate

def buildMessage():
    message = dataTemplate()

    message.header = 0x77
    message.dataSize = 0x0E
    message.function = 0x24
    message.offset = 0x05
    message.count = 0x01
    message.data = 0x99DAC5FFC5321CD
    message.crc16 = 0x8CF8

    return message


def concatenateMessage(message) :
    return str(message.header) + str(message.dataSize) + str(message.function) + str(message.offset) + str(message.count) + str(message.data) + str(message.crc16)

