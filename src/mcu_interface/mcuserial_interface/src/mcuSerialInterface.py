import rospy
from time import sleep
import multiprocessing
import sys
from mcuserial_msgs.msg import TopicInfo, dataTemplate


class mcuMessage :

    def __init__(self):
        self.startBits = b'\x77'
        self.packetSize = b'\x00'
        self.function = b'\x00'
        self.registerNumber = b'\x00'
        self.offset = b'\x00'
        self.count = b'\x00'
        self.data = b'\x00'
        self.crc16 = b"\x00\x00"
    
    def setData(self, data):
        self.data = data

    def concatenateMessage(self):
        result = self.startBits + self.packetSize + self.function + self.registerNumber + self.offset + self.count + self.data + self.crc16
        return result


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

    message.header = 77
    message.lengh = 5
    message.function = 3
    message.offset = 0
    message.count = 0
    message.data = [45, 13, 78, 34, 21]
    message.crc16 = 0

    #message.header = b'\0x77'
    #message.lengh = b'\0x02'
    #message.function = b'\0x11'
    #message.offset = b'\0x00'
    #message.count = b'\0x00'
    #message.data = b'\0x08\0x23'
    #message.crc16 = b'\0x00'

    return message


if __name__ == "__main__" :

    rospy.init_node("test_node")

    publisher = rospy.Publisher("rosToMcu", dataTemplate, queue_size=10)
    msg = buildMessage()    
    i = 0

    while not rospy.is_shutdown() :
        publisher.publish(msg)
        i += 1
        print("number of message pushlished : {}".format(i))
        sleep(2)
        


