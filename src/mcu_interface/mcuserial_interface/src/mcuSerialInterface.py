import rospy
from time import sleep
import multiprocessing
import sys
from mcuros_msgs.McuToRosMsg import * 
from mcuserial_python import SerialClient
from mcuserial_msgs.msg import TopicInfo, dataTemplate

#print(id(mcuserial_python))
#print(id(SerialClient))

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

    message.header = 0x77
    message.dataSize = 0x0E
    message.function = 0x24
    message.offset = 0x05
    message.count = 0x01
    message.data = 0x99DAC5FFC5321CD
    message.crc16 = 0x8CF8

    return message


if __name__ == "__main__" :

    rospy.init_node("test_node")

    publisher = rospy.Publisher("rosToMcu", dataTemplate, queue_size=10)
    msgT = buildMessage()
    print(msgT)

    #msg = McuToRosMsg()

    #mcuSerialInterface = SerialClient('/dev/ttyUSB0', 9600, 10000)

    i = 0
    while not rospy.is_shutdown() :
        publisher.publish(msgT)
        #print(id(mcuSerialInterface.general_write_queue))
        #mcuSerialInterface.send_internal_data(msg)
        
        i += 1
        print("number of message pushlished : {}".format(i))
        sleep(2)
        


