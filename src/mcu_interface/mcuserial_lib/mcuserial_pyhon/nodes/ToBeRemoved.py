import rospy
from time import sleep
import multiprocessing
import sys
import math
#from mcuros_msgs import McuToRosMsg
#from mcuserial_python import SerialClient
#from mcuserial_msgs.msg import TopicInfo, dataTemplate

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
    packet = bytearray()
    packet.append(0x77)
    packet.append(message.dataSize)
    #packet.append(num_seq << 4 | num_fonc)
    packet.append(message.function)
    #packet.append(num_reg)
    #if list_offset is not None and list_count is not None:
    packet.append(message.offset)
    packet.append(message.count)
    if message.data is not None:
        #taille = int(math.ceil((len(hex(message.data)) - 2) / 2))
        data_hex = hex(message.data)
        data_hex_lengh = len(data_hex)
        data_size = (float((data_hex_lengh) - 2) / 2)
        taille_r = math.ceil(data_size)
        taille = int(taille_r)
        mask = 0xFF
        liste = []
        for i in range(taille):
            liste.append((message.data & mask) >> i * 8)
            mask = mask << 8
        for i in range(len(liste) - 1, -1, -1):
            packet.append(liste[i])
    packet.append(message.crc16 >> 8)
    packet.append(message.crc16 & 0xFF)
    
    return packet
    #return str(message.header) + str(message.dataSize) + str(message.function) + str(message.offset) + str(message.count) + str(message.data) + str(message.crc16)

