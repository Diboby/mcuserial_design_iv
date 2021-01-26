import rospy
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing
import sys


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


serial = mcuSerialInterface('/dev/ttyUSB0', 9600)
serial.sendNullMessage()

