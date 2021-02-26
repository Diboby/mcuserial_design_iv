import sys
import time
import rospy
import Queue
import threading
import multiprocessing

from mcuserial_msgs.srv import RequestParam

from mcuserial_python import SerialClient
from serial import SerialException
from mcuserial_msgs.msg import TopicInfo, dataTemplate


