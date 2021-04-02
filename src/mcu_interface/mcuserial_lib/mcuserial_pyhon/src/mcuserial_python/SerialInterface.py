import array
import errno
import imp
import io
import multiprocessing
import Queue
import socket
import struct
import sys
import threading
import time

from serial import Serial, SerialException, SerialTimeoutException

import roslib
import rospy
from std_msgs.msg import Time
from mcuserial_msgs.msg import TopicInfo, Log
from mcuserial_msgs.srv import RequestParamRequest, RequestParamResponse
from mcuros_msgs import McuToRosMsg

import diagnostic_msgs.msg

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or mcuserial_python is from different ros release than the mcuserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"

def load_pkg_module(package, directory):
    #check if its in the python path
    path = sys.path
    try:
        imp.find_module(package)
    except ImportError:
        roslib.load_manifest(package)
    try:
        m = __import__( package + '.' + directory )
    except ImportError:
        rospy.logerr( "Cannot import package : %s"% package )
        rospy.logerr( "sys.path was " + str(path) )
        return None
    return m

def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

def load_service(package,service):
    s = load_pkg_module(package, 'srv')
    s = getattr(s, 'srv')
    srv = getattr(s, service)
    mreq = getattr(s, service+"Request")
    mres = getattr(s, service+"Response")
    return srv,mreq,mres

class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """

    _instance = None
    def __new__(class_, *args, **kwargs):
        if not isinstance(class_._instance, class_):
            class_._instance = object.__new__(class_, *args, **kwargs)
        return class_._instance

    def __init__(self, port='/dev/ttyUSB0', baud=115200, timeout_recv=0.2, timeout_logic_end=2.0):
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        self.header = 0x77
        self.timeout_logic_end = timeout_logic_end
        self.timeout_recv = timeout_recv

        if port is None:
            # Pas de port specifiee
            pass
        elif hasattr(port, 'read'):
            # Quand le port est deja ouvert
            self.port=port
        else:
            # ouvrir un port
            while not rospy.is_shutdown():
                try:
                    self.port = Serial(port, baud, timeout=self.timeout_recv, write_timeout=self.timeout_recv)                        
                    break
                except SerialException as e:
                    rospy.logerr("Error opening serial: %s", e)
                    time.sleep(3)

        if rospy.is_shutdown():
            return

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        rospy.sleep(2.0)
        self.requestTopics()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        rospy.loginfo('Requesting topics...')
        self.port.flushInput()

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """
        self.port.flushInput()

    def tryRead(self, length):
        try:
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0:
                received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    result.extend(received)
                    bytes_remaining -= len(received)
                else:
                    return bytearray()

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)


    def receive(self):
        """ Handle message reception from mcu to ROS : First treating function. """
        # Handle reading.
        data = ''
        read_step = None
        result = bytearray()
            
        # This try-block is here because we make multiple calls to read(). Any one of them can throw
        # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
        # bottom attempts to reconfigure the topics.
        try:
             # we begin reading
            # Read header
            flag = '0'
            read_start = time.time()
            while ord(flag) != self.header:
                read_step = 'header'
                flag = self.tryRead(1)

                if len(flag) == 0: #IF TIMEDOUT
                    out_array = bytearray()
                    return out_array

                if (ord(flag) != self.header): #IF RECEIVED NOISE, CONTINUE UNTIL TIMEDOUT
                    if (time.time() - read_start) > self.timeout_logic_end:
                        out_array = bytearray()
                        out_array.extend(flag)
                        return out_array
                    continue

            msg_bytes = bytearray()
            msg_bytes.extend(flag)

            try:
                read_step = 'packet size'
                packet_size_bytes = self.tryRead(1)
                msg_bytes.extend(packet_size_bytes)
                packet_size = ord(packet_size_bytes)
            except Exception:
                return msg_bytes

            # Read serialized message data.
            read_step = 'data'
            try:
                data_bytes = self.tryRead(packet_size)
                msg_bytes.extend(data_bytes)

            except IOError:
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                rospy.loginfo("Packet Failed :  Failed to read msg data")
                rospy.loginfo("expected msg length is %d", msg_length)
                raise

            return msg_bytes

        except IOError as exc:
            rospy.logwarn('Last read step: %s' % read_step)
            rospy.logwarn('Run loop error: %s' % exc)
            # One of the read calls had an issue. Just to be safe, request that the client
            # reinitialize their topics.
            self.port.flushInput()
            self.port.flushOutput()
            self.requestTopics()

    def send(self, data):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """

        try:
            if isinstance(data, bytes):
                self._write(data)
            elif isinstance(data, bytearray):
                self._write(data)
            elif isinstance(data, int) or isinstance(data, long):
                data = str(data).encode()
                self._write(data)                        
            else:
                rospy.logerr("Trying to write invalid data type: %s" % type(data))
            
        except SerialTimeoutException as exc:
            rospy.logerr('Write timeout: %s' % exc)
            time.sleep(1)
        except RuntimeError as exc:
            rospy.logerr('Write thread exception: %s' % exc)


    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/mcuserial/Overview/Protocol
        """
        self.port.write(data)
            
    def sendDiagnostics(self, level, msg_text):
        msg = diagnostic_msgs.msg.DiagnosticArray()
        status = diagnostic_msgs.msg.DiagnosticStatus()
        status.name = "mcuserial_python"
        msg.header.stamp = rospy.Time.now()
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[0].key="last sync"
        status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
