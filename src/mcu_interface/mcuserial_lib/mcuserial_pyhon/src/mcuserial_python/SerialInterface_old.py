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

    def __init__(self, port='/dev/ttyUSB0', baud=9600, timeout=5.0):
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        self.header = 0x77

        self.read_lock = threading.RLock()
        self.write_lock = threading.RLock()
        
        self.writeQueue_lock = threading.RLock()
        self.write_queue = Queue.Queue()
        self.write_thread = None

        self.readQueue_lock = threading.RLock()
        self.read_queue = Queue.Queue()
        self.read_thread = None

        self.lastsync = rospy.Time(0)
        self.lastsync_lost = rospy.Time(0)
        self.lastsync_success = rospy.Time(0)
        self.last_read = rospy.Time(0)
        self.last_write = rospy.Time(0)
        self.timeout = timeout
        self.synced = False

        self.publishers = dict()  # id:Publishers
        
        self.pub_diagnostics = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=10)

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
                    #write_timeout=10
                    self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10000)                        
                    break
                except SerialException as e:
                    rospy.logerr("Error opening serial: %s", e)
                    time.sleep(3)

        if rospy.is_shutdown():
            return

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        rospy.sleep(2.0)
        self.requestTopics()
        self.lastsync = rospy.Time.now()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        rospy.loginfo('Requesting topics...')

        with self.read_lock:
            self.port.flushInput()

        # request topic sync
        #self.write_queue.put(self.header + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """
        with self.read_lock:
            self.port.flushInput()

        #self.write_queue.put(self.header + b"\x00\x00\xff\x0b\x00\xf4")
        #rospy.loginfo("Sending tx stop request")

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = rospy.Time.now()
                    result.extend(received)
                    bytes_remaining -= len(received)

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def tryReadAll(self, length):
        """ Read all the message arrived on the serial port from mcu to ROS """
        try:
            read_start = time.time()
            result = bytearray()
            while time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read()
                if len(received) != 0:
                    self.last_read = rospy.Time.now()
                    result.extend(received)

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)


    def handle_data_reception(self):
        """ Handle message reception from mcu to ROS : First treating function. """
        # Handle reading.
        data = ''
        read_step = None
        result = bytearray()
        while self.write_thread.is_alive() and not rospy.is_shutdown():

            if self.can_received_data == 0:
                continue
            
            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:   # Quand le port est en train de recevoir des donnees
                        time.sleep(0.001)
                        continue

                # we begin reading
                # Read header
                flag = 0x00
                read_step = 'header'
                flag = self.tryRead(1)
                if (ord(flag) != self.header):
                    continue
                msg_bytes = bytearray()
                msg_bytes.extend(flag)

                # For details about unpack, follow https://docs.python.org/3/library/struct.html

                read_step = 'packet size'
                packet_size_bytes = self.tryRead(1)
                msg_bytes.extend(packet_size_bytes)
                packet_size = ord(packet_size_bytes)

                #msg_length, _ = struct.unpack(">hB", msg_len_bytes)

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

                with self.read_lock:
                    self.read_queue.put(msg_bytes)
                    self.can_received_data = 0

            except IOError as exc:
                rospy.logwarn('Last read step: %s' % read_step)
                rospy.logwarn('Run loop error: %s' % exc)
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()
        self.txStopRequest()

    def run(self):
        """ Forward recieved messages to appropriate publisher. """

        # Perso
        #while not rospy.is_shutdown() :            
            #self.processWriteQueue()
            

        # Launch write thread.
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self.processWriteQueue)
            self.write_thread.daemon = True
            self.write_thread.start()
        
        if self.read_thread is None:
            self.read_thread = threading.Thread(target=self.handle_data_reception)
            self.read_thread.daemon = True
            self.read_thread.start()

        
        

    def send(self, msg):
        """
        Queues data to be written to the serial port.
        """
        rospy.loginfo("Putting message in Queue (ROS ----> MCU)")
        
        #msg = str(msg).encode()
        
        with self.writeQueue_lock:
            self.write_queue.put(msg)

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/mcuserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = rospy.Time.now()

    def processWriteQueue(self):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """
        while not rospy.is_shutdown():
            if self.write_queue.empty():
                time.sleep(0.01)
            else:
                rospy.loginfo("Sending message from ROS to MCU")
                data = self.write_queue.get()                
                while True:
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
                        self.can_received_data = 1
                        break
                    except SerialTimeoutException as exc:
                        rospy.logerr('Write timeout: %s' % exc)
                        time.sleep(1)
                    except RuntimeError as exc:
                        rospy.logerr('Write thread exception: %s' % exc)
                        break

            
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
        if self.lastsync.to_sec() > 0:
            status.values[0].value=time.ctime(self.lastsync.to_sec())
        else:
            status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
        status.values[1].value=time.ctime(self.lastsync_lost.to_sec())

        self.pub_diagnostics.publish(msg)
