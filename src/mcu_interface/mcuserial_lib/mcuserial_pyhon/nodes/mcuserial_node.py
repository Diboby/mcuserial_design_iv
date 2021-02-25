#!/usr/bin/env python

import rospy
from mcuserial_python import SerialClient
from serial import SerialException
from time import sleep
import multiprocessing
import threading
from mainController import *
import Queue
import time

from ToBeRemoved import *
from mcuserial_msgs.msg import TopicInfo, dataTemplate
from mcuserial_msgs.srv import *

import sys

noeud_write_queue = Queue.Queue()
noeud_reception_queue = Queue.Queue()

next_seq_num = 0
seq_num_in_use = set()


def noeud_service_callback(thread_event) :
    # Attribuer un ID a ce message
    # Appeler l'abstraction layer
    data = ""
    noeud_write_queue.put(data)


    """
    msg = buildMessage()
    data = concatenateMessage(msg)

    thread_event = thread_event
    while not rospy.is_shutdown() and not thread_event.is_set():
        noeud_write_queue.put(data)
        time.sleep(1)
    """

def sendMessage(self, thread_event, serialClient):
    thread_event = thread_event
    while not rospy.is_shutdown() and not thread_event.is_set():
        if noeud_write_queue.empty():
            time.sleep(0.01)
        else:
            data = noeud_write_queue.get()
            
            serialClient.send(data)
            
            #if isinstance(data, bytes) :                    
            #    serialClient.send(data)   


"""
# Knows the logic between each message exchange so that one message transmitted should have an ack, and
# if it's the case, a response message containing data. If nack, resend message a couple of times. If nack
# all the time, throws exception so that main entry point can send back to the service a None data type
# because communication seems wrong.
def message_sequence_attributer(self):
    attributed_num = self.next_seq_num

    if set([attributed_num]).issubset(self.seq_num_in_use):
        notFound = True
        for i in range(1, 16):
            test_num = (attributed_num + i) % 16
            if set([test_num]).isdisjoint(self.seq_num_in_use):
                attributed_num = test_num
                notFound = False
                break

        if notFound:
            raise ID_ATTRIBUTION_FAILED

    self.next_seq_num = (self.next_seq_num + 1) % 16
    self.seq_num_in_use.add(attributed_num)

    return attributed_num


def message_sequencer(self, mcu_reg_number, mcu_function_number, data, list_offset, list_count, can_send_data):
        dataTemp = data
        if not can_send_data:
            dataTemp = None
        mcu_command = self.message_constructor(mcu_reg_number, mcu_function_number, dataTemp, list_offset, list_count)
        # TODO send command
        # TODO wait for ack
        # TODO wait for data
        # TODO send ack
        # TODO if nack, check error type
        # TODO resend if possible
        # TODO else, tell user that problem with command
        # TODO if timeout, resend
        print(mcu_command)
        self.seq_num_in_use.remove(mcu_command[2] >> 4)
        return mcu_command


"""

serial_service = rospy.Service("alim_serial_com", RequestParam, noeud_service_callback)


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

    
    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
        try:
            mcu_serial_interface = SerialClient(port_name, baud, 10)
           
            thread_event = threading.Event()            
            noeud_send_msg_thread = threading.Thread(target=sendMessage, args=(thread_event, mcu_serial_interface))
            noeud_send_msg_thread.daemon = True
            noeud_send_msg_thread.start()

            serial_service = rospy.Service("alim_serial_com", RequestParam, noeud_service_callback)

            mcu_serial_interface.run()            


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
            thread_event.set()
            mcu_serial_interface.port.close()
            sleep(1.0)
            continue

