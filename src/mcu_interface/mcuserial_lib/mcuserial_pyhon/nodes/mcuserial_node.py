#!/usr/bin/env python

import sys
import time
import rospy
import Queue
import threading
import multiprocessing

from ToBeRemoved import *
from mainController import *
from mcuserial_msgs.srv import RequestParam

from mcuserial_python import SerialClient
from serial import SerialException
from mcuserial_msgs.msg import TopicInfo, dataTemplate

noeud_write_queue = Queue.Queue()
noeud_reception_queue = Queue.Queue()

next_seq_num = 0
seq_num_in_use = set()


def noeud_service_callback(req):
    curr_id = message_sequence_attributer(next_seq_num, seq_num_in_use)
    # data = abstraction_layer.entry_point_to_main_controller(utility, curr_id, [device_ids, command_data])
    data = ""
    noeud_write_queue.put(data)

    # TODO send command
    # TODO wait for ack
    # TODO wait for data
    # TODO send ack
    # TODO if nack, check error type
    # TODO resend if possible
    # TODO else, tell user that problem with command
    # TODO if timeout, resend

    seq_num_in_use.remove(curr_id)  # When finished, ID removed

    """
    msg = buildMessage()
    data = concatenateMessage(msg)

    thread_event = thread_event
    while not rospy.is_shutdown() and not thread_event.is_set():
        noeud_write_queue.put(data)
        time.sleep(1)
    """


def sendMessage(thread_event, serialClient):
    thread_event = thread_event
    while not rospy.is_shutdown() and not thread_event.is_set():
        if noeud_write_queue.empty():
            time.sleep(0.01)
        else:
            data = noeud_write_queue.get()

            serialClient.send(data)

def message_sequence_attributer(next_seq_num, seq_num_in_use):
    attributed_num = next_seq_num

    if set([attributed_num]).issubset(seq_num_in_use):
        notFound = True
        for i in range(1, 16):
            test_num = (attributed_num + i) % 16
            if set([test_num]).isdisjoint(seq_num_in_use):
                attributed_num = test_num
                notFound = False
                break

        if notFound:
            raise ID_ATTRIBUTION_FAILED

    next_seq_num = (next_seq_num + 1) % 16
    seq_num_in_use.add(attributed_num)

    return attributed_num

# alimentation serial communication service
serial_service = rospy.Service("alim_serial_com", RequestParam, noeud_service_callback)

if __name__ == "__main__":
    rospy.init_node("mcuserial_node")
    rospy.loginfo("ROS <--> MCU Serial Python Node")

    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud', '9600'))

    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 4:
        exit(0)
    if len(sys.argv) >= 2:
        port_name = sys.argv[1]
    if len(sys.argv) == 3:
        baud = int(sys.argv[2])

    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            mcu_serial_interface = SerialClient(port_name, baud, 10)

            thread_event = threading.Event()
            noeud_send_msg_thread = threading.Thread(target=sendMessage, args=(thread_event, mcu_serial_interface))
            noeud_send_msg_thread.daemon = True
            noeud_send_msg_thread.start()

            mcu_serial_interface.run()


        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue
        # except:
        #    rospy.logwarn("Unexpected Error: %s", sys.exc_info()[0])
        #    thread_event.set()
        #    mcu_serial_interface.port.close()
        #    sleep(1.0)
        #    continue
