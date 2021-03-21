#!/usr/bin/env python

import sys
import struct
import time
import rospy
import Queue
import threading
import multiprocessing

from ToBeRemoved import *
from mainController import *
from mcuserial_msgs.srv import alim_serial_com_srv, RequestParam

from mcuserial_python import SerialClient
from serial import SerialException
from mcuserial_msgs.msg import TopicInfo, dataTemplate


write_node_Queue_lock = threading.RLock()
noeud_write_queue = Queue.Queue()

read_node_Queue_lock = threading.RLock()
node_reception_queue = Queue.Queue()

next_seq_num = 0
seq_num_in_use = set()
seq_num_lock = threading.RLock()

class ID_ATTRIBUTION_FAILED(Exception):
    pass


def send_and_acquire_data(curr_id, utility, data, iteration_retry):
    data_out = []
    is_error = 0 # TODO WHAT ARE THE RIGHT ERROR_CODE? LIKE WRONG CRC OR BAD FORMAT OR ELSE?
    error_code = 0 # TODO comerr_regNotFound = 0x1, comerr_illegalAccess = 0x2, comerr_badFunction = 0x3, comerr_dataMissing = 0x4, tooManyRetries = 0x5, idattributionfailed = 0x6

    if not rospy.is_shutdown() and iteration_retry > 0:
        print_hexstr(data, 'sending ')
        noeud_write_queue.put((curr_id, data))
    else:
        is_error = 1
        error_code = 0x5

    msg_bytes = ''
    while not rospy.is_shutdown() and iteration_retry > 0 :
        if node_reception_queue.empty():
            time.sleep(0.01)
        else:
            with read_node_Queue_lock:
                (idd, msg_bytes) = node_reception_queue.get()
            print_hexstr(msg_bytes, 'received ')
            is_correct, seq_num, is_ack, is_nack, data_out = parse_correct_data(msg_bytes) 

            if seq_num != curr_id or idd != curr_id: # if data retrieved isn't for us, we put it back in the queue
                node_reception_queue.put(msg_bytes)
                time.sleep(0.01)
                continue
            else:
                if is_correct == 0 or is_nack == 1 or (is_ack == 0 and is_nack == 0): # if problem, try again
                    if is_nack == 1 and is_correct == 1:
                        nack_code = extract_data_and_convert(utility, data_out) # TODO TEST NACK WITH NON EXISTENT FUNCTION CALL
                        if not nack_code:
                            nack_code = 5
                        else:
                            nack_code = nack_code[0]
                    data_out, is_error, error_code = send_and_acquire_data(curr_id, utility, data, iteration_retry-1) 
                    if is_error == 1 and is_nack == 1 and is_correct == 1:
                        error_code = nack_code
                    if utility in rmt.function_no_return:
                        is_error = False
                        error_code = 0
                    break 
                else:
                    data_out = extract_data_and_convert(utility, data_out)

                break

    return data_out, is_error, error_code


def noeud_service_callback(req):
    utility = req.utility
    device_ids = list(req.device_ids)
    command_data = list(req.command_data)

    if utility in rmt.function_needs_password:
        command_data = [rmt.password_for_admin_fct_mcu]

    data_out = []
    is_error = 1
    error_code = -1 # ERROR LINKED TO FORMAT OF INPUT. CODE ISN'T ACCEPTING THE INPUTS AND THROWS IT RIGHT BACK TO USER.

    try:
        with seq_num_lock:
            curr_id = message_sequence_attributer(next_seq_num, seq_num_in_use)
        data = entry_point_to_main_controller(utility, curr_id, [device_ids, command_data])

        for element in data:
            data_out_elem, is_error_elem, error_code_elem = send_and_acquire_data(curr_id, utility, element, rmt.number_of_retrys)
            if error_code == -1:
                is_error = is_error_elem
                error_code = error_code_elem
            else:
                if is_error == 0:
                    is_error = is_error_elem
                    error_code = error_code_elem

            for dat in data_out_elem:
                data_out.append(dat)

    except ID_ATTRIBUTION_FAILED as e:
        print(e)
        with seq_num_lock:
            seq_num_in_use.remove(curr_id)  # When finished, ID removed
        return data_out, is_error, 6
    except Exception as e:
        print(e)
        with seq_num_lock:
            seq_num_in_use.remove(curr_id)  # When finished, ID removed
        return data_out, is_error, error_code
   
    with seq_num_lock:
        seq_num_in_use.remove(curr_id)  # When finished, ID removed
    return data_out, is_error, error_code


def sendMessage(thread_event, serialClient):
    thread_event = thread_event
    while not rospy.is_shutdown() and not thread_event.is_set():
        if noeud_write_queue.empty():
            time.sleep(0.01)
        else:
            with write_node_Queue_lock:
                (idd, data) = noeud_write_queue.get()

            serialClient.send(data)
            msg_bytes = serialClient.receive()
            with read_node_Queue_lock:
                node_reception_queue.put((idd, msg_bytes))

def message_sequence_attributer(next_seq_num, seq_num_in_use):
    attributed_num = next_seq_num

    if set([attributed_num]).issubset(seq_num_in_use):
        notFound = True
        for i in range(1, rmt.number_max_of_ids):
            test_num = (attributed_num + i) % rmt.number_max_of_ids
            if set([test_num]).isdisjoint(seq_num_in_use):
                attributed_num = test_num
                notFound = False
                break

        if notFound:
            raise ID_ATTRIBUTION_FAILED

    next_seq_num = (next_seq_num + 1) % rmt.number_max_of_ids
    seq_num_in_use.add(attributed_num)

    return attributed_num

if __name__ == "__main__":
    rospy.init_node("mcuserial_node")
    rospy.loginfo("ROS <--> MCU Serial Python Node")

    # alimentation serial communication service
    serial_service = rospy.Service("alim_serial_com", alim_serial_com_srv, noeud_service_callback)

    port_name = rospy.get_param('~port', '/dev/ttyUSB1')
    baud = int(rospy.get_param('~baud', '115200'))

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
            mcu_serial_interface = SerialClient(port_name, baud, 1)

            thread_event = threading.Event()
            noeud_send_msg_thread = threading.Thread(target=sendMessage, args=(thread_event, mcu_serial_interface))
            noeud_send_msg_thread.daemon = True
            noeud_send_msg_thread.start()

            while not rospy.is_shutdown():
                pass

            noeud_send_msg_thread.join()


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
