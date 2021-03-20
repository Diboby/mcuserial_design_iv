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

def parse_correct_data(msg):

    byte_list = []
    for byte in msg:
        byte_list.append(byte)

    if(len(byte_list) < 6):
        return 0, 0, 0, 0, None

    packet_size = int(byte_list[1])
    computed_crc = CRC16_Compute(byte_list, packet_size)
    given_crc = int(str(msg[len(msg)-2:len(msg)]).encode('hex'), 16)

    valid_crc = (given_crc == computed_crc)

    seq_num = (byte_list[2] & 0xF0) >> 4
    function_num = byte_list[2] & 0xF

    valid_function = (function_num==0xB or function_num==0xC)

    is_correct = valid_function and valid_crc

    if function_num == 0xB:
        is_ack = 1
    else:
        is_ack = 0
    
    if function_num == 0xC:
        is_nack = 1
    else:
        is_nack = 0
    
    try:
        data_out = byte_list[4:-2]
    except Exception:
        data_out = None

    return is_correct, seq_num, is_ack, is_nack, data_out

def print_hexstr(inputdd, monko):
    string = monko
    for b in inputdd:
        string = string + str(hex(b)) + ' '
    print(string)


def extract_data_and_convert(utility, data_out):
    data_out_params = []
    if utility == 4 or utility == 5 or utility == 6 or utility == 7: # if command was about a list, we remove from data offset and count bytes
        data_out_params.append(data_out[0])
        data_out_params.append(data_out[1])
        data_out = data_out[2:]
    
    data_out_segmented_in_chunks = []
    for i in range(0, len(data_out)-3, 4): # separates data into chunks of 4 bytes
        data_out_segmented_in_chunks.append([data_out[i], data_out[i+1], data_out[i+2], data_out[i+3]])
    
    
    data_out_segmented_in_chunks_endian = []
    for chunk in data_out_segmented_in_chunks:
        data_out_temp = []
        for i in range(len(chunk) - 1, -1, -1): # convert from little endian to big endian
            data_out_temp.append(chunk[i])
        outnum = 0
        for i in range(0, len(data_out_temp)): # concatenate bytes to form list of floats
            outnum = outnum | data_out_temp[i] << 8*(len(data_out_temp)-1-i)
        data_out_segmented_in_chunks_endian.append(outnum)
    data_out = data_out_segmented_in_chunks_endian

    return data_out


def send_and_acquire_data(curr_id, utility, data, iteration_retry):
    data_out = []
    is_error = 0
    error_code = 0 # TODO comerr_regNotFound = 0x1, comerr_illegalAccess = 0x2, comerr_badFunction = 0x3, comerr_dataMissing = 0x4, tooManyRetries = 0x5

    if not rospy.is_shutdown() and iteration_retry > 0:
        print_hexstr(data, 'sending ')
        noeud_write_queue.put((curr_id, data)) # TODO VERIFY MULTIPLE COMMANDS WITH SAME ID
    else:
        is_error = 1
        error_code = 0x5

    msg_bytes = ''
    while not rospy.is_shutdown() and iteration_retry > 0:
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
                        nack_code = extract_data_and_convert(utility, data_out)
                        #TODO INPUT CERTAIN BEHAVIOR FOR NACK CODE
                    data_out, is_error, error_code = send_and_acquire_data(curr_id, utility, data, iteration_retry-1) 
                    break 
                else:
                    data_out = extract_data_and_convert(utility, data_out)

                break

    return data_out, is_error, error_code


def noeud_service_callback(req):
    with seq_num_lock:
        curr_id = message_sequence_attributer(next_seq_num, seq_num_in_use)
    utility = req.utility
    device_ids = list(req.device_ids)
    command_data = list(req.command_data)

    data_out = []
    is_error = 1
    error_code = -1 # ERROR LINKED TO FORMAT OF INPUT. CODE ISN'T ACCEPTING THE INPUTS AND THROWS IT RIGHT BACK TO USER.

    try:
        data = entry_point_to_main_controller(utility, curr_id, [device_ids, command_data])

        for element in data:
            data_out_elem, is_error_elem, error_code_elem = send_and_acquire_data(curr_id, utility, element, 3)
            if error_code == -1:
                is_error = is_error_elem
                error_code = error_code_elem
            else:
                if is_error == 0:
                    is_error = is_error_elem
                    error_code = error_code_elem

            for dat in data_out_elem:
                data_out.append(dat)

    except Exception:
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

if __name__ == "__main__":
    rospy.init_node("mcuserial_node")
    rospy.loginfo("ROS <--> MCU Serial Python Node")

    # alimentation serial communication service
    serial_service = rospy.Service("alim_serial_com", alim_serial_com_srv, noeud_service_callback)

    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
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
