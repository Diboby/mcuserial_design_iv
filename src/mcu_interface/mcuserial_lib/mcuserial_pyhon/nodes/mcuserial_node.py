#!/usr/bin/env python

import sys
import struct
import time
import rospy
import Queue
import threading
import multiprocessing

from mcuserial_msgs.srv import alim_serial_com_srv, RequestParam

from mcuserial_python import SerialClient
from mcuserial_python import mainTranslator as mt
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


# sending and receiving logic
# curr_id, the id of the request,
# utility, function number given to service by user
# data, the data generated with the module to convert the user request to bytes to be sent
# iteration_retry, the number of retrys left
# returns data_out, which is the data returned from the request
#         is_error, if there was an error
#         error_code, the error code if there was an error
def send_and_acquire_data(curr_id, utility, data, iteration_retry):
    data_out = []
    is_error = 0
    error_code = 0

    if not rospy.is_shutdown() and iteration_retry > 0:
        if mt.rmt.service_print_debug_info:
            mt.print_hexstr(data, 'sending ')
        noeud_write_queue.put((curr_id, data, utility not in mt.rmt.function_no_return)) # puts the request in the write queue for the thread to handle it
    else: # if rospy is down or too many retries
        is_error = 1
        error_code = 0x5

    msg_bytes = ''
    while not rospy.is_shutdown() and iteration_retry > 0 and utility not in mt.rmt.function_no_return:
        if node_reception_queue.empty():
            time.sleep(0.01) # wait for the thread to finish handling the request
        else:
            with read_node_Queue_lock:
                (idd, msg_bytes) = node_reception_queue.get() # when thread finishes handling a request
            if mt.rmt.service_print_debug_info:
                mt.print_hexstr(msg_bytes, 'received ')
            is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(msg_bytes) 

            if seq_num != curr_id or idd != curr_id: # if the handled request (by the thread) doesn't correspond to this request, we put it back in the queue
                node_reception_queue.put(msg_bytes)
                time.sleep(0.01)
                continue
            else:
                data_ok, data_out = mt.extract_data_and_convert(utility, data_out)
                if is_correct == 0 or is_nack == 1 or (is_ack == 0 and is_nack == 0) or not data_ok and not is_ack: # if problem with the request, try again
                    if is_nack == 1 and is_correct == 1: # we received a valid message, but there was a problem (nack)
                        if not data_ok:
                            nack_code = 5
                        else:
                            nack_code = data_out[0]
                    data_out, is_error, error_code = send_and_acquire_data(curr_id, utility, data, iteration_retry-1) # because there was a problem, we retry again the sending process
                    if is_error == 1 and is_nack == 1 and is_correct == 1: # if we still receive a problem (nack) after n retries, we only return the first nack code we encountered
                        error_code = nack_code

                break

    if utility in mt.rmt.function_no_return:
        time.sleep(2)

    return data_out, is_error, error_code


# This is the callback executed when a service call is made
# req is request received from service
# returns data_out, which is the data returned from the request
#         is_error, if there was an error
#         error_code, the error code if there was an error (wrongInputFormat = -1, comerr_regNotFound = 0x1, comerr_illegalAccess = 0x2, comerr_badFunction = 0x3, comerr_dataMissing = 0x4, tooManyRetries = 0x5, idattributionfailed = 0x6)
#                     user of service should only receive error -1, 4, 5 or 6. Others can be caused by internal register based communication handling, but shouldn't happen in a normal execution. If there is, problem with our code.
def noeud_service_callback(req): 
    utility = req.utility                   # Utility is the function number to be executed (p. ex. 0 is reading temperature)
    device_ids = list(req.device_ids)       # Device ids are used for list calls (p. ex., when writing switches values, to close or open them, we need to address them by index. the device ids are those indexes) (see reference manual for assignation between and index and devices)
    command_data = list(req.command_data)   # Command data is the data used to write in the MCU. (p. ex., when writing switches values, we want to open switches (open circuit) at device ids 0 and 1, command data is [0, 0]. If we want to close them (closed circuit), command data is [1, 1])

    if utility in mt.rmt.function_needs_password:
        command_data = [mt.rmt.password_for_admin_fct_mcu]

    data_out = []
    is_error = 1
    error_code = -1 # ERROR LINKED TO FORMAT OF INPUT. CODE ISN'T ACCEPTING THE INPUTS AND THROWS IT RIGHT BACK TO USER.

    try:
        with seq_num_lock:
            curr_id = message_sequence_attributer(next_seq_num, seq_num_in_use)             # we attribute an id to every request
        data = mt.entry_point_to_main_controller(utility, curr_id, [device_ids, command_data]) # this is the module to convert the user request to bytes to be sent.

        for element in data:    # data produced by entry_point_to_main_controller can be a list of requests, in the event that the device ids aren't continuous (p. ex. [0, 2, 3, 5, 6]). Access can only be made with continous indexes with the MCU.
            data_out_elem, is_error_elem, error_code_elem = send_and_acquire_data(curr_id, utility, element, mt.rmt.number_of_retrys) # Main logic function to send and receive data
            if error_code == -1: #if no exception, format is correct, so error_code(-1) isn't applicable. we take the error_code from sending and receiving logic instead. 
                is_error = is_error_elem
                error_code = error_code_elem
            else: # if we already took error_code from sending and receiving logic.
                if is_error == 0: # if there was no error with previous requests, take the new error_code, because this one might be an error.
                    is_error = is_error_elem
                    error_code = error_code_elem

            for dat in data_out_elem:
                data_out.append(dat)

    except ID_ATTRIBUTION_FAILED as e: # If there was a problem with id attribution (maybe all id's are attributed (try another time, when requests all timeout))
        print(e)
        with seq_num_lock:
            seq_num_in_use.remove(curr_id)  # When finished, ID removed
        return data_out, is_error, 6
    except Exception as e: # If there was any other problem, like error linked to format of input
        print(e)
        with seq_num_lock:
            seq_num_in_use.remove(curr_id)  # When finished, ID removed
        return data_out, is_error, error_code
   
    with seq_num_lock:
        seq_num_in_use.remove(curr_id)  # When finished, ID removed
    return data_out, is_error, error_code


# Thread function that's responsible to handle the serial client for all service calls (if multiple service calls are made at once, this ensures that only one of them is treated at a time)
def sendMessage(thread_event, serialClient):
    thread_event = thread_event
    while not rospy.is_shutdown() and not thread_event.is_set():
        if noeud_write_queue.empty():
            time.sleep(0.01)
        else:
            with write_node_Queue_lock:
                (idd, data, can_receive) = noeud_write_queue.get()

            serialClient.send(data)
            if can_receive:
                msg_bytes = serialClient.receive()
                is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(msg_bytes)
                if is_correct == 0: # WHEN BAD CRC, COULD BE CAUSED BY NOISE, SO WAIT FOR TIMEOUT, THEN TRY AGAIN
                    time.sleep(mt.rmt.serial_comm_logic_end_timeout)
                    msg_bytes = serialClient.receive()
                with read_node_Queue_lock:
                    node_reception_queue.put((idd, msg_bytes))


# Attributes an id to every requests
def message_sequence_attributer(next_seq_num, seq_num_in_use): # TODO CLEAR NUMS WHEN ALL FULL, BECAUSE TIMEOUT
    attributed_num = next_seq_num

    if set([attributed_num]).issubset(seq_num_in_use):
        notFound = True
        for i in range(1, mt.rmt.number_max_of_ids):
            test_num = (attributed_num + i) % mt.rmt.number_max_of_ids
            if set([test_num]).isdisjoint(seq_num_in_use):
                attributed_num = test_num
                notFound = False
                break

        if notFound:
            raise ID_ATTRIBUTION_FAILED

    next_seq_num = (next_seq_num + 1) % mt.rmt.number_max_of_ids
    seq_num_in_use.add(attributed_num)

    return attributed_num


# Creates the service and starts it.
if __name__ == "__main__":
    rospy.init_node("mcuserial_node")
    rospy.loginfo("ROS <--> MCU Serial Python Node")

    # alimentation serial communication service
    serial_service = rospy.Service("alim_serial_com", alim_serial_com_srv, noeud_service_callback)

    port_name = rospy.get_param('~port', mt.rmt.serial_device_port)
    baud = int(rospy.get_param('~baud', mt.rmt.serial_device_baud))

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
            mcu_serial_interface = SerialClient(port_name, baud, mt.rmt.serial_device_read_timeout, mt.rmt.serial_comm_logic_end_timeout)

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
