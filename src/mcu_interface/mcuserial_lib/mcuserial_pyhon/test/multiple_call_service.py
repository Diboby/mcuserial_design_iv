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

noeud_write_queue = Queue.Queue()
noeud_reception_queue = Queue.Queue()



def noeud_service_callback(req):
    curr_id = message_sequence_attributer(next_seq_num, seq_num_in_use)
    # data = abstraction_layer.entry_point_to_main_controller(utility, curr_id, [device_ids, command_data])
    s = getattr(req, any)
    print(s)
    data = ""
    noeud_write_queue.put(data)


    time.sleep(10)

    while not rospy.is_shutdown() :
        pass 

    
    seq_num_in_use.remove(curr_id)  # When finished, ID removed



# alimentation serial communication service
serial_service = rospy.Service("alim_serial_com_test", RequestParam, noeud_service_callback)


if __name__ == "__main__":
    rospy.init_node("mcuserial_node_test")
    rospy.loginfo("ROS <--> MCU Serial Python Node Test")


    while not rospy.is_shutdown():
        pass