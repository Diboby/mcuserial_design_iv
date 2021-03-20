import sys
import time
import rospy
import Queue
import threading
import multiprocessing

from mcuserial_msgs.srv import alim_serial_com_srv, RequestParam

from mcuserial_python import SerialClient
from serial import SerialException
from mcuserial_msgs.msg import TopicInfo, dataTemplate


if __name__ == "__main__":
    rospy.init_node("mcuserial_node_test1")
    rospy.loginfo("ROS <--> MCU Serial Python Node Test")
    rospy.wait_for_service('alim_serial_com')
    print("alim_serial_com service available")

    try:
        alim_serial_service = rospy.ServiceProxy('alim_serial_com', alim_serial_com_srv)
        # TODO UT
        # TODO WHEN WRITING DATA, MINIMUM IS ALWAYS 4 BYTES, BECAUSE REGISTERS (TO VALIDATE NOW)
        # TODO ADD TIMEOUT
        # TODO FINISH PARSER
        # TODO LOGIC OF SENDING/ACK/NACK

        print()
        print("This is the py node in ROS for UT test for communication with service that administraste the MCU.")
        print("Please connect the MCU to this computer with a serial link attached to ttyUSB0.")
        print("Press ENTER to continue.")
        x = raw_input()

        print("[INFO] Testing communication with MCU.")
        for i in range(0, 12):
            resp = alim_serial_service(i, [0], [1])
            if resp.is_error:
                print("[ERROR] Function " + str(i) + " has had an error : " + str(resp.error_code))
            else:
                print("[INFO] Function " + str(i) + " has successfully returned a result.")

        print()
        print("Visual testing UT for reading functions :")
        print("Verify that the values below are correct. We read them from the MCU. If there's a problem with the value, it might be in the communication or in the MCU itself.")
        temperature = alim_serial_service(0, [], [])
        print("Temperature value : ", temperature)
        tension_alim = alim_serial_service(1, [], [])
        print("Alimentation voltage value : ", tension_alim)
        courant_alim = alim_serial_service(2, [], [])
        print("Alimentation electric current value : ", courant_alim)
        puissance_alim = alim_serial_service(3, [], [])
        print("Alimentation power value : ", puissance_alim)
        read_output = alim_serial_service(5, [0, 1, 2, 3], [])
        print("Switch states values : ", read_output)
        read_fuse_isokay_state = alim_serial_service(6, [0, 1, 2, 3], [])
        print("Fuse is okay values : ", read_fuse_isokay_state)
        read_boot_time = alim_serial_service(7, [0, 1, 2, 3], [])
        print("Boot time values : ", read_fuse_isokay_state)
        
        print()
        print("Testing write communication :")
        write_output = alim_serial_service(4, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13], [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        read_output = alim_serial_service(5, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13], [])
        if read_output.floats != [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]:
            print("[ERROR] : write_output didn't work. Reading supposed to be all ones, output is : " + str(read_output.floats))
        write_output = alim_serial_service(4, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        read_output = alim_serial_service(5, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13], [])
        if read_output.floats != [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
            print("[ERROR] : write_output didn't work. Reading supposed to be all zeros, output is : " + str(read_output.floats))
        write_boot_time = alim_serial_service(8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [0.1, 6.4, 12.0, 0, 0, 0, 0, 44.0, 0, 0, 1100000.659, 0, 111.051, 1655.445]) # TODO VALIDATE IN MAINCONTROLLER (MESSAGE CONSTRUCTOR FCT) IF FLOAT NUMBER ARE WELL CONSTRUCTED
        read_boot_time = alim_serial_service(7, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [])
        if read_boot_time.floats != [0.1, 6.4, 12.0, 0, 0, 0, 0, 44.0, 0, 0, 1100000.659, 0, 111.051, 1655.445]:
            print("[ERROR] : write_boot_time didn't work. Boot time is : " + str(read_boot_time.floats))
        write_boot_time = alim_serial_service(8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        read_boot_time = alim_serial_service(7, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [])
        if read_boot_time.floats != [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
            print("[ERROR] : write_boot_time didn't work. Reading supposed to be all zeros, boot time is : " + str(read_boot_time.floats))

        #write_commit_to_flash = alim_serial_service(9, [], [password_for_admin_fct_mcu])
        #write_reset_to_default = alim_serial_service(10, [], [password_for_admin_fct_mcu])
        #write_reset_mcu = alim_serial_service(11, [], [password_for_admin_fct_mcu])

        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)