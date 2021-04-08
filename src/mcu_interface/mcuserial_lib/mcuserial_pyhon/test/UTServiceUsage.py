# UT to test service ability to handle requests and communicate with MCU, and give correct request answers back.
import time
import rospy
from mcuserial_msgs.srv import alim_serial_com_srv

def check_equal_list(list1, list2):
    equals = isinstance(list1, list) and isinstance(list2, list) and (len(list1) == len(list2))
    for i in range(len(list1)):
        equals = equals and (list1[i] == list2[i])

    return equals


if __name__ == "__main__":
    rospy.init_node("mcuserial_node_test1")
    rospy.loginfo("ROS <--> MCU Serial Python Node Test")
    rospy.wait_for_service('alim_serial_com')

    try:
        alim_serial_service = rospy.ServiceProxy('alim_serial_com', alim_serial_com_srv)

        print("\nThis is the py node in ROS for UT test for communication with service that administraste the MCU.")
        print("Please connect the MCU to this computer with a serial link attached to ttyUSB0.")
        print("Press ENTER to continue.")
        x = raw_input()

        number_of_errors = 0
        number_of_success = 0
        print("[INFO] Testing communication with MCU.")
        for i in range(0, 15):
            resp = alim_serial_service(i, [0], [1])
            if resp.is_error:
                print("[ERROR] Function " + str(i) + " has had an error : " + str(resp.error_code))
                number_of_errors = number_of_errors + 1
            else:
                print("[INFO] Function " + str(i) + " has successfully returned a result.")
                number_of_success = number_of_success + 1

        print("\nVisual testing UT for reading functions :")
        print("Verify that the values below are correct. We read them from the MCU. If there's a problem with the value, it might be in the communication or in the MCU itself.")
        temperature = alim_serial_service(0, [], [])
        print("Temperature value : ", temperature)
        tension_alim = alim_serial_service(1, [], [])
        print("Alimentation voltage value : ", tension_alim)
        courant_alim = alim_serial_service(2, [], [])
        print("Alimentation electric current value : ", courant_alim)
        puissance_alim = alim_serial_service(3, [], [])
        print("Alimentation power value : ", puissance_alim)
        read_output = alim_serial_service(5, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], [])
        print("Switch states values : ", read_output)
        read_fuse_isokay_state = alim_serial_service(6, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], [])
        print("Fuse is okay values : ", read_fuse_isokay_state)
        read_boot_time = alim_serial_service(7, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], [])
        print("Boot time values : ", read_fuse_isokay_state)
        reset = alim_serial_service(13, [], [])
        print("Reset value : ", reset)
        time.sleep(1)
        temperature = alim_serial_service(0, [], [])
        print("Temperature value : ", temperature)
        read_uptime = alim_serial_service(14, [0], [1])
        print("Boot uptime tick values : ", read_uptime)
        
        print("\nTesting write communication : (if no prints, then test successful)")
        write_output = alim_serial_service(4, [0, 1], [1, 1])
        read_output = alim_serial_service(5, [0, 1], [])
        if not check_equal_list(list(read_output.floats), [1.0, 1.0]):
            print("[ERROR] : write_output didn't work. Reading supposed to be all ones, output is : " + str(read_output.floats))
            print(read_output)
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_output = alim_serial_service(4, [0, 1], [0, 0])
        read_output = alim_serial_service(5, [0, 1], [])
        if not check_equal_list(list(read_output.floats), [0.0, 0.0]):
            print("[ERROR] : write_output didn't work. Reading supposed to be all zeros, output is : " + str(read_output.floats))
            print(read_output)
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_boot_time = alim_serial_service(8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [0, 6, 12, 0, 0, 0, 0, 44, 0, 0, 1100000, 0, 111, 1655])
        read_boot_time = alim_serial_service(7, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [])
        if not check_equal_list(list(read_boot_time.floats), [0.0, 6.0, 12.0, 0.0, 0.0, 0.0, 0.0, 44.0, 0.0, 0.0, 1100000.0, 0.0, 111.0, 1655.0]):
            print("[ERROR] : write_boot_time didn't work. Boot time is : " + str(read_boot_time.floats))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_boot_time = alim_serial_service(8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        read_boot_time = alim_serial_service(7, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [])
        if not check_equal_list(list(read_boot_time.floats), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
            print("[ERROR] : write_boot_time didn't work. Reading supposed to be all zeros, boot time is : " + str(read_boot_time.floats))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_commit_to_flash = alim_serial_service(9, [], [])
        if write_commit_to_flash.is_error == True:
            print("[ERROR] : can't commit to flash, error occured : " + str(write_commit_to_flash.error_code))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_reset_to_default = alim_serial_service(10, [], [])
        if write_reset_to_default.is_error == True:
            print("[ERROR] : can't reset flash to default, error occured : " + str(write_reset_to_default.error_code))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_auto_reset_output = alim_serial_service(11, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1])
        read_auto_reset_output = alim_serial_service(12, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [])
        if not check_equal_list(list(read_auto_reset_output.floats), [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0]):
            print("[ERROR] : write_boot_time didn't work. Boot time is : " + str(read_auto_reset_output.floats))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_auto_reset_output = alim_serial_service(11, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        read_auto_reset_output = alim_serial_service(12, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], [])
        if not check_equal_list(list(read_auto_reset_output.floats), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
            print("[ERROR] : write_boot_time didn't work. Reading supposed to be all zeros, boot time is : " + str(read_auto_reset_output.floats))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        write_reset_mcu = alim_serial_service(13, [], [])
        if write_reset_mcu.is_error == True:
            print("[ERROR] : can't reset MCU, error occured : " + str(write_reset_mcu.error_code))
            number_of_errors = number_of_errors + 1
        else:
            number_of_success = number_of_success + 1
        print("Writing test completed.")

        print("\nInput malformation test to service : (supposed to be all True. If not, problem with input parsing)")

        resp = alim_serial_service(100, [0], [1])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1

        resp = alim_serial_service(4, [0, 1], [1])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1

        resp = alim_serial_service(4, [0], [1, 1])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1

        resp = alim_serial_service(4, [0], [-1])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1

        resp = alim_serial_service(4, [], [])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1

        resp = alim_serial_service(5, [], [])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1
                
        resp = alim_serial_service(-1, [], [])
        print(resp.is_error == True)
        print(resp.error_code == -1)
        if(resp.error_code == -1 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1

        print("Input malformation test completed.")


        print("\nLatency and bandwidth test :")
        all_times = []
        for i in range(50):
            bef = time.time()
            alim_serial_service(0, [], [])
            aft = time.time()
            all_times.append(aft-bef)
        moy_times = 0
        for i in all_times:
            moy_times = moy_times + i
        moy_times = moy_times / len(all_times)
        print("Mean time to send a temperature reading command and to receive the response : " + str(moy_times))

        print("\nDisconnect now the link with the MCU to test problem with connection.")
        print("Press ENTER to continue.")
        x = raw_input()
        resp = alim_serial_service(0, [], [])
        print(resp.is_error == True)
        print(resp.error_code == 5)
        if(resp.error_code == 5 and resp.is_error == True):
            number_of_success = number_of_success + 1
        else:
            number_of_errors = number_of_errors + 1
        print("Test completed, should have tried number_of_retrys retries and should print True 2 times.")


        print("\nUT completed.")
        print(str(number_of_success) + " functions have completed successfully.")
        print(str(number_of_errors) + " functions have failed.")
        print("Tests that have failed can be seen by searching for [ERROR] or FALSE")


        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)