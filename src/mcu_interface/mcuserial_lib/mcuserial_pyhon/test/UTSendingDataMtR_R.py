import rospy
import struct
from mcuserial_msgs.srv import alim_serial_com_srv

# Send read command

rospy.init_node("mcuserial_node_test")
rospy.wait_for_service('alim_serial_com')

def test_bad_data(alim_serial_service):
    test_success = True
    resp = alim_serial_service(5, [0], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 5
    resp = alim_serial_service(5, [0], [])
    test_success = test_success and resp.floats[0] == 0x1 and resp.is_error == 0 and resp.error_code == 0
    if test_success:
        print("[SUCCESS] : ROS handles bad data")
    else:
        print("[ERROR] : ROS doesn't handle bad data")
    return test_success

def test_missing_data(alim_serial_service):
    test_success = True
    resp = alim_serial_service(3, [], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 5

    resp = alim_serial_service(5, [0], [])
    test_success = test_success and resp.floats[0] == 0x1 and resp.is_error == 0 and resp.error_code == 0

    resp = alim_serial_service(5, [0], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 5

    resp = alim_serial_service(5, [0], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 5

    resp = alim_serial_service(5, [0], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 5

    if test_success:
        print("[SUCCESS] : ROS handles missing data")
    else:
        print("[ERROR] : ROS doesn't handle missing data")
    return test_success

def test_bad_crc(alim_serial_service):
    test_success = True
    resp = alim_serial_service(3, [], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 0x5
    if test_success:
        print("[SUCCESS] : ROS handles bad crc")
    else:
        print("[ERROR] : ROS doesn't handle bad crc")
    return test_success


def test_noise_on_line(alim_serial_service):
    test_success = True
    for i in range(6):
        resp = alim_serial_service(3, [], [])
        test_success = test_success and struct.pack("f", resp.floats[0]) == b'\x9a\x19@C' and resp.is_error == 0 and resp.error_code == 0
    resp = alim_serial_service(3, [], [])
    test_success = test_success and resp.is_error == 1 and resp.error_code == 0x5
    if test_success:
        print("[SUCCESS] : ROS handles noise on line")
    else:
        print("[ERROR] : ROS doesn't handle correctly noise on line")
    return test_success


try:
        
        alim_serial_service = rospy.ServiceProxy('alim_serial_com', alim_serial_com_srv)
        

        number_of_total_fct = 4
        number_of_fct_success = 0
        number_of_fct_success = number_of_fct_success + int(test_noise_on_line(alim_serial_service))
        number_of_fct_success = number_of_fct_success + int(test_bad_crc(alim_serial_service))
        number_of_fct_success = number_of_fct_success + int(test_missing_data(alim_serial_service))
        number_of_fct_success = number_of_fct_success + int(test_bad_data(alim_serial_service))

        print("\nIn summary:")
        print(str(number_of_fct_success) + " functions have succeeded out of " + str(number_of_total_fct) + " functions.")
        print("Check prints for errors or successes.")


except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


