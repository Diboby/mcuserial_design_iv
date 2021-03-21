from mcuserial_node import *

mcu_serial_interface = SerialClient('/dev/ttyUSB0', 115200, 1)

utility = 3
curr_id = 0
device_ids = []
command_data = []
data = entry_point_to_main_controller(utility, curr_id, [device_ids, command_data])

for element in data:
    #data_out_elem, is_error_elem, error_code_elem = send_and_acquire_data(curr_id, utility, element, rmt.number_of_retrys)
    element2 = bytearray(b'0000') + element + bytearray(b'0000')
    print_hexstr(element2, 'sending ')
    is_correct, seq_num, is_ack, is_nack, data_out = parse_correct_data(element)
    print(extract_data_and_convert(utility, data_out))
    mcu_serial_interface.send(element2)
    msg_bytes = mcu_serial_interface.receive()
    print_hexstr(msg_bytes, 'received ')
    is_correct, seq_num, is_ack, is_nack, data_out = parse_correct_data(msg_bytes)
    print(extract_data_and_convert(utility, data_out))
