from mcuserial_python import SerialClient
from mcuserial_python import mainTranslator as mt

mcu_serial_interface = SerialClient('/dev/ttyUSB0', 115200, 0.2, 2.0)

utility = 3
curr_id = 0
device_ids = []
command_data = []
data = mt.entry_point_to_main_controller(utility, curr_id, [device_ids, command_data])

# TODO WHEN WRITING DATA, MINIMUM IS ALWAYS 4 BYTES, BECAUSE REGISTERS (TO VALIDATE NOW)
# TODO TEST NACK WITH NON EXISTENT FUNCTION CALL

for element in data:
    #data_out_elem, is_error_elem, error_code_elem = send_and_acquire_data(curr_id, utility, element, rmt.number_of_retrys)
    element2 = bytearray(b'0000') + element + bytearray(b'0000')
    mt.print_hexstr(element2, 'sending ')
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(element)
    print(mt.extract_data_and_convert(utility, data_out))
    mcu_serial_interface.send(element2)
    msg_bytes = mcu_serial_interface.receive()
    mt.print_hexstr(msg_bytes, 'received ')
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(msg_bytes)
    print(mt.extract_data_and_convert(utility, data_out))
