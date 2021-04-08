# UT to test MCU ability to communicate

from mcuserial_python import SerialClient
from mcuserial_python import mainTranslator as mt
import time

mcu_serial_interface = SerialClient('/dev/ttyUSB0', 115200, 0.2, 2.0)

def send_my_dat(data):
    recv = []
    for element in data:
        mcu_serial_interface.send(element)
        msg_bytes = mcu_serial_interface.receive()
        recv.append(msg_bytes)
    return recv

#comerr_regNotFound = 0x1, comerr_illegalAccess = 0x2, comerr_badFunction = 0x3, comerr_dataMissing = 0x4


# WHEN WRITING DATA, MINIMUM IS ALWAYS 4 BYTES, BECAUSE REGISTERS (TO VALIDATE NOW)
def test_mcu_min_bytes():
    test_success = True

    packet = bytearray()
    packet.append(0x77)
    packet.append(7)
    packet.append(0x04)
    packet.append(0x50)
    packet.append(0x0)
    packet.append(0x1)
    packet.append(0x1)
    packet.append(0x7B)
    packet.append(0x87)
    recv = send_my_dat([packet])
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x4 # TODO MCU DOESN'T RESPONSE CORRECTLY, NACK CODE IS 1 BYTE INSTEAD OF 4
    if test_success:
        print("[SUCCESS] : MCU handles bad data formatting (1 byte instead of 4)")
    else:
        print("[ERROR] : MCU doesn't handle bad data formatting (1 byte instead of 4)")
    return test_success

# TEST BAD CRC
def test_bad_crc():
    test_success = True
    packet = bytearray()
    packet.append(0x77)
    packet.append(0x0a)
    packet.append(0x04)
    packet.append(0x50)
    packet.append(0x0)
    packet.append(0x1)
    packet.append(0x1)
    packet.append(0x0)
    packet.append(0x0)
    packet.append(0x0)
    packet.append(0xf4)
    packet.append(0xc6)

    recv = send_my_dat([packet])
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 0
    if test_success:
        print("[SUCCESS] : MCU handles bad crc from request")
    else:
        print("[ERROR] : MCU doesn't handle bad crc from request")
    return test_success

# TEST DATA MISSING
def test_mcu_data_missing():
    test_success = True
    data = [mt.message_constructor(0x10, 0, 0x1, [1], None, None, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x4

    data = [mt.message_constructor(0x50, 0, 0x4, [], 0x0, 0x1, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x4 # TODO SUPPOSED TO RECEIVE NACK, BECAUSE MISSING WRITING DATA

    data = [mt.message_constructor(0x50, 0, 0x4, [1], 0x0, 0x2, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x4 # TODO SUPPOSED TO RECEIVE NACK, BECAUSE MISSING WRITING DATA
    if test_success:
        print("[SUCCESS] : MCU handles missing data from request")
    else:
        print("[ERROR] : MCU doesn't handle missing data from request")
    return test_success

def test_mcu_fct_number():
    test_success = True
    data = [mt.message_constructor(0x10, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x10, 0, 0x2, [1], None, None, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2
    data = [mt.message_constructor(0x10, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2
    data = [mt.message_constructor(0x10, 0, 0x4, [10], 0x0, 0x1, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2

    data = [mt.message_constructor(0x30, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    #test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2 # TODO THIS ISN'T GOOD (READING A LIST REG WITH A SIMPLE REG READ COMMAND) GIVES NACK (REG NOT FOUND)
    data = [mt.message_constructor(0x30, 0, 0x2, [1], None, None, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2
    data = [mt.message_constructor(0x30, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x30, 0, 0x4, [10], 0x0, 0x1, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2

    data = [mt.message_constructor(0x50, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    #test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2 # TODO THIS ISN'T GOOD (READING A LIST REG WITH A SIMPLE REG READ COMMAND) GIVES NACK (REG NOT FOUND)
    data = [mt.message_constructor(0x50, 0, 0x2, [1], None, None, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    #test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2 # TODO THIS ISN'T GOOD (WRITING A LIST REG WITH A SIMPLE REG WRITE COMMAND) GIVES ACK
    data = [mt.message_constructor(0x50, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x50, 0, 0x4, [1], 0x0, 0x1, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False

    data = [mt.message_constructor(0x70, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2
    data = [mt.message_constructor(0x70, 0, 0x2, [mt.rmt.password_for_admin_fct_mcu], None, None, True)] # TODO PROBLEM, NEEDS SLEEP?
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x70, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0]) # TODO IM RECEIVING AN ACK WHEN FULL SPEED, AND NACK WHEN ADDED WITH SLEEP?
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2
    data = [mt.message_constructor(0x70, 0, 0x4, [1], 0x0, 0x1, True)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x4, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x2

    data = [mt.message_constructor(0x10, 0, 0x5, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x3
    data = [mt.message_constructor(0x10, 0, 0xb, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x3
    data = [mt.message_constructor(0x10, 0, 0xc, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x3
    data = [mt.message_constructor(0x10, 0, 0xa, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == False and is_nack == True and nack_code[0] == 0x3
    if test_success:
        print("[SUCCESS] : MCU handles right and wrong function numbers")
    else:
        print("[ERROR] : MCU doesn't handle right and wrong function numbers")
    return test_success

def test_mcu_reg_number(): 
    test_success = True
    data = [mt.message_constructor(0x10, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x01, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x02, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x03, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x30, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x50, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x51, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x60, 0, 0x3, [], 0x0, 0x1, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x74, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x35, 1, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 1 and is_ack == False and is_nack == True and nack_code[0] == 0x1
    data = [mt.message_constructor(0x22, 2, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    _, nack_code = mt.extract_data_and_convert(0x9, data_out)
    test_success = test_success and is_correct == 1 and seq_num == 2 and is_ack == False and is_nack == True and nack_code[0] == 0x1
    if test_success:
        print("[SUCCESS] : MCU handles right and wrong register numbers")
    else:
        print("[ERROR] : MCU doesn't handle right and wrong register numbers")
    return test_success

# TEST IF MCU_NUM_SEQ IS CONSIDERED
def test_mcu_num_seq():
    test_success = True
    data = [mt.message_constructor(0x10, 0, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x10, 9, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 9 and is_ack == True and is_nack == False
    data = [mt.message_constructor(0x10, 15, 0x1, [], None, None, False)]
    recv = send_my_dat(data)
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv[0])
    test_success = test_success and is_correct == 1 and seq_num == 15 and is_ack == True and is_nack == False
    if test_success:
        print("[SUCCESS] : MCU handles different sequence numbers")
    else:
        print("[ERROR] : MCU doesn't handle different sequence numbers")
    return test_success


# TEST NOISE ON LINE
def test_noise_on_line():
    test_success = True
    data = [bytearray(b'0000w\x04\x01\x03H\n0000'), bytearray(b'w\x04\x01\x03H\n0000'), bytearray(b'0000w\x04\x01\x03H\n'), bytearray(b'00w\x04\x0100000000000000w\x04\x01\x03H\n0000')]
    recv = send_my_dat(data)
    for dat in recv:
        is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(dat)
        test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False

    data1 = bytearray(b'w\x04\x01')
    data2 = bytearray(b'\x03H\n')
    mcu_serial_interface.send(data1)
    mcu_serial_interface.send(data2)
    recv = mcu_serial_interface.receive()
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv)
    test_success = test_success and is_correct == 1 and seq_num == 0 and is_ack == True and is_nack == False

    mcu_serial_interface.send(data1)
    time.sleep(0.1)
    mcu_serial_interface.send(data2)
    recv = mcu_serial_interface.receive()
    is_correct, seq_num, is_ack, is_nack, data_out = mt.parse_correct_data(recv)
    test_success = test_success and is_correct == 0
    if test_success:
        print("[SUCCESS] : MCU handles noise on line")
    else:
        print("[ERROR] : MCU doesn't handle correctly noise on line")
    return test_success

number_of_total_fct = 7
number_of_fct_success = 0
number_of_fct_success = number_of_fct_success + int(test_noise_on_line())
number_of_fct_success = number_of_fct_success + int(test_mcu_num_seq())
number_of_fct_success = number_of_fct_success + int(test_mcu_reg_number())
number_of_fct_success = number_of_fct_success + int(test_mcu_fct_number())
number_of_fct_success = number_of_fct_success + int(test_mcu_data_missing())
number_of_fct_success = number_of_fct_success + int(test_bad_crc())
number_of_fct_success = number_of_fct_success + int(test_mcu_min_bytes())

print("\nIn summary:")
print(str(number_of_fct_success) + " functions have succeeded out of " + str(number_of_total_fct) + " functions.")
print("Check prints for errors or successes.")
