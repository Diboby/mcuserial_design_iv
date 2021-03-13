import serial
import math
import comm_protocol_def as rmt

crcTable1021 = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]


class ID_ATTRIBUTION_FAILED(Exception):
    pass


class INPUT_WRONG_FORMAT(Exception):
    pass


# data is uint8 array/iterable
def CRC16_Compute(data, size):
    crc = 0xFFFF
    for i in range(0, size):
        crc = (((crc << 8) & 0xFFFF) ^ crcTable1021[((crc >> 8) ^ (data[i])) & 0xFF])

    return crc
       

def message_constructor(mcu_reg_number, mcu_num_seq, mcu_function_number, data, list_offset, list_count, can_send_data):
    if not can_send_data:
        data = None
    
    mcu_header = 0x77

    list_size = 0
    if list_offset is not None:
        list_size = 2
    data_size = 0
    if data is not None:
        data_size = len(data)*4

    mcu_packetsize = 4 + list_size + data_size
    mcu_fused_seq_func = mcu_num_seq << 4 | mcu_function_number

    send_msg = [mcu_header, mcu_packetsize, mcu_fused_seq_func, mcu_reg_number]
    if list_offset is not None:
        send_msg.append(list_offset)
        send_msg.append(list_count)
    if data is not None:
        for i in range(len(data)):
            send_msg.append(data[i])
            send_msg.append(0x00)
            send_msg.append(0x00)
            send_msg.append(0x00)
    crc = CRC16_Compute(send_msg, len(send_msg))

    packet = bytearray()
    packet.append(mcu_header)
    packet.append(mcu_packetsize)
    packet.append(mcu_fused_seq_func)
    packet.append(mcu_reg_number)
    if list_offset is not None and list_count is not None:
        packet.append(list_offset)
        packet.append(list_count)
    if data is not None:
        data = int(str(bytearray(data)).encode('hex'), 16)
        taille = int(math.ceil((float(len(hex(data))) - 2) / 2))            
        mask = 0xFF
        liste = []
        for i in range(taille):
            liste.append(0x00)
            liste.append(0x00)
            liste.append(0x00)
            liste.append((data & mask) >> i * 8)
            mask = mask << 8
        for i in range(len(liste) - 1, -1, -1):
            packet.append(liste[i])
    packet.append(crc >> 8)
    packet.append(crc & 0xFF)

    return packet

    
# Entry point to abstraction layer, utility is the command number to be sent to MCU (ex : read temperature)
# arg_associated_data is the parameters of the command, contained in an array. [[ids of devices], [command data]]
# Ex : command, write output state of device 0 and 3, so that 0 is opened and 3 is closed [[0 3], [1 0]] .
# Ex : command is to read temperature, arg_associated_data needs to be [].
def entry_point_to_main_controller(utility, mcu_num_seq, arg_associated_data=[]):

    mcu_response = []
    try:
        if type(utility) is not int:
            raise INPUT_WRONG_FORMAT
        mcu_function_number, mcu_reg_number, index_start_list, can_send_data = rmt.find_utility_params[utility]()
        device_ids = None
        command_data = None
        if arg_associated_data is not None:
            if len(arg_associated_data) == 2:
                device_ids = arg_associated_data[0]
                command_data = arg_associated_data[1]
            elif len(arg_associated_data) == 0:
                pass
            else:
                raise INPUT_WRONG_FORMAT

        list_offset = None
        list_count = None
        if (mcu_function_number is rmt.read_list_function_number) or (
                mcu_function_number is rmt.write_list_function_number):

            if device_ids is None or not device_ids:
                raise INPUT_WRONG_FORMAT
            if can_send_data and (command_data is None or not command_data or len(device_ids) != len(command_data)):
                raise INPUT_WRONG_FORMAT

            # Determines if index are continuous or not, to send one or multiple messages
            device_ids.sort()
            idx_segments = []
            for i in range(len(device_ids) - 1):
                if (device_ids[i + 1] - device_ids[i]) != 1:
                    idx_segments.append(i + 1)

            # If index are continuous, send one message, containing all of them
            if not idx_segments:
                list_offset = device_ids[0] + index_start_list
                list_count = len(device_ids)
                mcu_response.append(message_constructor(mcu_reg_number,
                                                        mcu_num_seq,
                                                        mcu_function_number,
                                                        command_data, list_offset, list_count, can_send_data))
            else:
                idx_segments.insert(0, 0)
                idx_segments.append(len(device_ids) - 1)

                for i in range(len(idx_segments) - 1):

                    device_ids_i = device_ids[idx_segments[i]:idx_segments[i + 1]]
                    command_data_i = command_data[idx_segments[i]:idx_segments[i + 1]]

                    if idx_segments[i] == idx_segments[i + 1]:
                        device_ids_i = device_ids[idx_segments[i]]
                        command_data_i = [command_data[idx_segments[i]]]
                        list_offset = device_ids_i + index_start_list
                        list_count = 1
                    else:
                        list_offset = device_ids_i[0] + index_start_list
                        list_count = len(device_ids_i)

                    mcu_response_i = message_constructor(mcu_reg_number,
                                                            mcu_num_seq,
                                                            mcu_function_number,
                                                            command_data_i, list_offset, list_count, can_send_data)
                    mcu_response.append(mcu_response_i)

        else:
            mcu_response.append(message_constructor(mcu_reg_number,
                                                    mcu_num_seq,
                                                    mcu_function_number,
                                                    command_data, list_offset, list_count, can_send_data))

    except ID_ATTRIBUTION_FAILED:
        print("Too many requests at once. System is out of message ID's. Retry later.")
    except INPUT_WRONG_FORMAT:
        print(
            "Input to function entry_point_to_main_controller has the wrong format. Try again (Utility is int and "
            "arg_associated_data is a list of two lists)")

    return mcu_response

"""
abs = abstraction_layer()

test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
test = abs.entry_point_to_main_controller(4, [[0, 1, 3, 5], [1, 1, 1, 1]])
print(test)

"""
