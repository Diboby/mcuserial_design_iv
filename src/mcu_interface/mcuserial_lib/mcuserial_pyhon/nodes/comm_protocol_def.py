#number_of_outputs = 3
#number_of_fuses = 3

password_for_admin_fct_mcu = 0x74AB

read_function_number = 0x1
write_function_number = 0x2
read_list_function_number = 0x3
write_list_function_number = 0x4

temperature_reg_number = 0x10 # READS FLOAT
tension_alim_reg_number = 0x01 # READS FLOAT
courant_alim_reg_number = 0x02 # READS FLOAT
puissance_alim_reg_number = 0x03 # READS FLOAT

fuse_isokay_reg_number = 0x30 # READS int REPRESENTING BOOL (0 = FALSE, ELSE 1)
#fuse_issurcharge_reg_number = 0x31 # READS int REPRESENTING BOOL (0 = FALSE, ELSE 1)
output_control_reg_number = 0x50 # READS OR WRITE int REPRESENTING BOOL (0 = FALSE, ELSE 1)

boot_time_reg = 0x60 # READ OR WRITE LIST (time in ms as float)
commit_to_flash_reg = 0x70 # WRITE PASSWORD TO COMMIT TO FLASH
reset_to_default_reg = 0x71 # WRITE PASSWORD TO RESET FLASH TO DEFAULT
reset_mcu_reg = 0x72 # WRITE PASSWORD TO RESET MCU COMPLETELY

fuse_index_start_list = 0x00
output_index_start_list = 0x00
boot_time_index_start_list = 0x00

def read_temp():
    mcu_function_number = read_function_number
    mcu_reg_number = temperature_reg_number
    index_start_list = None
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


def read_tension_alim():
    mcu_function_number = read_function_number
    mcu_reg_number = tension_alim_reg_number
    index_start_list = None
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


def read_courant_alim():
    mcu_function_number = read_function_number
    mcu_reg_number = courant_alim_reg_number
    index_start_list = None
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


def read_puissance_alim():
    mcu_function_number = read_function_number
    mcu_reg_number = puissance_alim_reg_number
    index_start_list = None
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


def write_output():
    mcu_function_number = write_list_function_number
    mcu_reg_number = output_control_reg_number
    index_start_list = output_index_start_list
    can_send_data = True
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


def read_output():
    mcu_function_number = read_list_function_number
    mcu_reg_number = output_control_reg_number
    index_start_list = output_index_start_list
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


def read_fuse_isokay_state():
    mcu_function_number = read_list_function_number
    mcu_reg_number = fuse_isokay_reg_number
    index_start_list = fuse_index_start_list
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data
    
def read_boot_time_reg():
    mcu_function_number = read_list_function_number
    mcu_reg_number = boot_time_reg
    index_start_list = boot_time_index_start_list
    can_send_data = False
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data

def write_boot_time_reg():
    mcu_function_number = write_list_function_number
    mcu_reg_number = boot_time_reg
    index_start_list = boot_time_index_start_list
    can_send_data = True
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data

def write_commit_to_flash():
    mcu_function_number = write_function_number
    mcu_reg_number = commit_to_flash_reg
    index_start_list = None
    can_send_data = True
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data

def write_reset_to_default():
    mcu_function_number = write_function_number
    mcu_reg_number = reset_to_default_reg
    index_start_list = None
    can_send_data = True
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data

def write_reset_mcu():
    mcu_function_number = write_function_number
    mcu_reg_number = reset_mcu_reg
    index_start_list = None
    can_send_data = True
    return mcu_function_number, mcu_reg_number, index_start_list, can_send_data


find_utility_params = {0: read_temp,
                       1: read_tension_alim,
                       2: read_courant_alim,
                       3: read_puissance_alim,
                       4: write_output,
                       5: read_output,
                       6: read_fuse_isokay_state,
                       7: read_boot_time_reg,
                       8: write_boot_time_reg,
                       9: write_commit_to_flash,
                       10: write_reset_to_default,
                       11: write_reset_mcu
                       }
