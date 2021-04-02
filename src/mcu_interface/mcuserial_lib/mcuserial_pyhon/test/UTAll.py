import os
import threading

def service_usage():
    os.system("python src/mcu_interface/mcuserial_lib/mcuserial_pyhon/test/UTServiceUsage.py")

os.system("python src/mcu_interface/mcuserial_lib/mcuserial_pyhon/test/UTSendingData.py")

noeud_send_msg_thread = threading.Thread(target=service_usage, args=())
noeud_send_msg_thread.daemon = True
noeud_send_msg_thread.start()


os.system("python src/mcu_interface/mcuserial_lib/mcuserial_pyhon/nodes/mcuserial_node.py")

noeud_send_msg_thread.join()
