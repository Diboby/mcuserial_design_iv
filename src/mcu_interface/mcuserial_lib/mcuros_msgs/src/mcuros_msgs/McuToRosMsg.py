


class McuToRosMsg :

    def __init__(self):

        self.startBits = b'\x77'
        self.packetSize = b'\xae'
        self.function = b'\x01'
        self.registerNumber = b'\x10'
        self.offset = b'\x05'
        self.count = b'\x19'
        self.data = b'\x00\x01\x02\x03\x04\x05\x06\x07'
        self.crc16 = b"\x01\x10"
    
    def setData(self, data):
        self.data = data

    def concatenateMessage(self):
        result = self.startBits + self.packetSize + self.function + self.registerNumber + self.offset + self.count + self.data + self.crc16
        return result