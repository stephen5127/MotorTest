import serial

# -------------CMD---------------+
motorCommandList = {
    "version": 'v\r\n',
    "get_encoder": 'e\r\n',
    "enable_read": 'e1\r\n',
    "disable_read": 'e0\r\n',
    "set_position": 'g',
}

motorModeList = {
    "no_control": 'z0\r\n',
    "openloop_test": 'z1\r\n',
    "ele_cal": 'z2\r\n',
    "closeloop_test": 'z3\r\n',
    "speed_ctrl": 'z4\r\n',
    "position_ctrl": 'z5\r\n',
    "sweep_test": 'z6\r\n',
    "friction": 'z7\r\n',
    "last_entry": 'z8\r\n',
}


class motorObj(object):
    def __init__(self, f_port='COM7', f_bd=115200):
        self.com = serial.Serial(  # open the port
            port=f_port,
            baudrate=f_bd,
            timeout=0.5,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def getVersion(self):
        s_cmd = motorCommandList["version"].encode()
        self.com.write(bytearray(s_cmd))
        ret = self.com.read_until('\r')  # read until '\r' appears
        return (ret.decode("utf-8"))

    def getEncoder(self):
        s_cmd = motorCommandList["get_encoder"].encode()
        self.com.write(bytearray(s_cmd))
        ret = self.com.read_until('\r')  # read until '\r' appears
        return (ret.decode("utf-8"))

    def setPosition(self, position):
        s_cmd = (motorCommandList["set_position"] + str(position)).encode()
        self.com.write(bytearray(s_cmd))
        pass

    def setMotorMode(self, mode):
        s_cmd = mode.encode()
        self.com.write(bytearray(s_cmd))
        pass

    def continuousEncoding(self, enable):
        if enable:
            s_cmd = motorCommandList["enable_read"].encode()
            self.com.write(bytearray(s_cmd))
        else:
            s_cmd = motorCommandList["disable_read"].encode()
            self.com.write(bytearray(s_cmd))

    def readBytes(self, decode=True):
        ret = self.com.read_until('\r')  # read until '\r' appears
        if decode:
            return ret.decode("utf-8")
        else:
            return ret

    def readUntilEmpty(self):
        read_buff = ''.encode()

        while self.checkBuffer():
            tmp = self.readBytes(decode=False)
            read_buff += tmp

        return read_buff.decode('utf-8')

    def clearReadBuff(self):
        self.com.reset_input_buffer()

    def checkBuffer(self):
        return self.com.in_waiting > 0

