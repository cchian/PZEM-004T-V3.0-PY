import serial
import time
from threading import Timer

# Measured values
class PzemValues:
    def __init__(self):
        self.voltage = -1.0  # default value if data cannot be read
        self.current = -1.0
        self.power = -1.0
        self.energy = -1.0
        self.frequency = -1.0
        self.pf = -1.0
        self.alarms = 0

# Pzem-004t v3.0 class
class PZEM004TV30:
    def __init__(self, port, addr=0xF8):
        self.values = PzemValues()
        self._serial = serial.Serial(port)
        self._serial.baudrate = 9600
        self._serial.timeout = 0.5
        self._addr = addr
        self._last_read = 0
        self._timer = Timer(0, self._timer_tick)
        self._timer_interval = 200  # ms
        self._timer.start()
        self.init(addr)

    def _timer_tick(self):
        self.update_values()
        self.update(PzemEvent(self.values))
        self._timer = Timer(self._timer_interval / 1000.0, self._timer_tick)
        self._timer.start()

    @property
    def update_interval(self):
        return self._timer_interval

    @update_interval.setter
    def update_interval(self, value):
        self._timer_interval = value

    # Get most up to date values from device registers and cache them
    def update_values(self):
        response = bytearray(25)
        self.send_cmd8(CMD_RIR, 0x00, 0x0A, False)
        length = self.receive(response, 25)

        if length != 25:
            self.values.voltage = 0
            self.values.current = 0
            self.values.power = 0
            self.values.energy = 0
            self.values.frequency = 0
            self.values.pf = 0
            self.values.alarms = 0
            return False

        # Update the current values
        self.values.voltage = (response[3] << 8 | response[4]) / 10.0
        self.values.current = (response[5] << 24 | response[6] << 16 | response[7] << 8 | response[8]) / 1000.0
        self.values.power = (response[9] << 24 | response[10] << 16 | response[11] << 8 | response[12]) / 10.0
        self.values.energy = (response[13] << 24 | response[14] << 16 | response[15] << 8 | response[16]) / 1000.0
        self.values.frequency = (response[17] << 8 | response[18]) / 10.0
        self.values.pf = (response[19] << 8 | response[20]) / 100.0
        self.values.alarms = response[21] << 8 | response[22]

        if self.values.voltage > 260:
            self.values.voltage = 0
            self.values.current = 0
            self.values.power = 0
            self.values.energy = 0
            self.values.frequency = 0
            self.values.pf = 0
            self.values.alarms = 0
            return False

        self._last_read = int(round(time.time() * 1000))
        return True

    def init(self, addr):
        if addr < 0x01 or addr > 0xF8:
            addr = 0xF8  # default address if out of range
        self._addr = addr

    def send_cmd8(self, cmd, r_addr, val, check=False):
        send_buffer = bytearray(8)
        send_buffer[0] = self._addr
        send_buffer[1] = cmd
        send_buffer[2] = (r_addr >> 8) & 0xFF
        send_buffer[3] = r_addr & 0xFF
        send_buffer[4] = (val >> 8) & 0xFF
        send_buffer[5] = val & 0xFF
        self.set_crc(send_buffer, 8)
        self._serial.write(send_buffer)

        if check:
            resp_buffer = bytearray(8)
            length = self.receive(resp_buffer, 8)
            if length != 8:
                return False
            for i in range(8):
                if send_buffer[i] != resp_buffer[i]:
                    return False
        return True

    def receive(self, resp, length):
        start_time = time.time()
        index = 0
        while index < length and (time.time() - start_time) < 0.1:  # 100 ms timeout
            if self._serial.in_waiting > 0:
                resp[index] = ord(self._serial.read())
                index += 1
        if not self.check_crc(resp, index):
            return 0
        return index

    def set_crc(self, buf, length):
        if length <= 2:
            return
        crc = self.crc16(buf, length - 2)
        buf[length - 2] = crc & 0xFF
        buf[length - 1] = (crc >> 8) & 0xFF

    def check_crc(self, buf, length):
        if length <= 2:
            return False
        crc = self.crc16(buf, length - 2)
        return (buf[length - 2] | buf[length - 1] << 8) == crc

    def crc16(self, n_data, length):
        w_crc_table = [
            0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
            0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
            0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
            0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
            0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
            0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
            0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
            0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
            0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
            0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
            0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
            0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
            0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
            0xEE01, 0x2EC0, 0x2F80, 0xEE41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
            0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
            0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
            0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
            0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
            0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
            0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
            0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
            0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
            0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
            0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
            0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
            0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
            0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
            0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
            0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
            0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
            0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
            0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
        ]
        w_crc = 0xFFFF

        for c in range(length):
            w_crc = w_crc >> 8 ^ w_crc_table[(w_crc ^ n_data[c]) & 0xFF]

        return w_crc

class PzemEvent:
    def __init__(self, values):
        self.values = values

    def __str__(self):
        return f"Voltage: {self.values.voltage}V, Current: {self.values.current}A, Power: {self.values.power}W, Energy: {self.values.energy}kWh, Frequency: {self.values.frequency}Hz, Power Factor: {self.values.pf}, Alarms: {self.values.alarms}"

def main():
    pzem = PZEM004TV30('/dev/ttyUSB0')  # Replace with your actual serial port
    while True:
        if pzem.update_values():
            print(pzem.values)
        time.sleep(1)

if __name__ == "__main__":
    main()
