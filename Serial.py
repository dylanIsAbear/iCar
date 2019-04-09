import serial.tools.list_ports


class Command(object):
    def __init__(self, speed, turn, port):
        self.command = speed+'a'+turn+'b0'
        self.port = serial.Serial(port='COM4', baudrate=115200, timeout=2)

    def set_command(self, speed, turn):
        self.command = speed+'a'+turn+'b0'

    def send_command(self):
        self.port.write(self.command)


