import serial.tools.list_ports
import time
import os
import sys
import math

from ymodem.Protocol import ProtocolType
from ymodem.Socket import ModemSocket
from typing import Optional, Any, Union

ser = None

class ProgressBar:
    def __init__(self):
        self.bar_width = 50
        self.last_task_name = ""
        self.current_task_start_time = -1

    def show(self, task_index, task_name, total, success):
        if task_name != self.last_task_name:
            self.current_task_start_time = time.perf_counter()
            if self.last_task_name != "":
                print('\n', end="")
            self.last_task_name = task_name

        success_width = math.ceil(success * self.bar_width / total)

        a = "#" * success_width
        b = "." * (self.bar_width - success_width)
        progress = (success_width / self.bar_width) * 100
        cost = time.perf_counter() - self.current_task_start_time

        print(f"\r{task_index} - {task_name} {progress:.2f}% [{a}->{b}]{cost:.2f}s", end="")

def Port_Scan():
    ports = serial.tools.list_ports.comports()
    available_ports = []
    index = 0

    if len(ports) == 0:
        print("No device attach")
    else:
        for port, desc, hwid in sorted(ports):
            print("{}\t<----------->\t{}\t{}".format(index, port, desc))
            available_ports.append(port)
            index += 1

    return available_ports

def main():
    print("<------ Scanning for available ports... ------>")
    print("<------------- Input r ReScanning ------------>")
    while True:
        available_ports = Port_Scan()
        input_code = input("select index: ")
        if input_code == "r":
            print("<----- ReScanning for available ports... ----->")
            print("<------------- Input r ReScanning ------------>")
            continue

        try:
            input_code_i = int(input_code)
            if int(input_code_i) >= available_ports.__len__():
                print("input over range: {}".format(input_code_i))
                continue
            else:
                print("selected port: {}".format(available_ports[input_code_i]))
                break
        except ValueError:
            print("Invalid input:{}".format(input_code))
            continue

    # connect port
    try:
        ser = serial.Serial(available_ports[input_code_i], 460800, timeout=1)
        print("connect port: {}".format(available_ports[input_code_i]))
    except serial.SerialException as e:
        print("Error: {}".format(e))
        sys.exit(1)

    # search firmware path
    # firmware_path = '/Users/bit8/Desktop/develop/TDrone/TDrone_Bootloader/build'
    while True:
        firmware_path = input("input firmware file path: ")
        if not os.path.exists(firmware_path):
            print("firmware path not exist: {}".format(firmware_path))
        else:
            break

    file_list = []
    print("Firmware path {}".format(firmware_path))
    for root, dirs, files in os.walk(firmware_path):
        for file in files:
            if file.endswith('.bin'):
                file = os.path.join(root, file)
                file_list.append(file)

    print('found {} files in the firmware path'.format(file_list.__len__()))
    for i in range(file_list.__len__()):
        print('index {}  \t<---->\t  {}'.format(i, file_list[i]))

    def read(size: int, timeout: Optional[float] = 3) -> Any:
        ser.timeout = timeout
        return ser.read(size)

    def write(data: Union[bytes, bytearray], timeout: Optional[float] = 3) -> Any:
        ser.write_timeout = timeout
        ser.write(data)
        ser.flush()

    socket_args = {
        'packet_size': 128,
        'protocol_type': ProtocolType.YMODEM,
        'protocol_type_options': []
    }

    try:
        progress_bar = ProgressBar()
        ymodem_tran = ModemSocket(read, write, **socket_args)

        force_mode = False
        while not force_mode:
            print('set device to force mode send request')
            ser.write(b'force_mode')
            ser.flush()
            cnt = 0
            while True:
                t = ser.read()
                if t == b'C':
                    cnt = cnt + 1
                    if (cnt == 32):
                        force_mode = True
                        print('device switch to force mode')
                        break
                else:
                    force_mode = False
                    if len(t) == 0:
                        break
                
        # YModem transmit firmware to device
        print('YModem start trans firmware')
        ymodem_tran.send(file_list, progress_bar.show)
    finally:
        ser.close()
        print("\r\nSerial port closed")
        print("Firmware trans finish")

main()