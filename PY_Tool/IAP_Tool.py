import serial.tools.list_ports
import time
import os
import sys
import math
from ymodem.Socket import ModemSocket

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
        ser = serial.Serial(available_ports[input_code_i], 460800)
        print("connect port: {}".format(available_ports[input_code_i]))
    except serial.SerialException as e:
        print("Error: {}".format(e))
        sys.exit(1)

    # search firmware path
    firmware_path = os.getcwd() + os.sep + 'build' + os.sep
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

    def read(size, timeout = 100) -> any:
        ser.timeout = timeout
        return ser.read(size)

    def write(data, timeout = 100) -> any:
        ser.write_timeout = timeout
        ser.write(data)
        ser.flush()

    ymodem_tran = ModemSocket(read, write)
    progress_bar = ProgressBar()

    while True:
        ser.write(b'force_mode')
        ser.flush()
        force_mode = False
        cnt = 0
        while True:
            t = ser.read()
            if force_mode != True:
                if (t == b'C'):
                    cnt = cnt + 1
                    if (cnt == 32):
                        force_mode = True
                        print('device switch to force mode')
                else:
                    cnt = 0
                    force_mode = False
            
            if force_mode == True:
                # YModem transmit firmware to device
                ymodem_tran.send(file_list[0], progress_bar.show)
                time.sleep(0.05)

main()