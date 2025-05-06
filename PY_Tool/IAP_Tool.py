import serial.tools.list_ports
import time
import os
import sys
import tkinter as tk

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

    # while True:
    #     ser.write('Force_Mode')
    #     while True:
    #         t = ser.read()
    #         print(t)

main()