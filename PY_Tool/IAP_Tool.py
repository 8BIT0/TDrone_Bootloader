import serial.tools.list_ports
import time
import os
import sys
import tkinter as tk

def Port_Scan():
    ports = serial.tools.list_ports.comports()
    available_ports = []
    index = 0
    for port, desc, hwid in sorted(ports):
        print(index, "\t<----------->\t", port, "\t", desc)
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
                print("input over range:", input_code_i)
                continue
            else:
                print("selected port:", available_ports[input_code_i])
                break
        except ValueError:
            print("Invalid input:", input_code)
            continue

    # connect port
    # try:
    #     ser = serial.Serial(available_ports[input_code_i], 115200)
    #     print("connect port:", available_ports[input_code_i])
    # except serial.SerialException as e:
    #     print("Error:", e)
    #     sys.exit(1)

main()