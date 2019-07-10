# coding=utf-8
# !/usr/bin/env python  


import serial

import serial.tools.list_ports


def open_ser(baudrate=115200, time=64):
    global uart
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) <= 0:
        print ("The Serial port can not be found")
    else:

        print("%d serial can be found!" % (len(port_list)))
        print(port_list)

        for i in range(len(port_list)):
            port_list_0 = list(port_list[i])
            port_serial = port_list_0[0]
            uart = serial.Serial(port_serial, baudrate, timeout=time)
            if uart.isOpen() is True:
                print(uart.name, "Can be open>")
                uart.close()
        num = int(input())
        port_list_0 = list(port_list[num])
        port_serial = port_list_0[0]
        uart = serial.Serial(port_serial, baudrate, timeout=time)
        if uart.isOpen() is True:
            print(uart.name, "is Opened '%d'" % (uart.baudrate))
