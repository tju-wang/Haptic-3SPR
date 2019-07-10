#!/usr/bin/python3
# coding=utf-8
"""
created by MrWang_tju  2018.12.06
for control the haptic device
V1.0.0

2019.2.25
增加 forcesw函数--》选择是否使能某类型力补偿
2019.4.13
0.解决同时使用多个串口时，其他设备占用串口导致得COM编号读取失败  修改为不打开串口，直接查看COM编号的方式


"""

import serial

import serial.tools.list_ports
import time

import serial.tools.list_ports as ports


def open_ser(baudrate = 115200,time = 0.01):
    global uart
    port_list = list(serial.tools.list_ports.comports())
    # print([comport.device for comport in serial.tools.list_ports.comports()])
    port_list_name = [comport.device for comport in ports.comports()]
    print(port_list_name)
    # print(port_list)
    # ListPortInfo
    if len(port_list) <= 0:
        print ("The Serial port can not be found")
    else:
        print("%d serial can be found!"%(len(port_list)))
        print("Please input the serial sort:")
        num = int(input())
        port_list_0 = list(port_list[num])
        port_serial = port_list_0[0]
        uart = serial.Serial(port_serial, baudrate, timeout=time)
        if uart.isOpen() is True:
            print(uart.name,"is Opened '%d'"%(uart.baudrate))



"""
function：
    串口写数据
Args:
    data[]
    
Return：
    None

"""


def write_data(data=[]):
    if len(data) == 11:
        data[9] = 0
        sum = 0
        for i in range(8):
            sum =sum + data[i + 1]
        data[9] = sum % 100
        # print(data)
        uart.write(data)

    else:
        uart.write(data)


def read_data(num=15):
    # i = 10  # 经过测试，发现正常接收16位耗时大概为500，这里设置1000用来保证数据接收完成
    # byte_list = []
    # byte = 0
    # n_s = True
    byte_list = uart.readall()  #timeout 来控制读取时间

    '''
            if uart.any() > 0 and n_s:
            if list(uart.read(1))[0] == 123:
                n_s = False
                byte_list.append(123)
                time.sleep(0.01)
    while uart.any() > 0:
        byte_list.append(uart.readchar())
    '''
    return  byte_list
    # if len(byte_list) == num:
    #     return byte_list
    # else:
    #     print("接收的数据有误:")
    #     print(byte_list)
    #     return []


"""
function：
    使能或失能驱动板
Args:
    state == 1  使能
          == 0  失能
    
Return：
    None

"""
def board_enable(state = 1):
    data = [0,0,0,0,0,0,0,0,0,0,0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x10
    if state == 1:
        data[3] = 0x11
    else:
        data[3] = 0x10
    data[10] = 0x7D
    write_data(data)


"""
function：
    使能或失能驱动芯片
Args:
    state == 1  使能
          == 0  失能

Return：
    None

"""


def motor_enable(state=1):
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x10
    if state == 1:
        data[3] = 0x11
    else:
        data[3] = 0x10
    data[10] = 0x7D
    write_data(data)

"""
function：
    电机PWM设置
Args:
    id: 电机编号
    PwmCValue：电机pwm值 0-1000
    
Return：
    None

"""

def set_pwmall(PwmValue1 = 500,PwmValue2 = 500,PwmValue3 = 500):
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x20
    data[3] = PwmValue1//100
    data[4] = PwmValue1%100
    data[5] = PwmValue2//100
    data[6] = PwmValue2%100
    data[7] = PwmValue3//100
    data[8] = PwmValue3%100
    data[10] = 0x7D
    write_data(data)

def set_pwm(id = 1, PwmValue1 = 500):
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = id
    data[2] = 0x20
    data[3] = PwmValue1//100
    data[4] = PwmValue1%100
    data[5] = 0
    data[6] = 0
    data[7] = 0
    data[8] = 0
    data[10] = 0x7D
    write_data(data)

def set_mode(mode = 500):
    """
    设置模式
    #define		CTL_FREE		0x10	//free Mode
    #define		CTL_ENABLE		0x11	//board enable
    #define 	CTL_PUSHPULL	0x12	//

    :param mode:
    :return:
    """
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0
    data[2] = 0x40
    data[3] = mode
    data[4] = 0
    data[5] = 0
    data[6] = 0
    data[7] = 0
    data[8] = 0
    data[10] = 0x7D
    write_data(data)



def get_encoder(id=1):
    """
    function：
        编码器数据回读
    Args:
        id: 无意义
        编码器读数

    Return：
        None

    """
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = id
    data[2] = 0x30
    data[3] = 0x20
    data[10] = 0x7D
    write_data(data)
    byte_list = read_data(15)

    # print (byte_list)
    data = byte_list
    if len(byte_list) >= 15:
        ca = (data[1] + data[2] + data[3] + data[4] + data[5]+data[6]+data[7]+data[8]
              +data[9]+data[10]+data[11]+data[12]+data[13]) % 100
        if byte_list[14] == ca and byte_list[0] == 123:
            type = byte_list[1]
            M1EnCounter = UartDataConver(byte_list[2],byte_list[3],byte_list[4])
            M2EnCounter = UartDataConver(byte_list[5],byte_list[6],byte_list[7])
            M3EnCounter = UartDataConver(byte_list[8],byte_list[9],byte_list[10])
            print ("Encoder  M1 ", M1EnCounter, "  M2 ", M2EnCounter, "  M3 ", M3EnCounter)
        else:
            print("返回的数据校验失败")
            return False
    else:
        print(byte_list)
        print("返回的数据位数不够!")
        return False


def get_pwm(id=1):
    """
    function：
        编码器数据回读
    Args:
        id: 无意义
        PwmCValue：电机pwm值 0-1000

    Return：
        None

    """
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = id
    data[2] = 0x31
    data[10] = 0x7D
    write_data(data)
    byte_list = read_data(15)

    # print (byte_list)
    data = byte_list
    if len(byte_list) >= 15:
        ca = (data[1] + data[2] + data[3] + data[4] + data[5]+data[6]+data[7]+data[8]
              +data[9]+data[10]+data[11]+data[12]+data[13]) % 100
        if byte_list[14] == ca and byte_list[0] == 123:
            type = byte_list[1]
            M1PWM = UartDataConver(byte_list[2],byte_list[3],byte_list[4])
            M2PWM = UartDataConver(byte_list[5],byte_list[6],byte_list[7])
            M3PWM = UartDataConver(byte_list[8],byte_list[9],byte_list[10])
            print ("PWM  M1: ",M1PWM,"  M2 : ",M2PWM,"  M3 : ",M3PWM)
        else:
            print("返回的数据校验失败")
            return False
    else:
        print(byte_list)
        print("返回的数据位数不够!")
        return False

def get_pwm_times(tim = 0.2):
    while 1:
        get_pwm(0)
        time.sleep(tim)


"""
function：
    选择不同种类的力是否使能
Args:
    state == 1  使能
          == 0  失能
          
#define		CMD_ForceSwitch		0x41	//不同种类力补偿类型
#define		CTL_StaFreFlag		0x11
#define		CTL_DynFreFlag		0x12
#define		CTL_GraFlag			0x13
#define		CTL_InerFlag		0x14

Return：
    None

"""


def force_sw(num = 0,state=1):
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x41
    if(num==0):
        print("num = 1-4 对应  1.静摩擦 2.滑动摩擦 3.重力 4.惯性力")
    else:
        if num == 1:
            data[3] = 0x11
        elif num == 2:
            data[3] = 0x12
        elif num == 3:
            data[3] = 0x13
        elif num == 4:
            data[3] = 0x14
        data[4] = state
        data[10] = 0x7D
        write_data(data)

def write_flash(num = 1,value = 0):
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x50
    data[3] = 0x11
    data[4] = num

    data[5] = int(value/256)
    data[6] = value%256
    data[10] = 0x7D
    write_data(data)

"""
输出flash表内容

"""
def flash():
    print("力反馈设备 固定参数FLASH表"
          "0,	//1.产品型号\n"
          "0,	//2.软件版本											2\n"
          "0,	//3.机械结构版本										3\n"
          "0,	//4.电路硬件版本										4\n"
          "500,   //5.PWMMID-ui  	电机零扭力参考PWM值				5\n"
          "15,	//6.PULL_PWM-ui	电机同向推拉时 增加的pwm值			6\n"
          "54,	//7.SigPWMPulse-ui 检测到转动后  单电机摩擦力补偿	7\n"
          "30,	//8.StopPwm-ui	检测到静止时  正反向的摩擦力波动	8\n"
          "88,	//9.PLPH_Para_k-f	推拉随速度变化力补偿值 一次函数 k值	9\n"
          "4,	//10.PLPH_Para_b-i										10\n"
          "//return (float)(0.PLPH_Para_k*sp+PLPH_Para_b);  返回本项计算PWM值\n"
          "6,	//11.重力补偿参数                                             \n"
          "100,	//12.InertiaPara-f   惯性力计算 与加速度相乘的系数              \n"
          )

def read_flash(num = 1):
    """
    function：
        FLASH表数据回读
    Args:
        num: FLASH 顺序

    Return：
        None

    """
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x50
    data[3] = 0x10
    data[4] = num
    data[10] = 0x7D
    write_data(data)
    byte_list = read_data(15)
    # if num>0 and num<=6:
    #     part = 1
    # elif num>6 and num<=12:
    #     part = 2
    # elif num>12 and num<=18:
    #     part = 3
    # elif num>18 and num<=24:
    #     part = 4
    print (byte_list)
    data = byte_list
    if len(byte_list) >= 15:
        ca = (data[1] + data[2] + data[3] + data[4] + data[5]+data[6]+data[7]+data[8]
              +data[9]+data[10]+data[11]+data[12]+data[13]) % 100
        if byte_list[14] == ca and byte_list[0] == 123:
            type = byte_list[1]
            if type == 0x30:
                Value = data[((num-1)%6)*2+2] *256 +data[((num-1)%6)*2+3]
                print ("FLASH表 ",num," 对应的值为",Value)
            else:
                print ("返回数据非FLASH表")
        else:
            print("返回的数据校验失败")
            return False
    else:
        print(byte_list)
        print("返回的数据位数不够!")
        return False

def flash_init():
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x50
    data[3] = 0x13
    data[10] = 0x7D
    write_data(data)

def UartDataConver(num1=0, num2=0, num3=0):
    if(num1//128==1) is True:
        num1 = (num1 %128)
        data = -(num1*65536 + num2*256 + num3)
    else:
        data = num1*65536 + num2*256 + num3

    return data


def debug(state=1):
    """
    function：
        打开或关闭调试模式
    Args:
        state == 1  使能
              == 0  失能

    Return：
        None

    """
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    data[0] = 0x7B
    data[1] = 0x79
    data[2] = 0x80
    if state == 1:
        data[3] = 0x11
    else:
        data[3] = 0x10
    data[10] = 0x7D
    write_data(data)



























