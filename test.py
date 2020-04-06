#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os
from dynamixelSDK.src.dynamixel_sdk import *                    # Uses Dynamixel SDK library


# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_MOVING_SPEED       = 32
ADDR_AX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_AX_LED_RED             = 1
LEN_AX_GOAL_POSITION       = 2
LEN_AX_PRESENT_POSITION    = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1 = {'ID':1,'minPos':255,'maxPos':767,'maxSpeed':100,'goalPos':512}              # Dynamixel#1 ID : 1
DXL2 = {'ID':2,'minPos':205,'maxPos':819,'maxSpeed':100,'goalPos':512}              # Dynamixel#1 ID : 2
DXL3 = {'ID':3,'minPos':105,'maxPos':800,'maxSpeed':100,'goalPos':512}              # Dynamixel#1 ID : 3
DXL4 = {'ID':4,'minPos':50,'maxPos':793,'maxSpeed':100,'goalPos':512}               # Dynamixel#1 ID : 4
motors = [DXL1, DXL2, DXL3, DXL4]
BAUDRATE                    = 1000000                                               # Dynamixel default baudrate : 1000000
DEVICENAME                  = '/dev/tty.usbmodem14201'                              # Check which port is being used on your controller

TORQUE_ENABLE               = 1                                                     # Value for enabling the torque
TORQUE_DISABLE              = 0                                                     # Value for disabling the torque
AX_MOVING_STATUS_THRESHOLD = 20                                                     # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque and Speed
for m in motors:
    packetHandler.write1ByteTxRx(portHandler, m['ID'], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    if ax_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(ax_comm_result))
    elif ax_error != 0:
        print("%s" % packetHandler.getRxPacketError(ax_error))
    else:
        print("Dynamixel has been successfully connected")
    print(portHandler, m['ID'], 24, 1)
    packetHandler.write2ByteTxRx(portHandler, m['ID'], ADDR_AX_MOVING_SPEED, m['maxSpeed'])
    print(portHandler, m['ID'], ADDR_AX_MOVING_SPEED, m['maxSpeed'])

while 1:
    # print("Press any key to continue! (or press ESC to quit!)")
    # if getch() == chr(0x1b):
    #     break

    # Write goal position
    for m in motors:
        print (portHandler, m['ID'], ADDR_AX_GOAL_POSITION, m['goalPos'])
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, m['ID'], ADDR_AX_GOAL_POSITION, m['goalPos'])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, m['ID'], ADDR_AX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (m['ID'], m['goalPos'], dxl_present_position))

            if not abs(m['goalPos'] - dxl_present_position) > AX_MOVING_STATUS_THRESHOLD:
                break
    break
while True:
    print ("hi")
    for m in motors:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, m['ID'], ADDR_AX_GOAL_POSITION, m['goalPos'])
        while True:
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, m['ID'], ADDR_AX_PRESENT_POSITION)
            if not abs(m['goalPos'] - dxl_present_position) > AX_MOVING_STATUS_THRESHOLD:
                print(m['ID'], dxl_present_position)
                break


# Disable Dynamixel Torque
for m in motors:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, m['ID'], ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
