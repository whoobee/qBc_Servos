#!/usr/bin/env python
#
# *********     Ping Example      *********
#
#
# Available SC Servo model on this example : All models using Protocol SC
# This example is tested with a SC15/SC09 Servo, and an URT
#

import sys
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from scservo_sdk import *                   # Uses SC Servo SDK library

# Default setting
SCS_ID                  = 102                 # SC Servo ID : 14
BAUDRATE                = 1000000             # SC Servo default baudrate : 1000000
DEVICENAME              = '/dev/ttyAMA0'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)

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

# Try to ping the SC Servo
# Get SC Servo model number
print("Scanning for ID range from 100 to 253...")
for i in range(100,254):
    print("[ID:%03d]\r" % i, end="")
    scs_model_number, scs_comm_result, scs_error = packetHandler.ping(i)
    if scs_comm_result != COMM_SUCCESS:
        #print("[WARN][ID:%03d]%s" % (i, packetHandler.getTxRxResult(scs_comm_result)))
        pass
    else:
        print("[OK][ID:%03d] ping Succeeded. SC Servo model number : %d" % (i, scs_model_number))
    if scs_error != 0:
        print("[ERR][ID:%03d]%s" % (i, packetHandler.getRxPacketError(scs_error)))
    time.sleep(0.1)

# Close port
portHandler.closePort()

