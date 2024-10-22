import dynamixel_sdk as dxl

ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
PROTOCOL_VERSION = 1.0
DXL_IDS = [1,2,3,4]
DEVICENAME = "COM4"
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)
for DXL_ID in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 100)

# Move motor 1
# Default values
pos1 = 512
pos2 = 512
pos3 = 512
pos4 = 512

packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, pos1)
packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, pos2)
packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION, pos3)
packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION, pos4)



while True:

    print("Moving motor 1 to:")
    pos1x = input()
    if pos1x == "":
        pos1 = pos1
    else:
        pos1 = int(pos1x)
    packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, pos1)

    print("Moving motor 2 to:")
    pos2x = input()
    if pos2x == "":
        pos2 = pos2
    else:
        pos2 = int(pos2x)
    packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, pos2)

    print("Moving motor 3 to:")
    pos3x = input()
    if pos3x == "":
        pos3 = pos3
    else:
        pos3 = int(pos3x)
    packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION, pos3)

    print("Moving motor 4 to:")
    pos4x = input()
    if pos4x == "":
        pos4 = pos4
    else:
        pos4 = int(pos4x)
    packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION, pos4)

