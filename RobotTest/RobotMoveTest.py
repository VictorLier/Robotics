import dynamixel_sdk as dxl
import numpy as np
import sympy as sp
from sympy import sin, cos, pi
import time

def get_angles(pos, initial_guess) -> np.ndarray:
    """
    Calculate the angles from the position of the end effector and the initial guess
    """
    theta0, theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta0 theta1 theta2 theta3 theta4 theta5')
    d0, d1, d2, d3, d4, d5 = sp.symbols('d0 d1 d2 d3 d4 d5')
    a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = sp.symbols('alpha0 alpha1 alpha2 alpha3 alpha4 alpha5')

    theta_0, d_0, a_0, alpha_0 = theta0, 50e-3, 0, pi/2
    theta_1, d_1, a_1, alpha_1 = theta1 + pi/2, 0, 93e-3, 0
    theta_2, d_2, a_2, alpha_2 = theta2, 0, 93e-3, 0
    theta_3, d_3, a_3, alpha_3 = theta3, 0, 0, 0

    theta = [theta_0, theta_1, theta_2, theta_3]
    d = [d_0, d_1, d_2, d_3]
    a = [a_0, a_1, a_2, a_3]
    alpha = [alpha_0, alpha_1, alpha_2, alpha_3]

    T = lambda theta, d, a, alpha: sp.Matrix([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                                          [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                                          [0, sin(alpha), cos(alpha), d],
                                          [0, 0, 0, 1]])
    Translation = lambda x, y, z: sp.Matrix([[1, 0, 0, x],
                                         [0, 1, 0, y],
                                         [0, 0, 1, z],
                                         [0, 0, 0, 1]])
    A = sp.Identity(4)
    T_arr = [A]
    for (th, di, ai, al) in zip(theta, d, a, alpha):
        T_arr.append(T_arr[-1]*T(th, di, ai, al))

    pointer = [50e-3, 0, 0]
    camera = [35e-3, 45e-3, 0]

    T= T_arr[4] * Translation(*pointer)

    x_04 = T[0:3,0]
    o_04 = T[0:3,3]
    eq1 = sp.Eq(o_04[0], pos[0])
    eq2 = sp.Eq(o_04[1], pos[1])
    eq3 = sp.Eq(o_04[2], pos[2])
    eq4 = sp.Eq(x_04[2], 0)
    
    # Solve numerically
    solution = sp.nsolve((eq1, eq2, eq3, eq4), [theta0, theta1, theta2, theta3], initial_guess, tol=1e-6)
    #convert to degrees
    solution = [s for s in solution]
    
    return solution



if __name__ == "__main__":

    # Circle points
    circle_center_x = 150e-3
    circle_center_y = 0e-3
    circle_center_z = 120e-3

    radius = 32e-3

    num_points = 36

    angles = np.linspace(0, 2*np.pi, num_points)

    x_coord = circle_center_x + radius*np.cos(angles)
    y_coord = circle_center_y + radius*np.sin(angles)
    z_coord = circle_center_z*np.ones(num_points)

    points = np.vstack((x_coord, y_coord, z_coord)).T


# Int motor
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
    DEVICENAME = "COM5"
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
        packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 50)










    # Initial guess
    initial_guess = [1, 10, 10, 1]

    angles = np.zeros((num_points, 4))

    for i in range(num_points):
            angle = get_angles(points[i], initial_guess)
            angle = np.array(angle, dtype=np.float64)
            angles[i] = angle

    steps_per_degree = 1023/300

  
    for i in range(num_points):

        step = angles[i] * steps_per_degree
        step[0] = step[0] + 512
        step[1] = step[1] + 205
        step[2] = step[2] + 512
        step[3] = step[3] + 830


        step = step.astype(int)
    
        print(step)

        # Move motor 1
        # Default values
        pos1 = step[0]
        pos2 = step[1]
        pos3 = step[2]
        pos4 = step[3]

        packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, pos1)
        packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, pos2)
        packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION, pos3)
        packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION, pos4)

        time.sleep(1)

    


    






    print("Stop")