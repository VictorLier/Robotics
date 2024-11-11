import dynamixel_sdk as dxl
import sympy as sp
import cv2 as cv
import numpy as np
import time
from sympy import sin, cos, pi

class Camera:
    '''
    Camera class for the robot arm camera
    '''
    def __init__(self) -> None:
        pass

    def take_image(self, plot:bool = False) -> None:
        '''
        Take an image from the camera
        '''
        # import image
        capture1 = cv.VideoCapture(1, cv.CAP_DSHOW)
        ret, image = capture1.read()
        cv.VideoCapture(0).release()

        # find image size
        self.image_size = image.shape

        # Convert image to HSV
        image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # White image with same size as image
        image_wh = np.zeros(image.shape, dtype=np.uint8)

        # mask
        l_b = np.array([0, 123, 75])
        u_b = np.array([19, 255, 153])

        mask = cv.inRange(image_hsv, l_b, u_b)

        # Convert mask to binary
        _, mask = cv.threshold(mask, 127, 255, cv.THRESH_BINARY)

        # inverse mask
        mask_inv = cv.bitwise_not(mask)

        # Contour
        cnts, hierchy = cv.findContours(mask_inv, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Sort contours by area
        cnt = sorted(cnts, key=cv.contourArea, reverse=True)

        # Remove outer edge
        cnt = cnt[1:]

        # Find the max 5% of contour areas
        max_area = cv.contourArea(cnt[0])
        contourse = []
        for c in cnt:
            if cv.contourArea(c) > 0.05 * max_area:
                contourse.append(c)
            else:
                break


        # Find the center of the contour
        cx = []
        cy = []
        for c in contourse:
            M = cv.moments(c)
            cx.append(int(M['m10']/M['m00']))
            cy.append(int(M['m01']/M['m00']))

        points = np.array([cx, cy]).T

        # Sort points by distance from with start point 0,0
        self.point_sort = sorted(points, key=lambda point: point[0]**2 + point[1]**2)


        if plot:
            cv.imshow('image', image)
            cv.imshow('mask', mask)
            for i in range(len(points)):
                cv.circle(image,(points[i,0], points[i,1]),10,(50*i,50*i,50*i), -1)
            cv.imshow("Circles", image)

            cv.drawContours(image_wh, contourse, -1, (255, 255, 255), 3)
            cv.imshow("Contour", image_wh)
            cv.waitKey(0)


    def convert_points(self) -> None:
        '''
        Convert the points to a numpy array
        '''
        #  Get the points from center
        self.point_sort = np.array(self.point_sort)
        self.point_sort[:,0] = self.point_sort[:,0] - self.image_size[1]/2
        self.point_sort[:,1] = self.point_sort[:,1] - self.image_size[0]/2

        # Invert to get same coordinate system as the robot
        self.point_sort = np.flip(self.point_sort, axis=1)

        # Flip x axis
        self.point_sort[:,0] = -self.point_sort[:,0]

        # Convert to meters
        pixelwidth = 0.12/1116 # The size of a pixel in meters taken from the camera calibration position
        self.points_meters = self.point_sort * pixelwidth


    def camera_position(self):
        # We are very much assuming that the camera is pointing straight down. Therefore only the x and y coordinates are needed.
        # The center of the area is assumed to be the same x,y coordinates as the center of the image.

        theta = np.array([0, 0, -80, -100]) * np.pi/180

        # The transformation matrix
        self.cameraPos = [
            0.065 * (np.sin(theta[1]) * np.sin(theta[2]) * np.cos(theta[0]) - np.cos(theta[0]) * np.cos(theta[1]) * np.cos(theta[2])) * np.sin(theta[3]) + 
            0.065 * (-np.sin(theta[1]) * np.cos(theta[0]) * np.cos(theta[2]) - np.sin(theta[2]) * np.cos(theta[0]) * np.cos(theta[1])) * np.cos(theta[3]) - 
            0.093 * np.sin(theta[1]) * np.cos(theta[0]) * np.cos(theta[2]) - 0.093 * np.sin(theta[1]) * np.cos(theta[0]) - 
            0.093 * np.sin(theta[2]) * np.cos(theta[0]) * np.cos(theta[1]), 

            0.065 * (np.sin(theta[0]) * np.sin(theta[1]) * np.sin(theta[2]) - np.sin(theta[0]) * np.cos(theta[1]) * np.cos(theta[2])) * np.sin(theta[3]) + 
            0.065 * (-np.sin(theta[0]) * np.sin(theta[1]) * np.cos(theta[2]) - np.sin(theta[0]) * np.sin(theta[2]) * np.cos(theta[1])) * np.cos(theta[3]) - 
            0.093 * np.sin(theta[0]) * np.sin(theta[1]) * np.cos(theta[2]) - 0.093 * np.sin(theta[0]) * np.sin(theta[1]) - 
            0.093 * np.sin(theta[0]) * np.sin(theta[2]) * np.cos(theta[1]), 

            0.065 * (-np.sin(theta[1]) * np.sin(theta[2]) + np.cos(theta[1]) * np.cos(theta[2])) * np.cos(theta[3]) + 
            0.065 * (-np.sin(theta[1]) * np.cos(theta[2]) - np.sin(theta[2]) * np.cos(theta[1])) * np.sin(theta[3]) - 
            0.093 * np.sin(theta[1]) * np.sin(theta[2]) + 0.093 * np.cos(theta[1]) * np.cos(theta[2]) + 
            0.093 * np.cos(theta[1]) + 0.05
        ]


    def points_in_global(self):
        '''
        Convert the points to global coordinates
        '''
        self.points_global = self.points_meters
        self.points_global[:,0] = self.points_meters[:,0] + self.cameraPos[0]
        self.points_global[:,1] = self.points_meters[:,1] + self.cameraPos[1]
        zvalue = -50e-3 # The height of the camera from the ground
        self.points_global = np.insert(self.points_global, 2, zvalue, axis=1)
        

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
    theta_3, d_3, a_3, alpha_3 = theta3 + pi/2, 0, 0, 0

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

    pointer = [80e-3, 20e-3, 0]
    camera = [35e-3, 45e-3, 0]

    T= T_arr[4] * Translation(*pointer)

    x_04 = T[0:3,0]
    o_04 = T[0:3,3]
    eq1 = sp.Eq(o_04[0], pos[0])
    eq2 = sp.Eq(o_04[1], pos[1])
    eq3 = sp.Eq(o_04[2], pos[2])
    eq4 = sp.Eq(x_04[2], 0)
    
    # Solve numerically
    solution = sp.nsolve((eq1, eq2, eq3, eq4), [theta0, theta1, theta2, theta3], initial_guess, tol=1e-4)
    #convert to degrees
    solution = [s for s in solution]
    
    return solution



if __name__ == "__main__":
    camera = Camera()
    camera.take_image(plot=False)
    camera.convert_points()
    camera.camera_position()
    camera.points_in_global()

    points = camera.points_global

    #  Int motor
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
    DEVICENAME = "COM8"
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
    initial_guess = [0.1, 3/8, 1, 1.5]

    num_points = len(points)

    angles = np.zeros((num_points, 4))

    for i in range(num_points):
            angle = get_angles(points[i], initial_guess)
            angle = np.array(angle, dtype=np.float64)
            angle = (angle + np.pi/2) % (np.pi) - np.pi/2

            angle[0] = (angle[0] + np.pi/2) % np.pi - np.pi/2  # Angle 1 in range [-pi/2, pi/2]
            angle[1] = -abs((angle[1] + np.pi) % (2 * np.pi) - np.pi)  # Angle 2 in range [-pi, pi]
            angle[2] = -abs((angle[2] + np.pi) % (2 * np.pi) - np.pi)  # Angle 3 in range [-pi, pi]
            angle[3] = -abs((angle[3] + np.pi/2) % np.pi - np.pi/2)  # Angle 4 in range [-pi/2, pi/2]

            angles[i] = angle



    steps_per_degree = 1023/300

  
    for i in range(num_points):

        step = angles[i] * 180/np.pi  * steps_per_degree
        step[0] = step[0] + 512
        step[1] = step[1] + 512
        step[2] = step[2] + 512 + steps_per_degree*90
        step[3] = step[3] + 512

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

        time.sleep(3)