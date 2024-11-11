import numpy as np


# We are very much assuming that the camera is pointing straight down. Therefore only the x and y coordinates are needed.
# The center of the area is assumed to be the same x,y coordinates as the center of the image.

theta = np.array([0, 0, -80, -100]) * np.pi/180

# The transformation matrix
cameraPos = [
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



print("Stop")