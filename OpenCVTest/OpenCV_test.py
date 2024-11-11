import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt




# import image
capture1 = cv.VideoCapture(1, cv.CAP_DSHOW)
ret, image = capture1.read()
cv.VideoCapture(0).release()

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
point_sort = sorted(points, key=lambda point: point[0]**2 + point[1]**2)


print("Stop")

cv.imshow('image', image)
cv.imshow('mask', mask)
for i in range(len(points)):
    cv.circle(image,(points[i,0], points[i,1]),10,(50*i,50*i,50*i), -1)
cv.imshow("Circles", image)

cv.drawContours(image_wh, contourse, -1, (255, 255, 255), 3)
cv.imshow("Contour", image_wh)
cv.waitKey(0)