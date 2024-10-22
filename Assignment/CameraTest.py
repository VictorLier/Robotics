import cv2

# Open video device
capture1 = cv2.VideoCapture(1)
while True:
    ret, img = capture1.read() # Read an image
    cv2.imshow("ImageWindow", img) # Display the image
    if (cv2.waitKey(2) >= 0): # If the user presses a key, exit while loop
        break
cv2.destroyAllWindows() # Close window
cv2.VideoCapture(1).release() # Release video device