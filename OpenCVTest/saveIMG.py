import cv2 as cv


capture1 = cv.VideoCapture(1, cv.CAP_DSHOW)
ret, image = capture1.read()
cv.VideoCapture(0).release()

save = cv.imwrite("test.jpg", image)