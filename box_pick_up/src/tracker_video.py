import cv2
import numpy as np


# text with coordinates of the center
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (0, 0, 0)
thickness = 2

video = cv2.VideoCapture(0)

kernel = np.ones((5, 5), np.uint8)

# color range - green (HSV)
lower_green = np.array([36, 0, 0])
upper_green = np.array([86, 255, 255])

while True:

    ret, img = video.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    result = cv2.bitwise_and(img, img, mask=mask_green)

    opening = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
    cnts, hei = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        area = cv2.contourArea(c)
        if area > 500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            x, y, w, h = cv2.boundingRect(c)

            img = cv2.putText(img, str(x + w // 2) + " " + str(y + h // 2), (x + w // 2, y + h // 2), font,
                              fontScale, color, thickness, cv2.LINE_AA)

    cv2.imshow("mask_green", mask_green)
    cv2.imshow("final", img)

    k = cv2.waitKey(1)
    if k == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
