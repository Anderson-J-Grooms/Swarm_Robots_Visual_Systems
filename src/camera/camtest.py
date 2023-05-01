import cv2
import numpy as np
from matplotlib import pyplot as plt
cam = cv2.VideoCapture(0)
result, image = cam.read()
cv2.imwrite("test.png", image)
img = cv2.imread("test.png", cv2.IMREAD_GRAYSCALE)
img = cv2.medianBlur(img,5)
thresh = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)
titles = ['Original Image', 'Adaptive Gaussian Thresholding']
images = [img, thresh]
for i in range(2):
    plt.subplot(2,1,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
#plt.show()
# find contours and get one with area about 180*35
# draw all contours in green and accepted ones in red
contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]
#area_thresh = 0
min_area = 0.95*180*35
max_area = 1.05*180*35
result = img.copy()
for c in contours:
    area = cv2.contourArea(c)
    cv2.drawContours(result, [c], -1, (0, 255, 0), 1)
    if area > min_area and area < max_area:
            cv2.drawContours(result, [c], -1, (0, 0, 255), 1)

# save result
cv2.imwrite("box_found.png", result)

# show images
cv2.imshow("THRESH", thresh)
cv2.imshow("RESULT", result)
cv2.waitKey(0)