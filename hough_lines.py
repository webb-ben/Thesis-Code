import cv2
import sys
import numpy as np
import freenect
import frame_convert2
import imutils
from imutils import perspective

def nothing(x):
    pass

def get_video():
    f = frame_convert2.video_cv(freenect.sync_get_video()[0])
    return cv2.fastNlMeansDenoisingColored(f, None, 10, 10, 3, 3)

img = img1 = get_video()
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
blur = cv2.bilateralFilter(hsv, 2, 50, 50)
mask = cv2.inRange(blur, 0, 0)
mask += cv2.inRange(blur, np.array((140,255,104)), np.array((180,255,244)))
mask += cv2.inRange(blur, np.array((0,165,96)), np.array((50,255,255)))
mask += cv2.inRange(blur, np.array((161, 128, 76)), np.array((179, 255, 255)))

kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

edges = cv2.Canny(mask,0,0)
edges = cv2.dilate(edges, None, iterations=1)
edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
# edges = cv2.erode(edges, None, iterations=2)


# Create a window
cv2.namedWindow('image')
cv2.createTrackbar('r','image',1,100,nothing)
cv2.createTrackbar('num_i','image',1,255,nothing)
cv2.createTrackbar('line_len','image',1,255,nothing)
cv2.createTrackbar('line_gap','image',1,255,nothing)

cv2.setTrackbarPos('r', 'image', 1)
cv2.setTrackbarPos('num_i', 'image', 100)
cv2.setTrackbarPos('line_len', 'image', 10)
cv2.setTrackbarPos('line_gap', 'image', 50)

l = []
while(1):
    img = get_video()
    line_len = cv2.getTrackbarPos('line_len', 'image')
    line_gap = cv2.getTrackbarPos('line_gap', 'image')
    r = cv2.getTrackbarPos('r', 'image')
    num_i = cv2.getTrackbarPos('num_i', 'image')

    try:
        lines = cv2.HoughLinesP(edges,r,np.pi/180, num_i, line_len, line_gap)

        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
    except cv2.error:
        continue

    # cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL,
    #                         cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    # #
    # # # loop over the contours individually
    # cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
    # # loop over the contours
    # for c in cnts:
    #     # approximate the contour
    #     peri = cv2.arcLength(c, True)
    #     approx = cv2.approxPolyDP(c, peri, False)
    #     cv2.drawContours(img, [approx], -1, (0, 0, 255), 1)

    # print (len(lines))
    cv2.imshow('image',img)
    cv2.imshow('e',edges)

    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()