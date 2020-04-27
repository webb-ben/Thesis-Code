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

def home():
    while 1:
        img = get_video()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blur = cv2.bilateralFilter(hsv, 2, 50, 50)
        mask = cv2.inRange(blur, np.array((131,92,31)), np.array((156,248,149)))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.dilate(mask, kernel, iterations=3)
        edges = cv2.Canny(mask, 30, 100)
        cv2.imshow("e",edges)
        cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        #
        # # loop over the contours individually
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
        # loop over the contours

        pts = []
        for c in cnts:
            # approximate the contour
            if cv2.contourArea(c) < 150:
                continue
            print (len(cnts), cv2.contourArea(c))
            box = cv2.minAreaRect(c)
            box = cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            pts.append((np.average(box[:, 0]), np.average(box[:, 1])))

        if len(pts) == 4:
            break
    print (pts)
    return perspective.order_points(np.array(pts))



def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = perspective.order_points(pts)
    (tl, tr, br, bl) = rect

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

# Create a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('HMin','image',0,255,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('HMax','image',0,255,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

cv2.createTrackbar('k-size','image',1,19,nothing)
cv2.createTrackbar('o-size','image',1,19,nothing)
cv2.createTrackbar('c-size','image',1,19,nothing)

# cv2.createTrackbar('h','image',,255,nothing)

# # Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMin', 'image', 0)
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

width = 629
height = 503

rect = perspective.order_points(np.array([(162, 34), (1123, 35), (20, 819), (1247, 824)]))
# r = np.array([(8.0, 906.0), (109.0, 22.0), (1163, 36), (1255, 899)])
# print(r)
# h = home()
(tl, tr, br, bl) = rect
widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
maxWidth = max(int(widthA), int(widthB))

# compute the height of the new image, which will be the
# maximum distance between the top-right and bottom-right
# y-coordinates or the top-left and bottom-left y-coordinates
heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
maxHeight = max(int(heightA), int(heightB))
# now that we have the dimensions of the new image, construct
# the set of destination points to obtain a "birds eye view",
# (i.e. top-down view) of the image, again specifying points
# in the top-left, top-right, bottom-right, and bottom-left
# order
maxHeight = int(maxWidth * 0.8)
dst = np.array([
    [0, 0],
    [maxWidth - 1, 0],
    [maxWidth - 1, maxHeight - 1],
    [0, maxHeight - 1]], dtype="float32")
# compute the perspective transform matrix and then apply it
M = cv2.getPerspectiveTransform(rect, dst)
# Initialize to check if HSV min/max value changes

hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0
img = get_video()

output = img
waitTime = 33
while(1):
    img = get_video()
    # img = cv2.warpPerspective(img, M, (maxWidth, maxHeight))
    # img = cv2.rotate(img, cv2.ROTATE_180)

    # get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin','image')
    sMin = cv2.getTrackbarPos('SMin','image')
    vMin = cv2.getTrackbarPos('VMin','image')

    hMax = cv2.getTrackbarPos('HMax','image')
    sMax = cv2.getTrackbarPos('SMax','image')
    vMax = cv2.getTrackbarPos('VMax','image')

    k = cv2.getTrackbarPos('k-size', 'image')
    o = cv2.getTrackbarPos('o-size', 'image')
    c = cv2.getTrackbarPos('c-size', 'image')

    # Set minimum and max HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])
    # Create HSV Image and threshold into a range.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blur = cv2.bilateralFilter(hsv, 2, 50, 50)
    mask = cv2.inRange(blur, lower, upper)
    # mask += cv2.inRange(blur, np.array((150,99,84)), np.array((180,255,255)))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=c)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=o)

    # mask = cv2.dilate(mask, kernel, iterations = 3)
    output = cv2.bitwise_and(img, img, mask=mask)

    # Find edges
    edges = cv2.Canny(mask, 30, 100)
    # edges = cv2.dilate(edges, kernel, iterations=2)
    # edges = cv2.erode(edges, 1 - kernel, iterations=1)
    # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, 10, 0)
    # if lines:
    #     for x1, y1, x2, y2 in lines[0]:
    #         cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # edges = cv2.dilate(edges, kernel, iterations=2)
    # edges = cv2.erode(edges, 1 - kernel, iterations=1)

    cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    #
    # # loop over the contours individually
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
    # loop over the contours
    for c in cnts:
        # approximate the contour
        if cv2.contourArea(c) < 10000:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        # if our approximated contour has four points, then we
        # can assume that we have found our screen

    # Print if there is a change in HSV value
    if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax
    # Display output image
    # cv2.imshow('image',output)
    # cv2.imshow('orig', img)
    cv2.imshow('edges', edges)
    # print ([f.shape for f in (img, hsv, blur, output, edges)])
    cv2.imshow('image', imutils.resize(np.hstack(((img, output, hsv, blur))),width=2560))
    # cv2.imshow('hsv', imutils.resize(np.hstack((hsv, blur)), height=600))

    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(waitTime) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

# # Create a window
#
# cv2.namedWindow('image')
#
# # create trackbars for color change
# cv2.createTrackbar('Low','image',0,255,nothing) # Hue is from 0-179 for Opencv
# cv2.createTrackbar('High','image',0,255,nothing)
#
# cv2.setTrackbarPos('Low', 'image', 30)
# cv2.setTrackbarPos('High', 'image', 100)
#
# # Initialize to check if HSV min/max value changes
# Low = High = pLow = pHigh = 0
# img = get_video()
#
# output = img
# waitTime = 33
#
# while(1):
#     img = get_video()
#
#     # get current positions of all trackbars
#     Low = cv2.getTrackbarPos('Low','image')
#     High = cv2.getTrackbarPos('High','image')
#
#     # Create HSV Image and threshold into a range.
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#
#     # mask = cv2.inRange(hsv, np.array([127, 67, 142]), np.array([179, 255, 255]))
#     mask = cv2.inRange(hsv, np.array([37, 112, 0]), np.array([179, 255, 201]))
#     mask += cv2.inRange(hsv, np.array([47, 133, 81]), np.array([179, 255, 148]))
#
#     mask = cv2.dilate(mask, None, iterations=3)
#     blur = cv2.GaussianBlur(mask, (3, 3), 0)
#
#     edges = cv2.Canny(blur, 30, 100)
#     lines = cv2.HoughLines(edges, Low, np.pi / 180, High)
#     for rho, theta in lines[0]:
#         a = np.cos(theta)
#         b = np.sin(theta)
#         x0 = a * rho
#         y0 = b * rho
#         x1 = int(x0 + 1000 * (-b))
#         y1 = int(y0 + 1000 * (a))
#         x2 = int(x0 - 1000 * (-b))
#         y2 = int(y0 - 1000 * (a))
#     edges = cv2.dilate(edges, None, iterations=1)
#
#     output = cv2.bitwise_and(img,img, mask= edges)
#     cv2.line(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
#
#     # Print if there is a change in HSV value
#     if( (pLow != Low) | (pHigh != High)  ):
#         print("(Low = %d , High = %d)" % (Low, High))
#         pLow = Low
#         pHigh = High
#
#     # Display output image
#     cv2.imshow('image',output)
#
#     # Wait longer to prevent freeze for videos.
#     if cv2.waitKey(waitTime) & 0xFF == ord('q'):
#         break
#
# cv2.destroyAllWindows()