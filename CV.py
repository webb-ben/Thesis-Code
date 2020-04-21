# import the necessary modules
import freenect
import cv2
import numpy as np
from imutils import perspective
from imutils import contours
import imutils
from scipy.spatial import ConvexHull


# function to get RGB image from kinect
def get_video():
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


# function to get depth image from kinect
def get_depth():
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

# loop runs if capturing has been initialized
while (1):
    # reads frames from a camera
    frame = get_video()
    # converting BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of red color in HSV
    lower_red = np.array([30, 150, 50])
    upper_red = np.array([255, 255, 180])

    # create a red HSV colour boundary and
    # threshold HSV image
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    gray = cv2.GaussianBlur(hsv, (7, 7), 0)

    # finds edges in the input image image and
    # marks them in the output map edges
    edged = cv2.Canny(gray, 30, 100)
    # edged = cv2.Canny(hsv, 100, 200)
    edged = cv2.dilate(edged, None, iterations=2)
    edged = cv2.erode(edged, None, iterations=3)

    # cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
    #                         cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    #
    # # loop over the contours individually
    # for c in cnts:
    #     # if the contour is not sufficiently large, ignore it
    #     if cv2.contourArea(c) < 100:
    #         continue
    #     # compute the rotated bounding box of the contour
    #     box = cv2.minAreaRect(c)
    #     box = cv2.boxPoints(box)
    #     box = np.array(box, dtype="int")
    #     # order the points in the contour such that they appear
    #     # in top-left, top-right, bottom-right, and bottom-left
    #     # order, then draw the outline of the rotated bounding
    #     # box
    #     box = perspective.order_points(box)
    #     # compute the center of the bounding box
    #     cX = np.average(box[:, 0])
    #     cY = np.average(box[:, 1])
    #
    #     # orig = frame.copy()
    #     cv2.drawContours(frame, [c.astype("int")], -1, (0, 255, 0), 2)

    # Display edges in a frame
    cv2.imshow('Edges', edged)

    # Display an original image
    cv2.imshow('Original', frame)

    # Wait for Esc key to stop
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

# De-allocate any associated memory usage
cv2.destroyAllWindows()