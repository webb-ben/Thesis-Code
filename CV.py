# import the necessary modules
import freenect
import cv2
import numpy as np
import imutils


def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
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

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([127, 67, 142]), np.array([179, 255, 255]))
    mask += cv2.inRange(hsv, np.array([37, 112, 0]), np.array([179, 255, 201]))

    mask = cv2.dilate(mask, None, iterations =2)
    blur = cv2.GaussianBlur(mask, (7,7), 0)

    edges = cv2.Canny(blur, 30, 100)
    edges = cv2.dilate(edges, None, iterations=1)
    edges = cv2.erode(edges, None, iterations=1)

    cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    #
    # # loop over the contours individually
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
    # loop over the contours
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        # if our approximated contour has four points, then we
        # can assume that we have found our screen
        if len(approx) == 4:
            screenCnt = approx
            break

    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    cv2.drawContours(frame, [screenCnt], -1, (0, 255, 0), 2)
    warped = four_point_transform(frame, screenCnt.reshape(4, 2))
    # cv2.drawContours(frame, [c.astype("int")], -1, (0, 255, 0), 2)
    # cv2.circle(frame, extLeft, 8, (0, 0, 255), -1)
    # cv2.circle(frame, extRight, 8, (0, 255, 0), -1)
    # cv2.circle(frame, extTop, 8, (255, 0, 0), -1)
    # cv2.circle(frame, extBot, 8, (255, 255, 0), -1)
        # warped = four_point_transform(frame, c)
    # Display edges in a frame
    cv2.imshow('Edges', edges)

    # Display an original image
    cv2.imshow("Original", frame)
    cv2.imshow("Scanned", imutils.resize(warped, height=500, width = 500))


    # cv2.imshow('Warped', warped)

    # Wait for Esc key to stop
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

# De-allocate any associated memory usage
cv2.destroyAllWindows()
