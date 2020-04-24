# import the necessary modules
import freenect
import cv2
import numpy as np
import imutils
from imutils import perspective
import frame_convert2

def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])
    return cv2.fastNlMeansDenoising(f, None, 10, 7, 21)

def get_video():
    f = frame_convert2.video_cv(freenect.sync_get_video()[0])
    return cv2.fastNlMeansDenoisingColored(f, None, 10, 10, 3, 3)

class CV(object):
    def __init__(self):
        self.width = 629
        self.height = 503
        # rect = perspective.order_points(np.array([(100, 24), (555, 21), (9, 382), (633 ,387)]))
        rect = perspective.order_points(np.array([(201, 38), (1107, 30), (20, 756), (1273, 766)]))

        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [self.width - 1, 0],
            [self.width - 1, self.height - 1],
            [0, self.height - 1]], dtype="float32")

        # compute the perspective transform matrix and then apply it
        self.M = cv2.getPerspectiveTransform(rect, dst)
        self.uw = False
        self.uf = False
        self.box_cnt = ()
        self.box = None
        self.frame = None
        self.warped_frame = None
        self.depth = None
        self.warped_depth = None

    def update_frame(self):
        self.frame = get_video()
        # self.depth = get_depth()
        self.uw = False
        self.uf = True

    def update_warped(self):
        if not self.uf: self.update_frame()
        if self.uw: return
        self.warped_frame = cv2.warpPerspective(self.frame, self.M, (629, 500))
        self.warped_frame = cv2.rotate(self.warped_frame, cv2.ROTATE_180)

        # self.warped_depth = cv2.warpPerspective(self.frame, self.M, (629, 500))
        self.uw = True
        if len(self.box_cnt) == 4:
            pass
            cv2.drawContours(self.warped_frame, [self.box_cnt], -1, (0, 0, 255), 1)

    def update_box(self):
        self.update_warped()
        hsv = cv2.cvtColor(self.warped_frame, cv2.COLOR_BGR2HSV)
        blur = cv2.bilateralFilter(hsv, 2, 50, 50)
        mask = cv2.inRange(blur, np.array((0, 0, 0)), np.array((180, 255, 255)))
        mask += cv2.inRange(blur, np.array((140, 255, 104)), np.array((180, 255, 244)))
        mask += cv2.inRange(blur, np.array((136, 61, 95)), np.array((179, 255, 176)))
        mask += cv2.inRange(blur, np.array((0, 165, 96)), np.array((104, 255, 255)))
        mask += cv2.inRange(blur, np.array((161, 128, 76)), np.array((179, 255, 255)))

        self.box_edges = cv2.Canny(mask, 0, 0)

        cnts = cv2.findContours(self.box_edges.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        #
        # # loop over the contours individually
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
        # loop over the contours
        for c in cnts:
            # approximate the contour
            if 17000 < cv2.contourArea(c) < 15000:
                continue

            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            # if our approximated contour has four points, then we
            # can assume that we have found our screen
            # cv2.drawContours(self.warped_frame, [approx], -1, (0, 0, 0), 2)
            if len(approx) == 4:
                self.box_cnt = approx

            self.box = perspective.four_point_transform(self.warped_frame, self.box_cnt.reshape(4, 2))
            break

    def update_line(self):
        self.update_warped()
        hsv = cv2.cvtColor(self.warped_frame, cv2.COLOR_BGR2HSV)
        blur = cv2.bilateralFilter(hsv, 2, 50, 50)
        mask = cv2.inRange(blur, np.array((0, 0, 0)), np.array((180, 255, 255)))
        mask += cv2.inRange(blur, np.array((140, 255, 104)), np.array((180, 255, 244)))
        mask += cv2.inRange(blur, np.array((136, 61, 95)), np.array((179, 255, 176)))
        mask += cv2.inRange(blur, np.array((0, 165, 96)), np.array((104, 255, 255)))
        mask += cv2.inRange(blur, np.array((161, 128, 76)), np.array((179, 255, 255)))

        self.box_edges = cv2.Canny(mask, 0, 0)

    # loop runs if capturing has been initialized
    def run(self):
        cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
        while (1):
            self.update_line()

            # Display an original image
            if self.frame is not None:
                cv2.imshow("Original", self.frame)
                cv2.imshow("edges", self.box_edges)
                pass

            # cv2.imshow('Edges', edges)
            if self.warped_frame is not None:
                cv2.imshow("Warped Frame", self.warped_frame)
                pass

            if self.depth is not None:
                cv2.imshow("Depth", self.depth)
                pass

            if self.depth is not None:
                cv2.imshow("Warped Depth", self.depth)
                pass

            if self.box is not None:
                cv2.imshow("Box", self.box)
                pass


            # Wait for Esc key to stop
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
            self.uf = False

        # De-allocate any associated memory usage
        cv2.destroyAllWindows()

if __name__ == "__main__":
    cv = CV()
    cv.run()