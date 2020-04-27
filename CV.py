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
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))

        # rect = perspective.order_points(np.array([(100, 24), (555, 21), (9, 382), (633 ,387)]))
        rect = perspective.order_points(np.array([(162, 34), (1123, 35), (20, 819), (1247, 824)]))
        (tl, tr, br, bl) = rect
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        self.width = max(int(widthA), int(widthB))
        self.height = int(self.width * 0.8)

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
        self.line= None
        self.line_edges = None
        self.line_mask = None
        self.pos=[]

    def update_frame(self):
        self.frame = get_video()
        # self.depth = get_depth()
        self.uw = False
        self.uf = True

    def update_warped(self):
        if not self.uf: self.update_frame()
        if self.uw: return
        self.warped_frame = cv2.warpPerspective(self.frame, self.M, (self.width, self.height))
        self.warped_frame = cv2.rotate(self.warped_frame, cv2.ROTATE_180)

        # self.warped_depth = cv2.warpPerspective(self.frame, self.M, (629, 500))
        self.uw = True
        if len(self.box_cnt) == 4:
            # cv2.drawContours(self.warped_frame, [self.box_cnt], -1, (255, 0,0), 2)
            pass

    def update_box(self):
        self.update_warped()
        hsv = cv2.cvtColor(self.warped_frame, cv2.COLOR_BGR2HSV)
        blur = cv2.bilateralFilter(hsv, 2, 50, 50)
        mask = cv2.inRange(blur, np.array((68, 23, 41)), np.array((109, 255, 255))) # Cyan
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        edges = cv2.Canny(mask, 30, 100)
        cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        #
        # # loop over the contours individually
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
        # loop over the contours
        for c in cnts:
            # approximate the contour
            if cv2.contourArea(c) < 60000:
                continue
            peri = cv2.arcLength(c, False)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            # if our approximated contour has four points, then we
            # can assume that we have found our screen
            if len(approx) == 4:
                self.box_cnt = approx

            self.box = perspective.four_point_transform(self.warped_frame, self.box_cnt.reshape(4, 2))
            self.box_edges = edges
            break

    def update_line(self):
        self.update_warped()
        hsv = cv2.cvtColor(self.warped_frame, cv2.COLOR_BGR2HSV)
        blur = cv2.bilateralFilter(hsv, 2, 50, 50)
        mask = cv2.inRange(blur, np.array((147, 99, 69)), np.array((180, 255, 255))) # Red1
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)

        if self.line_mask is not None:
            m = cv2.bitwise_xor(mask, self.line_mask)
            not_m = cv2.bitwise_not(cv2.dilate(self.line_mask,None,iterations=2))
            m = cv2.bitwise_and(m,m,mask=not_m)
            edges = cv2.Canny(m, 0, 0)
        else:
            edges = cv2.Canny(mask, 0, 0)
        cv2.imshow('l', edges)
        self.line_mask = mask
        self.line_edges = edges
        cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)
        cnts = imutils.grab_contours(cnts)
        for c in cnts:
            if cv2.contourArea(c) < 5 or cv2.contourArea(c) > 100:
                    continue
            box = cv2.minAreaRect(c)
            box = cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            # compute the center of the bounding box
            self.pos = (np.average(box[:, 0]), np.average(box[:, 1]))
            self.distance_between()
            break

    def distance_between(self):
        cnts = cv2.findContours(self.box_edges.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)
        cnts = imutils.grab_contours(cnts)
        for c in cnts:
            peri = cv2.arcLength(c, False)
            if peri < 300:
                continue
            cx = cy = 20
            cd = np.inf
            for i in range(c.shape[0]):
                d = np.sqrt((c[i, 0, 0]-self.pos[0])**2+(c[i, 0, 1]-self.pos[1])**2)
                if cd > d:
                    cx, cy, cd = c[i, 0, 0], c[i, 0, 1], d
            cv2.line(self.warped_frame, (int(cx), int(cy)), self.pos, (255,255,255), 2)
            print(cd)
            return


    # loop runs if capturing has been initialized
    def run(self):
        i = 0

        while 1:
            self.update_box()

            self.update_line()

            # Display an original image
            if self.frame is not None:
                cv2.imshow("edges", self.line_edges)
                # cv2.imshow("Original", self.frame)
                pass

            # cv2.imshow('Edges', edges)
            if self.warped_frame is not None:
                cv2.imshow("Warped Frame", self.warped_frame)
                pass

            if self.depth is not None:
                # cv2.imshow("Depth", self.depth)
                pass

            if self.depth is not None:
                # cv2.imshow("Warped Depth", self.depth)
                pass

            if self.box is not None:
                cv2.imshow("Box", self.box)
                pass

            # Wait for Esc key to stop
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
            self.uf = False
            i += 1

        # De-allocate any associated memory usage
        cv2.destroyAllWindows()

if __name__ == "__main__":
    cv = CV()
    cv.run()