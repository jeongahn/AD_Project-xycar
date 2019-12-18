# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time


class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.value_threshhold = 190
        self.image_width = 640
        self.scan_width, self.scan_height = 90, 20
        self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
        self.area_width, self.area_height = 20, 10
        self.roi_vertical_pos = 300
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.pixel_cnt_threshold = 0.15 * self.area_width * self.area_height

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        # self.color = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

        self.up_roi = np.zeros(shape=(300, 640, 3), dtype=np.uint8)

        self.linescan_offset = (self.row_end - self.row_begin) // 2
        self.left, self.right = -1, -1
        self.l_beg = self.scan_width
        self.l_end = 0
        self.r_beg = self.image_width - self.scan_width
        self.r_end = self.image_width - 1

        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.img30 = cv2.imread('/home/nvidia/xycar/src/auto_drive/src/30km.jpg', cv2.IMREAD_GRAYSCALE)
        self.kp30, self.des30 = self.orb.detectAndCompute(self.img30, None)

        self.img40 = cv2.imread('/home/nvidia/xycar/src/auto_drive/src/40km.jpg', cv2.IMREAD_GRAYSCALE)
        self.kp40, self.des40 = self.orb.detectAndCompute(self.img40, None)

        self.img50 = cv2.imread('/home/nvidia/xycar/src/auto_drive/src/50km.jpg', cv2.IMREAD_GRAYSCALE)
        self.kp50, self.des50 = self.orb.detectAndCompute(self.img50, None)

        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

    def find_edge(self, edge, beg, end):
        offset = self.linescan_offset
        r = range(beg, end) if beg < end \
            else range(beg, end - 1, -1)
        for h in r:
            if edge[offset - 1, h] == 255 or \
                    edge[offset, h] == 255 or \
                    edge[offset + 1, h] == 255:
                break
        else:
            h = -1
        # print("h:", h)
        return h

    def check_area(self, mask, col1, col2):
        row1 = (self.scan_height - self.area_height) // 2
        row2 = row1 + self.area_height
        area = mask[row1:row2, col1:col2]
        cv2.imshow('show', mask)
        return cv2.countNonZero(area) > self.pixel_cnt_threshold

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        self.up_roi = self.cam_img[:300, :]
        self.up_roi = cv2.cvtColor(self.up_roi, cv2.COLOR_BGR2GRAY)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)  # roi이미지를 이진화

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 40, 150)  # roi 이미지의 윤곽선 검출

    def detect_lines(self):
        detect_left, detect_right = -1, -1

        if self.left == -1:
            self.l_beg = self.scan_width * 2
            self.l_end = 0
        else:
            self.l_beg = min(self.left + self.scan_width, self.image_width - 1)
            self.l_end = max(self.area_width // 2, self.left - self.scan_width)
        l = self.find_edge(self.edge, self.l_beg, self.l_end)

        if self.right == -1:
            self.r_beg = self.image_width - self.scan_width * 2
            self.r_end = self.image_width - 1
        else:
            self.r_beg = max(self.right - self.scan_width, 0)
            self.r_end = min(self.right + self.scan_width, self.image_width - self.area_width // 2 - 1)
        r = self.find_edge(self.edge, self.r_beg, self.r_end)

        if l != -1 and self.check_area(self.mask, l - self.area_width, l):
            detect_left = l
        if r != -1 and self.check_area(self.mask, r, r + self.area_width):
            detect_right = r

        if detect_left > 0 and detect_right > 0 and detect_right - detect_left < 380:
            if self.left == -1:
                detect_left = -1
            elif self.right == -1:
                detect_right = -1

        if detect_left > 0:
            if self.left == -1:
                self.left = detect_left
            elif abs(detect_left - self.left) < 35:
                self.left = detect_left
        elif self.left < 2 * self.area_width:
            self.left = -1

        if detect_right > 0:
            if self.right == -1:
                self.right = detect_right
            elif abs(detect_right - self.right) < 35:
                self.right = detect_right
        elif self.right < 2 * self.area_width:
            self.right = -1

        return self.left, self.right

    def img_match(self):
        kp, des = self.orb.detectAndCompute(self.up_roi, None)
        if des is None:
            print("des is None")
            return 0, 0, 0
        elif self.des30 is None:
            print("des 30 is None")
            return 0, 0, 0
        matches30 = self.bf.match(des, self.des30)
        matches30 = sorted(matches30, key=lambda x: x.distance)
        matches40 = self.bf.match(des, self.des40)
        matches40 = sorted(matches40, key=lambda x: x.distance)
        matches50 = self.bf.match(des, self.des50)
        matches50 = sorted(matches50, key=lambda x: x.distance)

        dist30 = [m.distance for m in matches30 if m.distance < 38]
        dist40 = [m.distance for m in matches40 if m.distance < 38]
        dist50 = [m.distance for m in matches50 if m.distance < 38]

        return len(dist30), len(dist40), len(dist50)

    def show_images(self, left, right):

        frame = cv2.line(self.cam_img, (self.l_beg, self.roi_vertical_pos + self.scan_height // 2),
                         (self.l_end, self.roi_vertical_pos + self.scan_height // 2),
                         (0, 0, 255), 1)
        frame = cv2.line(frame, (self.r_beg, self.roi_vertical_pos + self.scan_height // 2),
                         (self.r_end, self.roi_vertical_pos + self.scan_height // 2),
                         (0, 0, 255), 1)
        frame = cv2.circle(frame, (self.l_beg, self.roi_vertical_pos + self.scan_height // 2),
                           5, (0, 0, 255))
        frame = cv2.circle(frame, (self.l_end, self.roi_vertical_pos + self.scan_height // 2),
                           5, (0, 0, 255))
        frame = cv2.circle(frame, (self.r_beg, self.roi_vertical_pos + self.scan_height // 2),
                           5, (0, 0, 255))
        frame = cv2.circle(frame, (self.r_end, self.roi_vertical_pos + self.scan_height // 2),
                           5, (0, 0, 255))
        if left != -1:
            frame = cv2.rectangle(frame,
                                  (left - self.area_width, self.roi_vertical_pos),
                                  (left, self.roi_vertical_pos + self.scan_height),
                                  (0, 255, 0), 3)
        else:
            print("Lost left line")
        if right != -1:
            frame = cv2.rectangle(frame,
                                  (right, self.roi_vertical_pos),
                                  (right + self.area_width, self.roi_vertical_pos + self.scan_height),
                                  (0, 255, 0), 3)
        else:
            print("Lost right line")
        frame = cv2.rectangle(frame,
                              ((left + right) // 2 - self.area_width // 2, self.roi_vertical_pos),
                              ((left + right) // 2 + self.area_width // 2, self.roi_vertical_pos + self.scan_height),
                              (255, 0, 0), 3)

        circle_roi = self.cam_img[:380, :]
        gray = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=150, param2=70, minRadius=50, maxRadius=0)
        point = []
        if circles is None:
            pass
        else:
            circles = np.uint(np.around(circles))
            for c in circles[0, :]:
                center = (c[0], c[1])
                radius = c[2]

                self.cam_img = cv2.circle(self.cam_img, center, radius, (255, 0, 0), 2)
                print("point", self.cam_img[center[1]][center[0]])
                point = self.cam_img[center[1]][center[0]]

        cv2.imshow("check", frame)
        cv2.imshow("des30", self.up_roi)
        cv2.imshow("circle", self.cam_img)
        cv2.waitKey(5)
        return point

