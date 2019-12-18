#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time

from linedetector_copy import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
import cv2


class AutoDrive:

    def __init__(self):

        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.left = 0
        self.right = 640
        self.angle = 0

    # 장애물, 차선 인식하여 조향각, 속도 결정, 모터 제어 메시지 발생
    def trace(self):
        if self.line_detector.parkingMatch() == True:
            self.parking()
        else:

            obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
            left, right = self.line_detector.detect_lines()
            self.line_detector.show_images(left, right)  # 디버깅위한 화면 표시
            angle = self.steer(left, right)
            speed = self.accelerate(angle, obs_l, obs_m, obs_r)  # 조향각과 속도 인자 결정
            self.driver.drive(angle + 90, speed + 90)  # 모터 제어를 위한 메서드 호출

    # 왼쪽/오른쪽 차선 위치에 따라 조향각 결정
    # 가운데 : 0, 오른쪽 : 양수, 왼쪽 : 음수
    def steer(self, left, right):

        angle = 0

        mid = (left + right) // 2

        angle = int((mid - 320) * 0.5)

        if angle >= 30:
            angle = 50

        elif angle <= -30:
            angle = -50

        return angle

    # 조향각, 장애물 거리에 따라 속도 결정 &
    def parking(self):
        parking_0, parking_3, parking_4, parking_6 = self.obstacle_detector.for_parking_distance()
        while parking_6 <= 50:
            for go1 in range(5):
                drive(85, 110)
                if parking_6 >= 50:
                    time.sleep(1)
                    break
                time.sleep(0.1)

        for stop1 in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(65, 110)
            time.sleep(0.1)

        for left1 in range(50):
            drive(65, 110)
            if data_0 <= 5:
                time.sleep(1)
                break
            time.sleep(0.1)

        for stop2 in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(180, 70)
            time.sleep(0.1)

        for back1 in range(30):
            drive(180, 70)
            if data_3 <= 5:
                time.sleep(1)
                break
            time.sleep(0.1)

        for stop3 in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(70, 110)
            time.sleep(0.1)

        for go2 in range(20):
            drive(70, 110)
            if data_0 <= 5:
                time.sleep(1)
                break
            time.sleep(0.1)

        for stop4 in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(140, 70)
            time.sleep(0.1)

        for back2 in range(5):
            drive(140, 70)
            time.sleep(0.1)

        for stop5 in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(90, 110)
            time.sleep(0.1)

        for go3 in range(5):
            drive(90, 110)
            time.sleep(0.1)

        for stop6 in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(90, 70)
            time.sleep(0.1)

        for back3 in range(50):
            drive(90, 70)

            if data_4 <= 5:
                time.sleep(1)
                break
            time.sleep(0.1)
        for finish in range(2):
            drive(90, 90)
            time.sleep(0.1)
            drive(90, 90)
            time.sleep(0.1)
        rate.sleep()
        break
        rospy.on_shutdown(exit_node)

    def accelerate(self, angle, left, mid, right):

        if min(left, mid, right) < 100:
            speed = min(left, mid, right) - 60
            if speed < 10:
                speed = 0

        elif angle < -20 or angle > 20:
            speed = 0
        else:
            speed = 0  # 4

        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    rospy.on_shutdown(car.exit)