#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time

from linedetector3 import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
import cv2


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

        self.speed = 0
        self.speed2 = 0

    # 장애물, 차선 인식하여 조향각, 속도 결정, 모터 제어 메시지 발생
    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        left, right = self.line_detector.detect_lines()
        self.len30, self.len40, self.len50 = self.line_detector.img_match()
        print(self.len30, self.len40, self.len50)
        point = self.line_detector.show_images(left, right)  # 디버깅위한 화면 표시
        angle = self.steer(left, right)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r, point)  # 조향각과 속도 인자 결정
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

    # 조향각, 장애물 거리에 따라 속도 결정
    def accelerate(self, angle, left, mid, right, point):

        if self.len30 > 30:
            self.speed = 5
            print("speed is 30")

        elif self.len40 > 20:
            self.speed = 10
            print("speed is 40")


        elif self.len50 > 15:
            self.speed = 15
            print("speed is 50")

        if min(left, mid, right) < 100:
            speed = min(left, mid, right) - 60
            if speed < 10:
                speed = 0

        elif angle < -20 or angle > 20:
            speed = 0
        else:
            speed = 0  # 40

        if len(point) == 0:
            pass
        else:

            if point[1] >= 155 and point[2] >= 155:
                pass
            elif 180 <= point[2]:
                print("red")
                self.speed2 = 0
                self.speed = 0
            elif 150 <= point[1]:
                print("green")
                self.speed2 = 15

        return self.speed2 + self.speed

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
