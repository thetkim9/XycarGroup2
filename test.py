#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from imuread import ImuRead
from YOLO import YOLO

class AutoDrive:
    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.imu = ImuRead('diagnostics')
        self.before = [0, 0, 0]
        self.before_sonic = 0
        self.startTime = time.time()

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r, line_s = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        r, p, y = self.imu.get_data()
        angle = self.steer(line_l, line_r, r, p, y)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r, r, p, y, line_s)
        self.driver.drive(angle + 90, speed + 90)
        # print(r, p, y, "imu")
        #print(self.read)

    def steer(self, left, right, r, p, y):
        mid = (left + right) // 2

        if left == -1 and right == -1:
            left, right, mid = self.before[0], self.before[1], self.before[2]

        if left > 0 and right > 0 and mid < 150:
            left, right, mid = self.before[0], self.before[1], self.before[2]

        # if right != -1 and (self.before[2] - mid) > 5:
        #     mid = (mid + self.before[2]) * 1 // 2 + 5
        #     print((mid + self.before[2]) * 1 // 2 + 5)

        if left == -1:
            if mid - 300 < -50:
                angle = -50
            else:
                angle = mid - 300
        elif right == -1:
            if mid - 15 > 50:
                angle = 50
            else:
                angle = mid - 15
        elif mid > 325:
            if mid - 325 > 50:
                angle = 50
            else:
                angle = mid - 325
        elif mid < 295:
            if mid - 295 < -50:
                angle = -50
            else:
                angle = mid - 295
        else:
            angle = 0

        if p > 5:
            angle = 0
        elif p < -5:
            angle = 0

        self.before[0], self.before[1], self.before[2] = left, right, mid
        return angle

    def accelerate(self, angle, left, mid, right, r, p, y, stop):



        if angle > 0:
            angle = -angle
        #if mid < 110:
        #    speed = 0
        if angle < -30 or angle > 30:
            speed = 50
        elif angle < -10:
            speed = 50
        else:
            speed = 50

        if p > 7:
            speed = -40
        elif p < -7:
            speed = 35

        print(left, mid, right, self.before_sonic - mid)
        self.before_sonic = mid

        if stop ==1:
            speed = 0
        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
