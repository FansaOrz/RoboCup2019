#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import cv2
import dlib
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist


class wave_detection:
    def __init__(self, session):
        self.VideoDev = session.service("ALVideoDevice")
        self.Motion = session.service("ALMotion")
        self.TextToSpe = session.service("ALTextToSpeech")

        # rospy.init_node("wave_detection")
        self.RobotPos = session.service("ALRobotPosture")
        self.BasicAwa = session.service("ALBasicAwareness")
        self.AutonomousLife = session.service("ALAutonomousLife")
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("StandInit", .2)


        self.get_image_switch = True
        self.stop_time = 0 #在每一个人前面停留的次数，检测这个人是否在挥手
        self.turn_right_time = 0
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.detector = dlib.get_frontal_face_detector()

    def set_velocity(self, x, y, theta, duration=-1.):  # m/sec, rad/sec
        # if duration > 0 : stop after duration(sec)
        tt = Twist()
        tt.linear.x = x
        tt.linear.y = y
        tt.angular.z = theta
        self.cmd_vel_pub.publish(tt)
        if duration < 0: return None
        tic = time.time()
        while time.time() - tic < duration:
            self.cmd_vel_pub.publish(tt)
            time.sleep(0.1)
        tt = Twist()
        tt.linear.x = 0
        tt.linear.y = 0
        tt.angular.z = 0
        self.cmd_vel_pub.publish(tt)

    def find_person(self):
        AL_kQVGA = 1
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        width = 320
        height = 240
        image = np.zeros((height, width, 3), np.uint8)
        while self.get_image_switch:
            result = self.VideoDev.getImageRemote(nameId)
            if result == None:
                print 'cannot capture.'
            elif result[6] == None:
                print 'no image data string.'
            else:
                values = map(ord, list(str(bytearray(result[6]))))
                i = 0
                for y in range(0, height):
                    for x in range(0, width):
                        image.itemset((y, x, 0), values[i + 0])
                        image.itemset((y, x, 1), values[i + 1])
                        image.itemset((y, x, 2), values[i + 2])
                        i += 3
                # print image
                cv2.imshow("pepper-top-camera-640*480px", image)
                cv2.waitKey(1)
                self.wave_detection(image)
        return "succe"

    def wave_detection(self, frame):
        # print "9999"
        frame_copy = frame.copy()
        rects = self.detector(frame_copy, 2)
        if len(rects) != 0:
            # print "yyyyyyyyyyy"
            for rect in rects:
                cv2.rectangle(frame_copy, (int(3 * rect.left() - 2 * rect.right()), int(rect.top() - (rect.bottom() - rect.top()) / 3)),
                              (int((4/3) * rect.left() - (2/3) * rect.right()), int(rect.bottom() + (rect.bottom() - rect.top()) * 1.3)), (0, 0, 255), 2, 8)
                cv2.imshow("yess", frame_copy)
                cv2.waitKey(1)
                frame = frame_copy[int(rect.top() - (rect.bottom() - rect.top()) / 3): int(rect.bottom() + (rect.bottom() - rect.top()) * 1.3),
                        int(3 * rect.left() - 2 * rect.right()): int((4/3) * rect.left() - (2/3) * rect.right())]
                blur = cv2.blur(frame, (3, 3))
                try:
                    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
                except:
                    continue
                mask2 = cv2.inRange(hsv, np.array([2, 50, 50]), np.array([15, 255, 255]))
                # mask2 = cv2.inRange(hsv, np.array([1, 20, 20]), np.array([60, 255, 255]))
                cv2.imshow("mask2", mask2)
                cv2.waitKey(1)
                kernel_square = np.ones((11, 11), np.uint8)
                kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
                dilation = cv2.dilate(mask2, kernel_ellipse, iterations=1)
                erosion = cv2.erode(dilation, kernel_square, iterations=1)
                dilation2 = cv2.dilate(erosion, kernel_ellipse, iterations=1)
                filtered = cv2.medianBlur(dilation2, 5)
                kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
                dilation2 = cv2.dilate(filtered, kernel_ellipse, iterations=1)
                kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                dilation3 = cv2.dilate(filtered, kernel_ellipse, iterations=1)
                median = cv2.medianBlur(erosion, 5)
                _, thresh = cv2.threshold(median, 127, 255, 0)
                cv2.imshow('Dilation1', median)
                cv2.waitKey(1)
                height, width = median.shape
                num_white = num_sum = 0
                for i in range(height):
                    for j in range(width):
                        # print median[i, j]
                        if median[i, j] == 255:
                            num_white += 1
                        num_sum += 1
                print "==================", float(num_white) / float(num_sum)
                if float(num_white) / float(num_sum) <= 0.03:
                    self.stop_time += 1
                    if self.stop_time <= 5:
                        continue
                    else:
                        self.stop_time = 0
                        if self.turn_right_time <= 7:
                            print "self.turn_right_time", self.turn_right_time
                            self.turn_right_time += 1
                            # self.set_velocity(0, 0, -0.2, duration=.7)
                        else:
                            self.Motion.move(0, 0, 1)
                            duration = time.time() + 4
                            while time.time() < duration:
                                continue
                            self.Motion.stopMove()
                            self.turn_right_time = 0
                        print "turn left times:"
                else:
                    self.stop_time = 0
                    # self.TextToSpe.say("hey, you are waving your hand")
                    left_right_center = int((rect.left() + rect.right()) / 2)
                    # top_bottom_center = int((rect.top() + rect.bottom()) / 2)



                    # if left_right_center > 190:
                    #     self.set_velocity(0, 0, -0.1, duration=1)
                    # elif left_right_center < 130:
                    #     self.set_velocity(0, 0, -0.1, duration=1)
                    # else:
                    #     self.set_velocity(0.1, 0, 0, duration=1)
                    # print "-------------------------------------"



                    if (rect.bottom() - rect.top()) * (rect.right() - rect.left()) > 1400:
                        self.get_image_switch = False
                        self.TextToSpe.say("I have found you.")
        else:
            if self.turn_right_time <= 7:
                # self.set_velocity(0, 0, -0.2, duration=.7)
                self.turn_right_time += 1
            else:
                # self.Motion.move(0, 0, 1)
                # duration = time.time() + 4
                # while time.time() < duration:
                #     continue
                # self.Motion.stopMove()
                self.turn_right_time = 0

            # print "turn left times:"

if __name__ == "__main__":
    sess = qi.Session()
    try:
        sess.connect("tcp://192.168.43.30:9559")
    except RuntimeError:
        print("[Kamerider E] : connection Error!!")
    temp = wave_detection(sess)
    temp.find_person()