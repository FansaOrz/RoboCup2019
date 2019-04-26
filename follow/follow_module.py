#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from time import sleep
from threading import Thread
import time


class pepper_follow:

    def __init__(self, session):
        self.follow_enable = False
        self.Motion = session.service("ALMotion")
        self.Tracker = session.service("ALTracker")
        self.RobotPos = session.service("ALRobotPosture")
        self.PeoplePer = session.service("ALPeoplePerception")
        self.TextToSpe = session.service("ALTextToSpeech")
        self.PeopPer = session.service("ALPeoplePerception")
        self.Memory = session.service("ALMemory")
        self.People_Dete = self.Memory.subscriber("PeoplePerception/PeopleDetected")
        self.People_Dete.signal.connect(self.callback_people_dete)
        # 设置成true时，所有其他可选的检测（比如脸、运动等）都将停用
        self.PeoplePer.setFastModeEnabled(True)
        # Set Stiffness
        name = "Body"
        stiffness = 1.0
        time = 1.0
        self.Motion.stiffnessInterpolation(name, stiffness, time)
        # Go to posture stand
        speed = 1.0
        self.people_id = 0
        self.RobotPos.goToPosture("Standing", speed)
        # 设置追踪模式
        # mode = "Navigate"
        # self.Tracker.setMode(mode)

        print("                        ↓                            ")
        print('\033[0;32m [Kamerider I] follow function initialized \033[0m')

    def callback_people_dete(self, msg):
        print "00000000000000000000"
        self.people_id = msg[1][0][0]

    def follow(self):
        print "111111================="
        # tracker_service.trackEvent("Face")
        # while self.follow_enable:
        #     if self.people_id == 0:
        #         print("\033[0;32;40m\t[Kamerider W] : There is nobody in front of me\033[0m")
        #         time.sleep(2)
        #         # self.TextToSpe.say("I can't see you, please adjust the distance between us  ")
        #         continue
        #     else:
        #         self.Tracker.registerTarget(self.target, self.people_id)
        #         print "registe Target successfully!!"
        #         break
        while self.follow_enable:
            print "111111"

            # 获得机器人躯干坐标系下距离目标的距离
            target_position = self.Tracker.getTargetPosition(0)
            if not target_position:
                continue
            # 距离大于1.7m
            if target_position[0] > 1.2:
                self.TextToSpe.say("Please slow down, I can not follow you")
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def __del__(self):
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()
