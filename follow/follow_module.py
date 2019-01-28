#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from time import sleep
from threading import Thread

class pepper_follow:
    def __init__(self, session):
        self.follow_enable = False
        self.Motion = session.service("ALMotion")
        self.Tracker = session.service("ALTracker")
        self.RobotPos = session.service("ALRobotPosture")
        self.PeoplePer = session.service("ALPeoplePerception")
        self.TextToSpe = session.service("ALTextToSpeech")
        # 设置成true时，所有其他可选的检测（比如脸、运动等）都将停用
        self.PeoplePer.setFastModeEnabled(True)
        # Set Stiffness
        name = "Body"
        stiffness = 1.0
        time = 1.0
        self.Motion.stiffnessInterpolation(name, stiffness, time)
        # Go to posture stand
        speed = 1.0
        self.RobotPos.goToPosture("Standing", speed)
        # 设置追踪模式
        mode = "Navigate"
        self.Tracker.setmode(mode)
        # 设置追踪目标
        self.target = "People"
        self.Tracker.trackEvent(self.target)
        # 调小安全距离
        self.Motion.setTangentialSecurityDistance(.05)
        self.Motion.setOrthogonalSecurityDistance(.1)
        # 设置追踪时的距离
        self.Tracker.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])

        print('\033[0;32m [Kamerider I] follow function initialized \033[0m')

    def follow(self):
        while self.follow_enable:
            target_position = self.Tracker.getTargetPosition(0)
            if not target_position:
                print("\033[0;32;40m\t[Kamerider W] : There is nobody in front of me\033[0m")
                self.TextToSpe.say("I can't see anyone, please stand in front of me.")
                continue
            print("↓↓↓↓↓↓↓↓↓↓↓↓point↓↓↓↓↓↓↓↓↓↓↓↓")
            print(target_position)
            print("↑↑↑↑↑↑↑↑↑↑↑↑point↑↑↑↑↑↑↑↑↑↑↑↑")
            self.Tracker.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])
            sleep(1)

    def __del__(self):
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()