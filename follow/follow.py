#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import time
from follow_me import follow_me


class pepper_follow:

    def __init__(self, session):
        # self.temp = follow_me.FollowMe(session)
        self.motion_service = session.service("ALMotion")
        self.posture_service = session.service("ALRobotPosture")
        self.people_service = session.service("ALPeoplePerception")
        self.tracker_service = session.service("ALTracker")
        self.RobotPos = session.service("ALRobotPosture")

        self.tts = session.service("ALTextToSpeech")
        self.AutonomousLife = session.service("ALAutonomousLife")
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)
        self.tts.setLanguage("English")
        self.tts.setParameter("speed", 90)
        self.motion_service.wakeUp()

        # Set Stiffness
        names = "Body"
        stiffness_lists = 1.0
        time_lists = 1.0
        self.motion_service.stiffnessInterpolation(names, stiffness_lists, time_lists)

        # Go to posture stand
        fraction_max_speed = 1.0
        self.posture_service.goToPosture("Standing", fraction_max_speed)
        self.event_name = "Face"
        self.no_person_warning = True

        mode = "Head"
        self.tracker_service.setMode(mode)
        self.tracker_service.trackEvent(self.event_name)

        # minimize Security Distance
        self.motion_service.setTangentialSecurityDistance(0.05)
        self.motion_service.setOrthogonalSecurityDistance(0.10)

        self.tracker_service.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])
        self.start_follow()

    def start_follow(self):
        while True:
            self.tracker_service.trackEvent(self.event_name)
            position = self.tracker_service.getTargetPosition(0)

            if not position:

                print("No person in sight")
                time.sleep(1)
                if self.no_person_warning:
                    self.no_person_warning = False
                    self.tts.say("I cant see you. Please get in front of me.")
            self.tracker_service.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])

    def __del__(self):
        # self.Tracker.stopTracker()
        # self.Tracker.unregisterAllTargets()
        print "[[["


if __name__ == '__main__':
    session = qi.Session()
    try:
        session.connect("tcp://172.16.0.10:9559")
    except RuntimeError:
        print("[Kamerider E] : connection Error!!")
        sys.exit(1)
    pepper_follow(session)