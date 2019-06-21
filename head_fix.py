#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import time
import sys
import atexit
import thread
from threading import Thread


class speech_person_recog():

    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        self.angle = .3
        # naoqi API
        self.Memory = self.session.service("ALMemory")
        self.Motion = self.session.service("ALMotion")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.RobotPos = self.session.service("ALRobotPosture")

        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)
        self.start_head_fix()
        self.keyboard_control()

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')

    def start_head_fix(self):
        arg = tuple([1])
        self.state = True
        self.head_fix = True
        thread.start_new_thread(self.head_fix_thread, arg)

    def set_velocity(self, x, y, theta):
        self.Motion.move(x, y, theta)

    def stop(self):
        # self.cancel_plan()
        self.set_velocity(0, 0, 0)

    def head_fix_thread(self, arg):
        while self.head_fix:
            self.Motion.setStiffnesses("head", 1.0)
            self.Motion.setAngles("Head", [0., self.angle], .05)
            time.sleep(2)

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'c':
                    break
                elif command == 'set':
                    self.angle = float(raw_input("angle:"))
                elif command == 'w':
                    self.set_velocity(.25, 0, 0)
                elif command == 's':
                    self.stop()
                elif command == 'x':
                    self.set_velocity(-0.25, 0, 0)
                elif command == 'a':
                    self.set_velocity(0, 0.25, 0)
                elif command == 'd':
                    self.set_velocity(0, -0.25, 0)
                elif command == 'q':
                    self.set_velocity(0, 0, 0.35)
                elif command == 'e':
                    self.set_velocity(0, 0, -0.35)
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"



if __name__ == "__main__":
    params = {
        'ip': "192.168.3.93",
        'port': 9559
    }
    speech_person_recog(params)
