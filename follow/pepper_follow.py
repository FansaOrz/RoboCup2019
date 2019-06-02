#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from threading import Thread
import thread
from .follow_module import pepper_follow
import time

class follow_me:
    def __init__(self, session):
        # 初始化follow me类
        self.a = pepper_follow(session)
        self.num = 1
        # 使能开关打开

    def start_follow(self):
        self.a.follow_enable = True
        print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        arg = tuple([1])
        thread.start_new_thread(self.a.follow, arg)
        # self.follow_thread = Thread(target=self.a.follow(), args=[])
        print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

        # self.follow_thread.start()
        print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

        # self.follow_thread.join()
        print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

    def stop_follow(self):
        self.a.follow_enable = False
