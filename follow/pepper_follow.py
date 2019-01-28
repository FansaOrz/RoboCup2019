#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from threading import Thread
from .follow_module import pepper_follow


class follow_me:
    def __init__(self, session):
        # 初始化follow me类
        pepper_follow(session)
        # 使能开关打开
        pepper_follow.follow_enable = True
        self.follow_thread = Thread(target=pepper_follow.follow, args=[])
        self.follow_thread.start()

    def stop_following(self):
        pepper_follow.follow_enable = False
        print('\033[0;32m [Kamerider I] follow function stopped \033[0m')


