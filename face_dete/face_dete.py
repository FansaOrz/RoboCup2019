#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import rospy
import time
from .face_dete_module import face_dete
from threading import Thread
from geometry_msgs.msg import Twist


class face_dete_control:
    def __init__(self, session):
        self.a = face_dete(session)

    def start_face_dete(self):
        print self.a.switch_face_dete
        self.a.switch_face_dete = True
        print "======"
        print self.a.switch_face_dete
        self.a.start_face_dete()



