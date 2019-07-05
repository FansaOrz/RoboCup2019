#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class get_pose():
    def __init__(self, file_name):
        self.num = 56
        rospy.init_node("get_pose")
        self.f = open(file_name, 'a')
        self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback)
        # rospy.spin()
        self.keyboard_control()

        # self.init_odom.pose.pose.orientation.x

    def init_pose_callback(self, msg):
        lines = "point"+ str(self.num)+","+str(msg.pose.pose.position.x)+","+str(msg.pose.pose.position.y)+","+\
                    str(msg.pose.pose.position.z)+","+str(msg.pose.pose.orientation.x)+","+str(msg.pose.pose.orientation.y)+\
                    ","+str(msg.pose.pose.orientation.z)+","+str(msg.pose.pose.orientation.w)+"\n"
        self.f.write(lines)
        self.num += 1

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'c':
                    break
                else:
                    print("Invalid Command!")
            except Exception as e:
                print e
        self.f.close()

if __name__ == '__main__':
    tf_cl = get_pose("./aa.txt")

