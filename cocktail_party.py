#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import time
import atexit
import rospy

import thread
from gender_predict import baidu_gender
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from threading import Thread


class speech_person_recog():

    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        rospy.init_node("cocktail_party")

        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)

        # naoqi API
        self.Memory = self.session.service("ALMemory")
        self.Motion = self.session.service("ALMotion")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.TabletSer = self.session.service("ALTabletService")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.Dialog = self.session.service("ALDialog")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.PhotoCap = self.session.service("ALPhotoCapture")
        # stop recording
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;32;40m\t[Kamerider W]ALFaceCharacteristics : You don't need stop record\033[0m")
        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        # amcl定位
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        # 声明一些变量
        self.angle = -.2
        self.if_need_record = False
        self.point_dataset = self.load_waypoint("waypoints.txt")
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 初始化平板
        self.TabletSer.cleanWebview()
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')
        # 设置dialog语言
        self.Dialog.setLanguage("English")
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "None"
        # Beep 音量
        self.beep_volume = 70
        # 设置初始化头的位置 走路的时候固定头的位置
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., -0.25], .05)
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 80.0)
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .8)
        self.SoundDet.subscribe('sd')
        self.SoundDet_s = self.Memory.subscriber("SoundDetected")
        self.SoundDet_s.signal.connect(self.callback_sound_det)
        # 调用成员函数
        self.start_head_fix()
        self.set_volume(.7)
        self.keyboard_control()

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()

    def start_head_fix(self):
        arg = tuple([1])
        self.state = True
        self.head_fix = True
        thread.start_new_thread(self.head_fix_thread, arg)

    def head_fix_thread(self, arg):
        while self.head_fix:
            self.Motion.setStiffnesses("head", 1.0)
            self.Motion.setAngles("Head", [0., self.angle], .05)
            time.sleep(2)

    def say(self, text):
        self.TextToSpe.say(text)

    def load_waypoint(self, file_name):
        curr_pos = PoseStamped()
        f = open(file_name, 'r')
        sourceInLines = f.readlines()
        dataset_points = {}
        for line in sourceInLines:
            temp1 = line.strip('\n')
            temp2 = temp1.split(',')
            point_temp = MoveBaseGoal()
            point_temp.target_pose.header.frame_id = '/map'
            point_temp.target_pose.header.stamp = curr_pos.header.stamp
            point_temp.target_pose.header.seq = curr_pos.header.seq
            point_temp.target_pose.pose.position.x = float(temp2[1])
            point_temp.target_pose.pose.position.y = float(temp2[2])
            point_temp.target_pose.pose.position.z = float(temp2[3])
            point_temp.target_pose.pose.orientation.x = float(temp2[4])
            point_temp.target_pose.pose.orientation.y = float(temp2[5])
            point_temp.target_pose.pose.orientation.z = float(temp2[6])
            point_temp.target_pose.pose.orientation.w = float(temp2[7])
            dataset_points[temp2[0]] = point_temp
        print ("↓↓↓↓↓↓↓↓↓↓↓↓point↓↓↓↓↓↓↓↓↓↓↓↓")
        print (dataset_points)
        print ("↑↑↑↑↑↑↑↑↑↑↑↑point↑↑↑↑↑↑↑↑↑↑↑↑")
        print ('\033[0;32m [Kamerider I] Points Loaded! \033[0m')
        return dataset_points

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'sr':
                    self.start_record()
                elif command == 'c':
                    break
                elif command == 'ta':
                    self.turn_around()
                elif command == 'tp':
                    self.take_picture()
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"



if __name__ == "__main__":
    params = {
        'ip': "192.168.3.12",
        'port': 9559
    }
    speech_person_recog(params)