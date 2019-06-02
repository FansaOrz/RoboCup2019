#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import re
import sys
import time
import rospy
import atexit
import thread
import datetime
import actionlib
from threading import Thread
from json import dumps
from follow import pepper_follow
from face_dete import face_dete
from speech_recog import baidu_recognition_text
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class help_me_carry():

    def __init__(self, params):
        # 初始化ROS节点
        rospy.init_node("help_me_carry")
        # 退出程序的时候进入__del__函数
        atexit.register(self.__del__)
        # 初始化pepper的ip和port
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        # 尝试连接pepper
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        #    需要使用的naoqi api
        self.Leds = self.session.service("ALLeds")
        self.Memory = self.session.service("ALMemory")
        self.Dialog = self.session.service("ALDialog")
        self.Motion = self.session.service("ALMotion")
        self.Tracker = self.session.service("ALTracker")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.AnimatedSpe = self.session.service("ALAnimatedSpeech")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        # 设置追踪目标
        self.current_place = "none"
        # 调小安全距离
        self.Motion.setTangentialSecurityDistance(.05)
        self.Motion.setOrthogonalSecurityDistance(.1)
        # 设置追踪时的距离
        self.Tracker.setRelativePosition([-.2, 0.0, 0.0, 0.1, 0.1, 0.3])
        self.Tracker.setMode("Move")
        self.target = "Face"
        self.return_ = False
        self.Tracker.registerTarget(self.target, .2)
        # 设置追踪时的距离        # 停止录音
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;33m\t[Kamerider W] : You don't need stop record\033[0m")
        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        self.if_stop_follow = False
        self.if_need_head = True
        self.if_start_follow = False
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.follow_start_pub = rospy.Publisher('/switch_pub_cmd', String, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        self.follow_enable = True
        # amcl定位point_dataset
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        # 声明一些变量
        self.angle = -0.4
        self.if_ask_time = False
        self.if_need_record = False
        self.point_dataset = self.load_waypoint("waypoints_help.txt")
        #    LED的group
        self.led_name = ["Face/Led/Blue/Right/0Deg/Actuator/Value", "Face/Led/Blue/Right/45Deg/Actuator/Value",
                         "Face/Led/Blue/Right/90Deg/Actuator/Value", "Face/Led/Blue/Right/135Deg/Actuator/Value",
                         "Face/Led/Blue/Right/180Deg/Actuator/Value", "Face/Led/Blue/Right/225Deg/Actuator/Value",
                         "Face/Led/Blue/Right/270Deg/Actuator/Value", "Face/Led/Blue/Right/315Deg/Actuator/Value",
                         "Face/Led/Blue/Left/0Deg/Actuator/Value", "Face/Led/Blue/Left/45Deg/Actuator/Value",
                         "Face/Led/Blue/Left/90Deg/Actuator/Value", "Face/Led/Blue/Left/135Deg/Actuator/Value",
                         "Face/Led/Blue/Left/180Deg/Actuator/Value", "Face/Led/Blue/Left/225Deg/Actuator/Value",
                         "Face/Led/Blue/Left/270Deg/Actuator/Value", "Face/Led/Blue/Left/315Deg/Actuator/Value"]
        self.Leds.createGroup("MyGroup", self.led_name)
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 初始化平板
        self.TabletSer.cleanWebview()
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')

        # 初始化关键字
        self.place = ["bathroom", "kitchen", "party room", "dining room","dinning room", "living room", "bedroom"]
        self.start = ["follow", "following", "start", "follow me"]
        self.stop = ["stop", "here is the car"]
        self.go_back = ["bathroom", "living room", "bedroom", "kitchen", "toilet"]
        # 当前时间戳（订阅相机的名字，每个只能使用6次）
        ticks = time.time()
        # 0代表top相机 最后一个参数是fps
        self.VideoDev.subscribeCamera(str(ticks), 0, 2, 11, 40)
        # 设置dialog语言
        self.Dialog.setLanguage("English")
        self.TextToSpe.setLanguage("English")
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "None"
        # Beep 音量
        self.beep_volume = 70
        self.head_fix = True
        # 设置初始化头的位置 走路的时候固定头的位置
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., self.angle], .05)
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 80.0)
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)
        # follow me function
        self.pepper_follow_me = pepper_follow.follow_me(self.session)
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .4)
        self.SoundDet.subscribe('sd')
        self.SoundDet_s = self.Memory.subscriber("SoundDetected")
        self.SoundDet_s.signal.connect(self.callback_sound_det)
        # 调用成员函数
        # self.start_head_fix()
        self.set_volume(.7)
        self.keyboard_control()

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def start_foll(self):
        '''
        self.TextToSpe.say("Dear operator.")
        self.TextToSpe.say("Please call my name pepper, before each question")
        self.TextToSpe.say("Please talk to me after you heard ")
        self.AudioPla.playSine(1000, self.beep_volume, 1, .3)
        time.sleep(.5)
        self.TextToSpe.say("three")
        time.sleep(.5)
        '''
        self.TextToSpe.say("two")
        time.sleep(.5)
        self.TextToSpe.say("one")
        self.start_recording(reset=True)
        self.analyze_content()
        while self.if_start_follow != True:
            self.TextToSpe.say("sorry please tell me again")
            time.sleep(.5)
            self.TextToSpe.say("three")
            time.sleep(.5)
            self.TextToSpe.say("two")
            time.sleep(.5)
            self.TextToSpe.say("one")
            self.start_recording(reset=True)
            self.analyze_content()
        print('\033[0;32m [Kamerider I] start following the operator \033[0m')
        # start follow function
        self.angle = .0
        # print "19191991911919919191"
        # self.follow()
        self.pepper_follow_me.start_follow()
        # self.follow_start_pub.publish("on")
        while not self.if_stop_follow:
            print "--0-0-0-0-0-0-0-0-0-0-0-00-"
            rospy.sleep(.5)
            self.start_recording(reset=True)
            self.analyze_content()
        print "--------------------------------------------------------------====================="


        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = -2.95008523941
        init_pose.pose.pose.position.y = 0.435851972961
        init_pose.pose.pose.orientation.z = 0.999605670231
        init_pose.pose.pose.orientation.w = 0.0280803141226
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] = 0.06853
        self.init_pose_pub.publish(init_pose)


        # self.pepper_follow_me
        self.start_head_fix()
        while not self.return_:
            self.start_recording(reset=True)
            self.analyze_content()
        self.TextToSpe.say("ok, I will go to the " + self.current_place + ", to find someone to help me")
        if self.current_place == "kitchen":
            self.go_to_waypoint(self.point_dataset["point11"], "point1")
            self.go_to_waypoint(self.point_dataset["point2"], "point2")
            self.go_to_waypoint(self.point_dataset["point3"], "point3")
            self.go_to_waypoint(self.point_dataset["point5"], "point4")
            self.go_to_waypoint(self.point_dataset["point8"], "point4")
            self.TextToSpe.say("I have arrived at kitchen")
            # self.go_to_waypoint(self.point_dataset["party room"], "party room")
            # self.Motion.moveTo(1, 0, 0)
            # self.Motion.moveTo(0, 0, -3.14 / 2)
        elif self.current_place == "bedroom":
            self.go_to_waypoint(self.point_dataset["point11"], "point1")
            self.go_to_waypoint(self.point_dataset["point2"], "point2")
            self.go_to_waypoint(self.point_dataset["point3"], "point2")
            self.go_to_waypoint(self.point_dataset["point4"], "point2")
            self.go_to_waypoint(self.point_dataset["point9"], "point2")
            self.TextToSpe.say("I have arrived at bedroom")
            # self.go_to_waypoint(self.point_dataset["bedroom"], "bedroom")
            # self.Motion.moveTo(.5, 0, 0)
        elif self.current_place == "living room":
            self.go_to_waypoint(self.point_dataset["point11"], "point1")
            self.go_to_waypoint(self.point_dataset["point1"], "point2")
            # self.go_to_waypoint(self.point_dataset["point14"], "point3")
            self.TextToSpe.say("I have arrived at living room")
        elif self.current_place == "dinning room" or self.current_place == "dining room":
            self.go_to_waypoint(self.point_dataset["point11"], "point1")
            self.go_to_waypoint(self.point_dataset["point2"], "point1")
            self.go_to_waypoint(self.point_dataset["point3"], "point1")
            self.go_to_waypoint(self.point_dataset["point17"], "point2")
            # self.go_to_waypoint(self.point_dataset["point14"], "point3")
            self.TextToSpe.say("I have arrived at dinning room")

            # self.go_to_waypoint(self.point_dataset["point5"], "point5")
        self.head_fix = False
        self.Motion.setStiffnesses("Head", 1.0)
        self.angle = -.3
        self.Motion.setAngles("Head", [0., self.angle], .05)
        self.face = face_dete.face_dete_control(self.session)
        self.face.start_face_dete()
        # self.go_to_waypoint(self.point_dataset["point5"], "point5")
        # self.go_to_waypoint(self.point_dataset["point6"], "point6")
        # self.go_to_waypoint(self.point_dataset["point7"], "point7")
        self.TextToSpe.say("this is the car location")
        self.TextToSpe.say("i have finished the mission")


    def callback_people_dete(self, msg):
        print "00000000000000000000"
        self.people_id = msg[1][0][0]

    def follow(self):
        self.Tracker.trackEvent("Face")
        print "111111================="
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
            self.start_recording(reset=True, withBeep=False)
            self.analyze_content()
            if self.if_stop_follow == True:
                break

            # 获得机器人躯干坐标系下距离目标的距离
            target_position = self.Tracker.getTargetPosition(0)
            if not target_position:
                continue
            # 距离大于1.7m
            if target_position[0] > 1.2:
                self.TextToSpe.say("Please slow down, I can not follow you")
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def callback_sound_det(self, msg):
        if self.if_need_record:
            print ('\033[0;32m [Kamerider I] Sound detected (In callback function) \033[0m')
            ox = 0
            for i in range(len(msg)):
                if msg[i][1] == 1:
                    ox = 1
            if ox == 1 and self.enable_speech_recog:
                self.record_time = time.time() + self.record_delay
                print "self.record_time += 3s ... ", self.record_time
                #if not self.thread_recording.is_alive():
                #    self.start_recording(reset=True)
                while self.recog_result == "00":
                    time.sleep(1)
                    continue
            else:
                return None
        else:
            print ('\033[0;32m [Kamerider I] Sound detected, but we don\'t need to record this audio \033[0m')

    def start_recording(self, reset=False, base_duration=3, withBeep=True):
        self.if_need_record = True
        self.record_time = time.time() + base_duration
        if reset:
            self.kill_recording_thread()
            self.AudioRec.stopMicrophonesRecording()
        print "self.record_time is:  ", self.record_time
        if not self.thread_recording.is_alive():
            self.thread_recording = Thread(target=self.record_audio, args=(self.speech_hints, withBeep))
            self.thread_recording.daemon = False
            self.thread_recording.start()
            self.thread_recording.join()
            print('\033[0;32m [Kamerider I] Start recording thread \033[0m')

    def analyze_content(self):
        for i in range(len(self.start)):
            if re.search(self.start[i].lower(), self.recog_result) != None:
                self.recog_result = "None"
                self.angle = -.1
                self.TextToSpe.say("I will start following you")
                self.if_start_follow = True
                return "follow"
        # for i in range(len(self.go_back)):
        #     if re.search(self.go_back[i], self.recog_result) != None:
        #         self.recog_result = "None"
        #         print('\033[0;32m [Kamerider I] Start navigating to ' + self.go_back[i] + '  \033[0m')
        #         self.kill_recording_thread()
        #         # navigation function
        #         self.TextToSpe.say("I'm going to the kitchen")
        #         self.go_to_waypoint(self.point_dataset[self.go_back[i]], self.go_back[i], label="go_back")
        #         # 调整头部的角度
        #         self.angle = -.4
        #         self.face_dete()
        #         return
        for i in range(len(self.stop)):
            if re.search(self.stop[i], self.recog_result) != None:
                # self.follow_start_pub.publish("off")
                self.pepper_follow_me.stop_follow()
                self.recog_result = "None"
                self.TextToSpe.say("I will stop following you")
                self.if_stop_follow = True
                print('\033[0;32m [Kamerider I] Stop following the person  \033[0m')
                return "stop"
        self.current_place = "none"
        for i in range(len(self.place)):
            if re.search(self.place[i].lower(), self.recog_result) != None:
                print self.place[i]
                self.current_place = self.place[i]
                self.return_ = True
                return
                # 记住车的位置

                # self.TextToSpe.say("I have memorized the location of the car")
                # self.thread_recording.join()
                # self.pepper_follow_me.stop_follow()
                # return

        self.start_recording(reset=True, withBeep=False)
        self.analyze_content()

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            self.if_need_record = False

    def record_audio(self, hints, withBeep = True):
        # 亮灯
        self.Leds.on("MyGroup")
        if withBeep:
            # 参数 playSine(const int& frequence, const int& gain, const int& pan, const float& duration)
            self.AudioPla.playSine(1000, self.beep_volume, 1, .3)
        print('\033[0;32m [Kamerider I] start recording... \033[0m')
        channels = [0, 0, 1, 0]
        self.AudioRec.startMicrophonesRecording("/home/nao/audio/recog.wav", "wav", 16000, channels)
        while time.time() < self.record_time:
            if self.audio_terminate:
                # 如果终止为True
                self.AudioRec.stopMicrophonesRecording()
                print('\033[0;32m [Kamerider I] Killing recording... \033[0m')
                return None
            time.sleep(.1)
        self.Leds.off("MyGroup")
        self.AudioRec.stopMicrophonesRecording()
        self.AudioRec.recording_ended = True
        if not os.path.exists('./audio_record'):
            # TODO
            os.mkdir('./audio_record', 0o755)
        # 复制录下的音频到自己的电脑上
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/audio/recog.wav ./audio_record"
        os.system(cmd)
        print('\033[0;32m [Kamerider I] Record ended start recognizing \033[0m')
        self.recog_result = baidu_recognition_text.main("./audio_record/recog.wav").lower()
        print "===============", self.recog_result

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while self.if_need_head:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

    def show_person_image(self):
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImage("http://198.18.0.1/apps/boot-config/person_image.png")

    def face_dete(self):
        # face detection函数
        self.face = face_dete.face_dete_control(self.session)
        self.face.start_face_dete()
        self.TextToSpe.say("Please follow me to the car to carry some items")
        self.go_to_waypoint(self.point_dataset["car"], "car", label="car")

    def get_car_position(self):
        curr_pos = PoseStamped()
        car_position = MoveBaseGoal()
        car_position.target_pose.header.frame_id = '/map'
        car_position.target_pose.header.stamp = curr_pos.header.stamp
        car_position.target_pose.header.seq = curr_pos.header.seq
        car_position.target_pose.pose.position.x = self.car_pose.pose.pose.position.x
        car_position.target_pose.pose.position.y = self.car_pose.pose.pose.position.y
        car_position.target_pose.pose.position.z = self.car_pose.pose.pose.position.z
        car_position.target_pose.pose.orientation.x = self.car_pose.pose.pose.orientation.x
        car_position.target_pose.pose.orientation.y = self.car_pose.pose.pose.orientation.y
        car_position.target_pose.pose.orientation.z = self.car_pose.pose.pose.orientation.z
        car_position.target_pose.pose.orientation.w = self.car_pose.pose.pose.orientation.w
        self.point_dataset["car"] = car_position
        print self.point_dataset["car"]

    def amcl_callback(self, msg):
        self.car_pose= msg

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

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

    def go_to_waypoint(self, Point, destination, label = None):
        self.angle = .3
        self.nav_as.send_goal(Point)
        self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            # 如果有人问时间了
            if self.if_ask_time:
                # 测试是不是取消导航
                self.goal_cancel_pub.publish(GoalID())
                # %y 两位数的年份表示（00-99）%Y 四位数的年份表示（000-9999）%m 月份（01-12）
                # %d 月内中的一天（0-31）%H 24小时制小时数（0-23）%I 12小时制小时数（01-12）
                # %M 分钟数（00=59）%S 秒（00-59）
                current_time = time.strftime('%H:%M',time.localtime(time.time())).split(":")
                sentence = "The time is " + current_time[0] + " " + current_time[1]
                print('\033[0;32m [Kamerider I] ' + sentence + ' \033[0m')
                self.TextToSpe.say(sentence)
                sentence = "Excuse me, I need to go to the " + destination + ", Please let me go"
                self.TextToSpe.say(sentence)
                sentence = "Thank you"
                time.sleep(2)
                self.TextToSpe.say(sentence)
                time.sleep(2)
                self.if_ask_time = False
                self.map_clear_srv()
                self.nav_as.send_goal(Point)
                # 每隔4s清除一次local map
            # if count_time == 3:
            #     self.map_clear_srv()
            #     count_time = 0
        if label == "go_back":
            print('\033[0;32m [Kamerider I] I have arrived at ' + destination + ', start looking for people \033[0m')
            self.TextToSpe.say("I have arrived at " + destination)
            # find person function
        elif label == "car":
            self.TextToSpe.say("I have arrived the car position.")
            self.TextToSpe.say("Mission succeeded")
            self.__del__()

    def cancel_plan(self):
        self.goal_cancel_pub.publish(GoalID())

    def stop_motion(self):
        self.cancel_plan()
        self.set_velocity(0, 0, 0)

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

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'w':
                    self.set_velocity(0.25, 0, 0)
                elif command == 's':
                    self.stop_motion()
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
                elif command == 'qq':
                    self.set_velocity(0, 0, 1)
                elif command == 'ee':
                    self.set_velocity(0, 0, -1)
                elif command == 'gc':
                    self.get_car_position()
                elif command == 'go':
                    self.go_to_waypoint(self.point_dataset["kitchen"], "kitchen", "none")
                elif command == 'sr':
                    self.start_foll()
                elif command == 'c':
                    self.cancel_plan(); self.set_velocity(0, 0, 0); break
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"

def main():
    params = {
        'ip' : "192.168.43.30",
        'port' : 9559,
        'rgb_topic' : 'pepper_robot/camera/front/image_raw'
    }
    help_me_carry(params)

if __name__ == "__main__":
    main()
