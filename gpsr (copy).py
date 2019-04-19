#!/usr/bin/env python3
# -*- encoding: UTF-8 -*-


import os
import re
import qi
import sys
import time
import rospy
import thread
import actionlib
import atexit
from threading import Thread
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Empty
from face_dete import face_dete
from speech_recog import baidu_recognition_text
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped,Twist

class gpsr():
    def __init__(self, params):
        # 初始化ROS节点
        rospy.init_node("gpsr_node")
        # 程序退出时的回调函数
        atexit.register(self.__del__)
        self.ip = params['ip']
        self.port = params['port']
        self.session = qi.Session()
        # 连接pepper
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except Exception as e:
            print e
            # 输出红字
            print("\033[0;30m\t[Kamerider E] : connection Error!!\033[0m")
            sys.exit(1)
        self.if_head_fix = True
        # 需要的naoqi的服务
        self.Leds = self.session.service("ALLeds")
        self.Dialog = self.session.service("ALDialog")
        self.Memory = self.session.service("ALMemory")
        self.Motion = self.session.service("ALMotion")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.TabletSer = self.session.service("ALTabletService")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.recog_result = "00"
        # 停止录音
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;33m\t[Kamerider W] : You don't need stop record\033[0m")
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
        self.if_need_record = False


        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器
        # 声明一些变量
        self.point_init = MoveBaseGoal()
        self.point_dataset = self.load_waypoint("waypoints_gpsr.txt")

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 初始化平板
        self.TabletSer.cleanWebview()
        self.TabletSer.hideImage()
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .2)
        self.SoundDet.subscribe('sd')
        self.SoundDet_s = self.Memory.subscriber("SoundDetected")
        self.SoundDet_s.signal.connect(self.callback_sound_det)
        # 初始化关键字
        self.action = ["go to", "look for", "search", "Navigate"]
        # self.answer = ["answer" ]
        self.place = ["bathroom", "kitchen", "party room", "dining room", "living room", "bedroom"]
        self.person_name = ["James", "Alex","Ryan","John","Eric","Adam","Carter","Jack",
                            "David","Tyler","Lily","Mary","Anna","Zoe","Sara","Sofia",
                            "Faith","Julia","Paige","Jessica", "jams", "job", "join",
                            "games", "Brian", "fight on", "out her", "barry", "finally",
                            "lady", "lilly", "marymy", "Sarah", "Sawyer", "Sophia", "Soviet",
                            "face", "page", "check", "cheque", "addicts"]
        self.object_name = ["grapejuice", "cola", "biscuit", "apple", "soap", "beer", "icetea",
                            "chips", "bread", "tray", "plate", "basket", "milk tea", "cup",
                            "pear", "orange", "banana"]
        self.current_action = "none"
        self.current_place = "none"
        self.current_item = "none"
        self.current_person_name = "none"
        self.current_content = "none"
        # 订阅相机
        # 当前时间戳（订阅相机的名字，每个只能使用6次）
        ticks = time.time()
        # 0代表top相机 最后一个参数是fps
        self.VideoDev.subscribeCamera(str(ticks), 0, 2, 11, 40)
        # 设置dialog语言
        self.Dialog.setLanguage("English")
        # 加载pepper电脑里的topic
        self.Topic_path = '/home/nao/top/competetion_enu.top'
        # 以utf-8的格式编码
        self.Topic_path = self.Topic_path.decode('utf-8')
        self.Topic_name = self.Dialog.loadTopic(self.Topic_path.encode('utf-8'))
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        # self.Motion.setTangentialSecurityDistance(.05)
        # self.Motion.setOrthogonalSecurityDistance(.1)
        # Beep 音量
        self.beep_volume = 70
        self.head_fix = True
        self.question_num = 0
        self.angle = -.3
        # 设置初始化头的位置 走路的时候固定头的位置
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., -0.25], .05)
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 80.0)
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)
        # 调用成员函数
        self.start_head_fix()
        self.get_init_pose()
        # self.set_volume(.7)

    def __del__(self):
        print ('\033[0;32m [Kamerider I] Shutting Down...... \033[0m')

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


    def amcl_callback(self, msg):
        self.car_pose = msg

    def get_init_pose(self):
        curr_pos = PoseStamped()
        position = MoveBaseGoal()
        position.target_pose.header.frame_id = '/map'
        position.target_pose.header.stamp = curr_pos.header.stamp
        position.target_pose.header.seq = curr_pos.header.seq
        position.target_pose.pose.position.x = self.car_pose.pose.pose.position.x
        position.target_pose.pose.position.y = self.car_pose.pose.pose.position.y
        position.target_pose.pose.position.z = self.car_pose.pose.pose.position.z
        position.target_pose.pose.orientation.x = self.car_pose.pose.pose.orientation.x
        position.target_pose.pose.orientation.y = self.car_pose.pose.pose.orientation.y
        position.target_pose.pose.orientation.z = self.car_pose.pose.pose.orientation.z
        position.target_pose.pose.orientation.w = self.car_pose.pose.pose.orientation.w
        self.point_init = position

    def start_recording(self, reset=False, base_duration=3, withBeep=True):
        self.if_need_record = True
        if reset:
            self.kill_recording_thread()
            self.if_need_record = True
            self.AudioRec.stopMicrophonesRecording()
            self.record_time = time.time() + base_duration
        print "self.record_time is:  ", self.record_time
        if not self.thread_recording.is_alive():
            self.thread_recording = Thread(target=self.record_audio, args=(self.speech_hints, withBeep))
            self.thread_recording.daemon = False
            self.if_need_record = True
            self.thread_recording.start()
            self.thread_recording.join()
            print('\033[0;32m [Kamerider I] Start recording thread \033[0m')

    def load_waypoint(self, file_name):
        curr_pos = PoseStamped()
        f = open(file_name, 'r')
        sourceInLines = f.readlines()
        print sourceInLines
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

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while self.if_head_fix:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(5)

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            time.sleep(0.3)
            self.audio_terminate = False

    def record_audio(self, hints, withBeep = True):
        print "================================================"
        if self.if_need_record:
            # 亮灯
            self.Leds.on("MyGroup")
            if withBeep:
                # 参数 playSine(const int& frequence, const int& gain, const int& pan, const float& duration)
                self.AudioPla.playSine(1000, self.beep_volume, 1, .3)
                time.sleep(.5)
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

    def speech_recog(self):
        channels = [0, 0, 1, 0]
        self.AudioRec.startMicrophonesRecording(self.audio_path, "wav", 16000, channels)
        speech_recognition_text.speech_recog()

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

    def stop_motion(self):
        self.cancel_plan()
        self.set_velocity(0, 0, 0)

    def say(self, string):
        self.TextToSpe.say(string)

    def start(self):
        # self.Motion.moveTo(.5, 0, 0)
        self.TextToSpe.say("Dear operator.")
        self.TextToSpe.say("Please call my name pepper, before each question")
        self.TextToSpe.say("Please talk to me after you heard ")
        self.AudioPla.playSine(1000, self.beep_volume, 1, .3)
        time.sleep(.2)
        self.say("three")
        self.say("two")
        self.say("one")
        for i in range(3):
            # self.say("please tell me the command")
            self.start_recording(reset=True)
            self.analyze_content()
            if self.current_person_name == "none" and self.current_item == "none":
                self.say("sorry, I could not finish this mission")
                self.say("please ask me the next question")
                continue
            elif self.current_item != "none":
                self.say("ok, I will go to the " + self.current_place + ", find " + self.current_item)
                self.say("sorry, I could not finish this mission")
                self.say("please ask me the next question")
                continue
            else:
                self.say("ok, I will go to the " + self.current_place + ", find " + self.current_person_name + ", and answer a question")
                if self.current_place == "kitchen" or self.current_place == "party room":
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                    self.go_to_waypoint(self.point_dataset["point2"], "point2")
                    self.go_to_waypoint(self.point_dataset["point3"], "point3")
                    self.go_to_waypoint(self.point_dataset["point4"], "point4")
                    self.go_to_waypoint(self.point_dataset["party room"], "party room")
                    self.Motion.moveTo(1.2, -1.2, 0)
                elif self.current_place == "bedroom":
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                    self.go_to_waypoint(self.point_dataset["point2"], "point2")
                    self.go_to_waypoint(self.point_dataset["bedroom"], "bedroom")
                    self.Motion.moveTo(1.2, 0, 0)
                elif self.current_place == "dining room":
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                    self.go_to_waypoint(self.point_dataset["point2"], "point2")
                    self.go_to_waypoint(self.point_dataset["point3"], "point3")
                    self.go_to_waypoint(self.point_dataset["point5"], "point5")
                    # self.go_to_waypoint(self.point_dataset["point6"], "point6")
                    # self.go_to_waypoint(self.point_dataset["point7"], "point7")
                elif self.current_place == "living room":
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                # self.go_to_waypoint(self.point_dataset[self.current_place], self.current_place)
                self.angle = -.3
                self.if_head_fix = False
                self.face = face_dete.face_dete_control(self.session)
                self.face.start_face_dete()
                self.if_head_fix = True
                self.start_head_fix()
                self.say("dear operator, please say pepper before each question")
                self.say("now, please ask me the question")
                self.start_recording(reset=True)
                self.analyze_content_later()
                if self.current_place == "kitchen" or self.current_place == "party room":
                    self.go_to_waypoint(self.point_dataset["party room"], "party room")
                    self.go_to_waypoint(self.point_dataset["point4"], "point4")
                    self.go_to_waypoint(self.point_dataset["point3"], "point3")
                    self.go_to_waypoint(self.point_dataset["point2"], "point2")
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                    self.Motion.moveTo(1.2, -1.2, 0)
                elif self.current_place == "bedroom":
                    self.go_to_waypoint(self.point_dataset["bedroom"], "bedroom")
                    self.go_to_waypoint(self.point_dataset["point2"], "point2")
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                    self.Motion.moveTo(1.2, 0, 0)
                elif self.current_place == "dining room":
                    # self.go_to_waypoint(self.point_dataset["point7"], "point7")
                    # self.go_to_waypoint(self.point_dataset["point6"], "point6")
                    self.go_to_waypoint(self.point_dataset["point5"], "point5")
                    self.go_to_waypoint(self.point_dataset["point3"], "point3")
                    self.go_to_waypoint(self.point_dataset["point2"], "point2")
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
                elif self.current_place == "living room":
                    self.go_to_waypoint(self.point_dataset["point1"], "point1")
        if self.current_place == "living room" or self.current_place == "bedroom":
            self.go_to_waypoint(self.point_dataset["point2"], "point2")
            self.go_to_waypoint(self.point_dataset["point3"], "point3")
            self.go_to_waypoint(self.point_dataset["point5"], "point5")
            self.go_to_waypoint(self.point_dataset["point6"], "point6")
            self.go_to_waypoint(self.point_dataset["point7"], "point7")
        elif self.current_place == "party room" or self.current_place == "kitchen":
            self.go_to_waypoint(self.point_dataset["point4"], "point4")
            self.go_to_waypoint(self.point_dataset["point5"], "point5")
            self.go_to_waypoint(self.point_dataset["point6"], "point6")
            self.go_to_waypoint(self.point_dataset["point7"], "point7")
        elif self.current_place == "dining room":
            self.go_to_waypoint(self.point_dataset["point6"], "point6")
            self.go_to_waypoint(self.point_dataset["point7"], "point7")
        # self.go_to_waypoint(self.point_dataset["exit"], "exit")

    def go_to_waypoint(self, Point, destination, label="none"):  # Point代表目标点 destination代表目标点的文本 label
        self.angle = .1
        self.nav_as.send_goal(Point)
        self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            if count_time == 3:
                self.map_clear_srv()
                count_time = 0
        self.TextToSpe.say("I have arrived at " + destination)
        if label == "none":
            return

    def show_person_image(self, name):
        # cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        # os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImage("http://198.18.0.1/apps/boot-config/" + name)

    def analyze_content_later(self):
        if re.search("china", self.recog_result)!= None and re.search("capital", self.recog_result)!= None:
            self.say("I heared what is the capital of china, the answer is beijing")
            self.show_person_image("1.jpg")
        elif re.search("hours", self.recog_result) != None and re.search("day", self.recog_result) != None:
            self.say("I heared how many hours in a day, the answer is twenty four")
            self.show_person_image("2.jpg")
        elif re.search("seasons", self.recog_result) != None and re.search("year", self.recog_result) != None:
            self.say("I heared how many season are there in one year, the answer is four")
            self.show_person_image("3.jpg")
        elif re.search("seconds", self.recog_result) != None and re.search("minute", self.recog_result) != None:
            self.say("I heared how many seconds in one minute, the answer is sixty")
            self.show_person_image("4.jpg")
        elif re.search("biggest", self.recog_result) != None and re.search("china", self.recog_result) != None:
            self.say("I heared what is the biggest province of china, the answer is xinjiang")
            self.show_person_image("5.jpg")
        elif re.search("china", self.recog_result) != None and re.search("large", self.recog_result) != None:
            self.say("I heared how large is the area of china, the answer is 960 square kilometers")
            self.show_person_image("6.jpg")
        elif re.search("biggest", self.recog_result) != None and re.search("island", self.recog_result) != None:
            self.say("I heared what is the word biggest island, the answer is Greenland")
            self.show_person_image("7.jpg")
        elif re.search("national", self.recog_result) != None and re.search("animal", self.recog_result) != None:
            self.say("I heared What is China's national animal, the answer is panda")
            self.show_person_image("8.jpg")
        elif re.search("president", self.recog_result) != None and re.search("first", self.recog_result) != None:
            self.say("I heared Who was the first president of the USA, the answer is George Washington")
            self.show_person_image("9.jpg")
        elif re.search("how many", self.recog_result) != None and re.search("children", self.recog_result) != None:
            self.say("I heared How many children did Queen Victoria have, the answer is Nine children")
            self.show_person_image("10.jpg")
        elif re.search("name", self.recog_result) != None and re.search("new york", self.recog_result) != None:
            self.say("I heared What was the former name of New York, the answer is New Amsterdam")
            self.show_person_image("11.jpg")
        else:
            self.say("sorry, I could not answer your question")

    def analyze_content(self):
        self.current_person_name = "none"
        for i in range(len(self.person_name)):
            if re.search(self.person_name[i].lower(), self.recog_result) != None:
                self.question_num += 1
                self.current_person_name = self.person_name[i]
        self.current_place = "none"
        for i in range(len(self.place)):
            if re.search(self.place[i].lower(), self.recog_result) != None:
                self.current_place = self.place[i]
                if self.current_place == "bathroom":
                    self.current_place = "bedroom"
        self.current_item = "none"
        for i in range(len(self.object_name)):
            if re.search(self.object_name[i].lower(), self.recog_result) != None:
                self.current_item = self.object_name[i]

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
                if command == 'st':
                    self.start()
                elif command == 'w':
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
                elif command == 'st':
                    self.start()
                elif command == 'c':
                    break
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"
def main():
    params = {
        'ip' : "192.168.43.30",
        'port' : 9559
    }
    pepper_gpsr = gpsr(params)
    pepper_gpsr.keyboard_control()

if __name__ == '__main__':
    main()
