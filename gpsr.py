#!/usr/bin/env python3
# -*- encoding: UTF-8 -*-


import os
import re
import qi
import sys
import time
import rospy
import thread
import atexit
from threading import Thread
from speech_recog import speech_recognition_text
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

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
            self.BasicAwa.setEnable(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 初始化平板
        self.TabletSer.cleanWebview()
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .3)
        self.SoundDet.subscribe('sd')
        self.SoundDet_s = self.Memory.subscriber("SoundDetected")
        self.SoundDet_s.signal.connect(self.callback_sound_det)
        # 初始化关键字
        self.action = ["say", "go to", "look for", "answer", "tell", "Navigate", "find"]
        self.place = ["baby chair", "corridor", "tv couch", "bathroom", "toilet",
                      "dining room", "cabinet", "the chair", "coffee table", "living room",
                      "bedroom", "shower", "desk", "stove", "washbasin", "armchair", "bathtub"
                      , "tv couch"]
        self.person_name = ["hayden", "me", "someone", "Robin", "Peyton", "Tracy", "Michael",
                            "Jamie", "Alex"]
        self.content = ["week", "name of person", "time", "month" , "country"
                        , "what day is today", "joke", "yourself", "affiliation"]
        self.current_action = []
        self.current_place = []
        self.current_person_name = []
        self.current_content = []
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
        # Beep 音量
        self.beep_volume = 70
        self.head_fix = True

        self.angle = -0.0
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
        self.set_volume(.7)

    def __del__(self):
        print ('\033[0;32m [Kamerider I] Shutting Down...... \033[0m')

    def callback_sound_det(self, msg):
        print ('\033[0;32m [Kamerider I] Sound detected (In callback function) \033[0m')
        ox = 0
        for i in range(len(msg)):
            if msg[i][1] == 1:
                ox = 1
        if ox == 1 and self.enable_speech_recog:
            self.record_time = time.time() + self.record_delay
            if not self.thread_recording.is_alive():
                self.start_recording(reset=True)
        else:
            return None

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
        while True:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

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
            self.recog_result = speech_recognition_text.main("./audio_record/recog.wav").lower()

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
        for i in range(3):
            self.say("hello, I am pepper")
            # self.say("please tell me the command")
            self.start_recording(reset=True)
            self.analyze_content()
            self.say(self.recog_result)
            # self.go_to_waypoint(self.point_dataset[self.current_place[0]], self.current_place[0])
            # self.face = face_dete.face_dete_control(self.session)
            # self.face.start_face_dete()
            # # 包含me要回去
            # for i in range(len(self.person_name)):
            #     if re.search("me", self.person_name[i]) != None:
            #         self.say("could you tell me your name?")
            #         self.start_recording(reset=True)
            #         self.analyze_content()
            #         self.go_to_waypoint(self.point_init)
            #         self.say("the name of person of " + self.current_place[0] + " is " + self.current_person_name[0])
            #         return
            # #"something about yourself", "affiliation"]
            # if self.current_content[0] == "week":
            #     self.say("hey, today is Friday")
            #     return
            # elif self.current_content[0] == "time":
            #     current_time = time.strftime('%H:%M', time.localtime(time.time())).split(":")
            #     sentence = "hey, The time is " + current_time[0] + " " + current_time[1]
            #     print('\033[0;32m [Kamerider I] ' + sentence + ' \033[0m')
            #     self.TextToSpe.say(sentence)
            #     return
            # elif self.current_content[0] == "month":
            #     self.say("hey, today is the ninteenth day of April")
            #     return
            # elif self.current_content[0] == "country":
            #     self.say("hey, the country ofour team is China")
            #     return
            # elif self.current_content[0] == "what day is today":
            #     self.say("hey, today is April ninteenth")
            #     return
            # elif self.current_content[0] == "joke":
            #     self.say("hey, I will tell you a joke")
            #     return
            # elif self.current_content[0] == "yourself":
            #     self.say("hey, My name is pepper. Nice to meet you")
            #     return
            # elif self.current_content[0] == "affiliation":
            #     self.say("hey, our team come from Nankai University, China, Tianjin")
            #     return



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

    def analyze_content(self):
        print
        self.current_place = []
        for i in range(len(self.place)):
            if re.search(self.place[i].lower(), self.recog_result) != None:
                for j in range(len(self.place)):
                    if re.search(self.place[j].lower(), self.recog_result) != None:
                        self.current_place.append(self.place[j].lower())
        self.current_action = []
        for i in range(len(self.action)):
            if re.search(self.action[i].lower(), self.recog_result) != None:
                for j in range(len(self.action)):
                    if re.search(self.action[j].lower(), self.recog_result) != None:
                        self.current_action.append(self.action[j].lower())
        self.current_person_name = []
        for i in range(len(self.person_name)):
            if re.search(self.person_name[i].lower(), self.recog_result) != None:
                for j in range(len(self.person_name)):
                    if re.search(self.person_name[j].lower(), self.recog_result) != None:
                        self.current_person_name.append(self.person_name[j].lower())
        self.current_content = []
        for i in range(len(self.content)):
            if re.search(self.content[i].lower(), self.recog_result) != None:
                for j in range(len(self.content)):
                    if re.search(self.content[j].lower(), self.recog_result) != None:
                        self.current_content.append(self.content[j].lower())
        if self.current_content == [] and self.current_action == [] and self.current_person_name == [] and self.current_place == []:
            self.TextToSpe.say("sorry, please tell me again")
            self.start_recording(reset=True)
            self.analyze_content()

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
        'ip' : "172.16.0.10",
        'port' : 9559
    }
    pepper_gpsr = gpsr(params)
    pepper_gpsr.keyboard_control()

if __name__ == '__main__':
    main()
