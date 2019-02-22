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
from speech_recog import speech_recognition_text
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseStamped
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
        self.AnimatedSpe = self.session.service("ALAnimatedSpeech")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.SoundDet = self.session.service("ALSoundDetection")

        # 停止录音
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;32;40m\t[Kamerider W] : You don't need stop record\033[0m")
        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器thread.
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        # 声明一些变量
        self.angle = -.33
        self.if_ask_time = False
        self.if_need_record = False
        self.point_dataset = self.load_waypoint("waypoints.txt")
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
        self.start = ["follow", "following", "start", "follow me"]
        self.stop = ["stop", "here is the car"]
        self.go_back = ["bathroom", "living room", "bedroom", "kitchen", "toilet"]
        # 订阅相机
        # 当前时间戳（订阅相机的名字，每个只能使用6次）
        ticks = time.time()

        # 0代表top相机 最后一个参数是fps
        self.VideoDev.subscribeCamera(str(ticks), 0, 2, 11, 40)
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
        # follow me function
        self.pepper_follow_me = pepper_follow.follow_me(self.session)
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .7)
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
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def start_head_fix(self):
        arg = tuple([1])
        self.state = True
        self.head_fix = True
        thread.start_new_thread(self.head_fix_thread, arg)

    def callback_sound_det(self, msg):
        print ('\033[0;32m [Kamerider I] Sound detected (In callback function) \033[0m')
        ox = 0
        for i in range(len(msg)):
            if msg[i][1] == 1:
                ox = 1
        if ox == 1 and self.enable_speech_recog:
            self.record_time = time.time() + self.record_delay
            print "self.record_time += 2s ... ", self.record_time
            if not self.thread_recording.is_alive():
                self.start_recording(reset=True)
            while self.recog_result == "None":
                time.sleep(1)
                continue
            self.analyze_content()
        else:
            return None

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

    def analyze_content(self):
        for i in range(len(self.start)):
            if re.search(self.start[i].lower(), self.recog_result) != None:
                self.recog_result = "None"
                self.say("I will start following you")
                print('\033[0;32m [Kamerider I] start following the operator \033[0m')
                self.thread_recording.join()
                # start follow function
                self.pepper_follow_me.start_follow()
                return
        for i in range(len(self.go_back)):
            if re.search(self.go_back[i], self.recog_result) != None:
                self.recog_result = "None"
                print('\033[0;32m [Kamerider I] Start navigating to ' + self.go_back[i] + '  \033[0m')
                self.kill_recording_thread()
                # navigation function
                self.say("I'm going to the kitchen")
                self.go_to_waypoint(self.point_dataset[self.go_back[i]], self.go_back[i], label="back")
                self.face_dete()
                return
        for i in range(len(self.stop)):
            if re.search(self.stop[i], self.recog_result) != None:
                self.recog_result = "None"
                self.say("I will stop following you")
                print('\033[0;32m [Kamerider I] Stop following the person  \033[0m')
                self.thread_recording.join()
                self.pepper_follow_me.stop_follow()
                return

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            self.if_need_record = False

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

    def head_fix_thread(self, arg):
        while self.head_fix:
            self.Motion.setStiffnesses("head", 1.0)
            self.Motion.setAngles("Head", [0., self.angle], .05)
            time.sleep(2)

    def show_person_image(self):
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImage("http://198.18.0.1/apps/boot-config/person_image.png")

    def face_dete(self):
        # face detection函数
        self.face = face_dete.face_dete_control(self.session)
        self.face.start_face_dete()

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

    def go_to_waypoint(self, Point, destination, label):
        self.angle = .1
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
                self.say(sentence)
                sentence = "Excuse me, I need to go to the " + destination + ", Please let me go"
                self.say(sentence)
                sentence = "Thank you"
                time.sleep(2)
                self.say(sentence)
                time.sleep(2)
                self.if_ask_time = False
                self.map_clear_srv()
                self.nav_as.send_goal(Point)
                # 每隔4s清除一次local map
            if count_time == 3:
                self.map_clear_srv()
                count_time = 0
        if label == "back":
            print('\033[0;32m [Kamerider I] I have arrived at ' + destination + ', start looking for people \033[0m')
            self.say("I have arrived at " + destination)
            # find person function

    def say(self, text):
        self.TextToSpe.say(text)

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
                elif command == 'fd':
                    self.face_dete()
                elif command == 'sr':
                    self.start_record()
                elif command == 'c':
                    self.set_velocity(0, 0, 0); break
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"

def main():
    params = {
        'ip' : "192.168.3.18",
        'port' : 9559,
        'rgb_topic' : 'pepper_robot/camera/front/image_raw'
    }
    help_me_carry(params)

if __name__ == "__main__":
    main()
