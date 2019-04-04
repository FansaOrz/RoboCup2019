#!/usr/bin/env python3
# -*- encoding: UTF-8 -*-


import os
import re
import qi
import sys
import time
import thread
import atexit
from threading import Thread
from speech_recog import speech_recognition_text
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped

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
        except RuntimeError:
            # 输出红字
            print("\033[0;30;40m\t[Kamerider E] : connection Error!!\033[0m")
            sys.exit(1)

        # 需要的naoqi的服务
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
        # 停止录音
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;32;40m\t[Kamerider W] : You don't need stop record\033[0m")
        # topic对话的回调函数

        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器
        # 声明一些变量
        self.point_dataset = self.load_waypoint("waypoints.txt")
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
        self.place = ["bathroom cabinet", "cabinet", "living room", "kitchen", "bedroom",
                      "bathroom", "bar", "cupboard", "dining table", "corridor", "stove",
                      "sofa", "shower", "toilet", "washing machine", "towel rail", "washbasin",
                      "bed", "bidet", "wardrobe"]
        self.item = ["water", "cookies", "soap", "pasta", "milk", "pringles", "drinks", "food"
                     "coke", "beer", "noodles"]
        self.action = ["tell me how many", "tell me the name", "what day is today", "someone", "the day of the month",
                       "tell what day is tomorrow", "tell me how many", "answer a question", "tell the time",
                       "place", "locate a person"]
        self.name = ["Jamie", "Robin", "Taylor", "Jordan", "Tracy", "me", "Morgan"]
        self.go_first = ["get", "locate", "find"]
        self.get_first = ["put", ""]
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

    def start_recording(self, reset=False, base_duration=3, withBeep=True):
        if reset:
            self.kill_recording_thread()
            self.AudioRec.stopMicrophonesRecording()
            self.record_time = time.time() + base_duration
        if not self.thread_recording.is_alive():
            self.thread_recording = Thread(target=self.record_audio, args=(self.speech_hints, withBeep))
            self.thread_recording.daemon = False
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
        self.state = True
        self.head_fix = True
        _thread.start_new_thread(self.head_fix_thread(), args=arg)

    def head_fix_thread(self):
        while self.head_fix:
            self.Motion.setStiffnesses("head", 1.0)
            self.Motion.setAngles("Head", [0., self.angle], .05)
            time.sleep(2)

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            time.sleep(0.3)
            self.audio_terminate = False

    def record_audio(self, hints, withBeep = True):
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
        self.AudioRec.stopMicrophonesRecording()
        self.AudioRec.recording_ended = True

        if not os.path.exists('./audio_record'):
            # TODO
            os.mkdir('./audio_record', 0o755)
        # 复制录下的音频到自己的电脑上
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/audio/recog.wav ./audio_record"
        os.system(cmd)
        print('\033[0;32m [Kamerider I] Record ended start recognizing \033[0m')
        self.recog_result = speech_recognition_text.speech_recog("./audio_record/recog.wav")

    def speech_recog(self):
        channels = [0, 0, 1, 0]
        self.AudioRec.startMicrophonesRecording(self.audio_path, "wav", 16000, channels)
        speech_recognition_text.speech_recog()

    def set_volume(self, volume):
        self.TextToSpe.setParameter(volume)
def main():
    params = {
        'ip' : "192.168.3.222",
        'port' : 9559
    }
    pepper_gpsp = gpsr(params)

if __name__ == '__main__':
    main()
