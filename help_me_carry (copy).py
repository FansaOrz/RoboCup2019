#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import re
import sys
import time
import rospy
import atexit
import thread
import _thread
import datetime
import actionlib
from json import dumps
from .follow import pepper_follow
from .face_recog import face_recog
from .speech_recog import speech_recognition_text
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
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
            session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        # 需要使用的naoqi api
        self.Memory = self.session.service("ALMemory")
        self.Dialog = self.session.service("ALDialog")
        self.Motion = self.session.service("ALMotion")
        self.Tracker = self.session.service("ALTracker")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.FaceDet = self.session.service("ALFaceDetection")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.AnimatedSpe = self.session.service("ALAnimatedSpeech")
        self.FaceCha = self.session.service("ALFaceCharacteristics")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        # 停止录音
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;32;40m\t[Kamerider W] : You don't need stop record\033[0m")
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
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .3)
        self.SoundDet.subscribe('sd')
        self.SoundDet = self.Memory.subscriber("SoundDetected")
        self.SoundDet.signal.connect(self.callback_sound_det)
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
        print('\033[0;32m [Kamerider I] Help me Carry Class initialized! \033[0m')
        # 调用成员函数
        self.start_head_fix()
        self.set_volume(.7)

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')

    def start_head_fix(self):
        arg = tuple([1])
        self.state = True
        self.head_fix = True
        _thread.start_new_thread(self.head_fix_thread(), args=arg)

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

    def analyze_content(self):
        for i in range(len(self.start)):
            if re.search(self.start[i], self.recog_result) != None:
                print('\033[0;32m [Kamerider I] start following the operator \033[0m')
                # start follow function
                pepper_follow.follow_me(self.session)
                return
        for i in range(len(self.go_back)):
            if re.search(self.go_back[i], self.recog_result) != None:
                print('\033[0;32m [Kamerider I] Start navigating to ' + self.go_back[i] + '  \033[0m')
                # navigation function

                return
        for i in range(len(self.stop)):
            if re.search(self.stop[i], self.recog_result) != None:
                print('\033[0;32m [Kamerider I] Stop following the person  \033[0m')
                # follow enable set to false
                pepper_follow.follow_me.stop_following()
                return

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
        print('\033[0;32m [Kamerider I] Strat analyzing... \033[0m')
        self.analyze_content()

    def head_fix_thread(self):
        while self.head_fix:
            self.Motion.setStiffnesses("head", 1.0)
            self.Motion.setAngles("Head", [0., self.angle], .05)
            time.sleep(2)

    def set_volume(self, volume):
        self.TextToSpe.setParameter(volume)


def main():
    params = {
        'ip' : "127.0.0.1",
        'port' : 36057,
        'rgb_topic' : 'pepper_robot/camera/front/image_raw'
    }
    help_me_carry(params)

if __name__ == "__main__":
    main()
