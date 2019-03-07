#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import time
import atexit
import thread
from gender_predict import gender
from threading import Thread


class speech_person_recog():

    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
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
        # 声明一些变量
        self.num_man = 0
        self.num_woman = 0
        self.angle = -.2
        self.if_need_record = False
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

    def record_audio(self, hints, withBeep = True):
        print "==================    print imuMsg #Gibt alle Sensorwerte in einer Art Tabelle aus
=============================="
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

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

    def turn_around(self):
        self.TextToSpe.say("I am going to turn around")
        self.Motion.move(0, 0, 1)
        duration = time.time() + 3.8
        while time.time() < duration:
            continue
        self.Motion.stopMove()
        self.take_picture()

    def take_picture(self):
        self.TextToSpe.say("I am going to take a picture")
        self.PhotoCap.takePictures(3, '/home/nao/picture', 'image')
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/picture/image_0.jpg ./person_image"
        os.system(cmd)
        self.num_man, self.num_woman = gender.gender_predict(self.session)
        #self.num_man, self.num_woman = image_predict.start()
        self.TextToSpe.say("There are " + str(self.num_woman + self.num_man) + "people")
        self.TextToSpe.say("the number of man is " + str(self.num_man))
        self.TextToSpe.say("the number of woman is " + str(self.num_woman))

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
        'ip': "192.168.3.18",
        'port': 9559
    }
    speech_person_recog(params)