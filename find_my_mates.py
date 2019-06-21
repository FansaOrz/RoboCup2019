#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import re
import cv2
import sys
import dlib
import time
import rospy
import thread
import atexit
import actionlib
import numpy as np
from threading import Thread
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID
from gender_predict import face_feature
from gender_predict import body_feature
from speech_recog import baidu_recognition_text
from object_detection.darknet import object_detection
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

#TODO 把人的信息都写到image上，然后在回去的时候展示照片
#TODO
class find_my_mate():

    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        rospy.init_node("find_my_mate")
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        self.car_pose = MoveBaseGoal()
        # naoqi API
        self.Leds = self.session.service("ALLeds")
        self.Memory = self.session.service("ALMemory")
        self.Dialog = self.session.service("ALDialog")
        self.Motion = self.session.service("ALMotion")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.PhotoCap = self.session.service("ALPhotoCapture")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        # stop recording
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;33m\t[Kamerider W]ALFaceCharacteristics : You don't need stop record\033[0m")
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
        self.Motion.setTangentialSecurityDistance(.05)
        self.Motion.setOrthogonalSecurityDistance(.1)
        #    LED的group
        self.led_name = ["Face/Led/Green/Right/0Deg/Actuator/Value", "Face/Led/Green/Right/45Deg/Actuator/Value",
                         "Face/Led/Green/Right/90Deg/Actuator/Value", "Face/Led/Green/Right/135Deg/Actuator/Value",
                         "Face/Led/Green/Right/180Deg/Actuator/Value", "Face/Led/Green/Right/225Deg/Actuator/Value",
                         "Face/Led/Green/Right/270Deg/Actuator/Value", "Face/Led/Green/Right/315Deg/Actuator/Value",
                         "Face/Led/Green/Left/0Deg/Actuator/Value", "Face/Led/Green/Left/45Deg/Actuator/Value",
                         "Face/Led/Green/Left/90Deg/Actuator/Value", "Face/Led/Green/Left/135Deg/Actuator/Value",
                         "Face/Led/Green/Left/180Deg/Actuator/Value", "Face/Led/Green/Left/225Deg/Actuator/Value",
                         "Face/Led/Green/Left/270Deg/Actuator/Value", "Face/Led/Green/Left/315Deg/Actuator/Value"]
        self.Leds.createGroup("MyGroup", self.led_name)
        # 声明一些变量
        # amcl_pose话题的订阅器
        self.amcl_pose_sb = None
        self.position_result = []
        self.get_image_switch = True
        self.to_find_person_name = []
        self.current_drink_name = []
        # 保存当前对话人的年龄性别
        self.gender = "none"
        self.age = "none"
        # 存储每次语音识别的类别
        self.type = "none"
        # 储存每次语音识别的结果
        self.audio_recog_result = "none"
        # 保存当前人的肤色，衣服颜色和类别
        self.lower_color = "none"
        self.lower_wear = "none"
        self.upper_color = "none"
        self.upper_wear = "none"
        self.skin_color  = "none"
        self.predefined_position = \
            {
                'table':[0.8722, -0.3192],
                "sofa1":[2.629, -0.37998],
                "tea table":[2.8597, -1.0975],
                "sofa2":[3.82847, -1.03030],
                "chair":[4.66809, -1.6237],
                "bed":[3.14724, -3.78763],
                "TV":[1.34616, -2.70090]
            }
        # 保存当前人旁边有什么东西
        self.position = "none"
        self.old_drink = "none"
        self.positive_list = ["yes", "of course", "we do"]
        self.negative_list = ["no", "sorry", "we don't", "regret"]
        self.name_list = ["Alex","Charlie","Elizabeth","Francis","James","Jennifer","John","Linda","Michael","Mary","Robert","Patricia","Robin","Skyler","William"]
        self.drink_list = ["ice tea","beer","coke","milk","orange juice","toothpaste","cookie","shampoo", "chips", "green tea"]
        self.stop_list = ["stop", "go back"]
        self.angle = -.1
        self.head_fix = True
        self.if_need_record = False
        self.point_dataset = self.load_waypoint("waypoints_help.txt")
        # 人脸识别
        self.detector = dlib.get_frontal_face_detector()
        # 物体识别
        self.object_detection = object_detection()
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        # 初始化平板
        self.TabletSer.hideImage()
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')
        # 设置dialog语言
        self.Dialog.setLanguage("English")
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "00"
        self.RobotPos.goToPosture("StandInit", .2)
        # Beep 音量
        self.beep_volume = 70
        # 脸最大的人的中心
        self.center = 0
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 75.0)
        # 初始化录音
        self.record_delay = 2.5
        self.speech_hints = []
        self.enable_speech_recog = True
        # 1代表最灵敏
        self.SoundDet.setParameter("Sensitivity", .4)
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
        cv2.destroyAllWindows()

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def show_person_image(self):
        cmd = "sshpass -p kurakura326 scp ./person.jpg nao@" + str(self.ip) + ":~/.local/share/PackageManager/apps/boot-config/html"
        os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImageNoCache("http://198.18.0.1/apps/boot-config/person.jpg")

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while True:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

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

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

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

    def cal_distence(self, position):
        min = 1000
        # 寻找距离最近的点
        for i in self.predefined_position.keys():
            dist = (self.predefined_position[i][0] - position[0])*(self.predefined_position[i][0] - position[0]) + \
                   (self.predefined_position[i][1] - position[1])*(self.predefined_position[i][1] - position[1])
            if dist < min:
                min = dist
                self.position = i
        # 防止返回sofa1和sofa2
        if self.position == "sofa1" or self.position == "sofa2":
            self.position = "sofa"

    def analyze_content(self):
        result = []
        # 记录是否找到人
        person_found = False
        # 获取要找的人姓名
        for i in range(len(self.name_list)):
            if re.search(self.name_list[i].lower(), self.recog_result) != None:
                print "found one person:=", self.name_list[i].lower()
                person_found = True
                # 记下当前人的名字
                result.append(self.name_list[i])

        self.recog_result = "00"
        if person_found:
            self.type = "NAME"
            self.audio_recog_result = result
            return
        else:
            # print "5", self.recog_result
            self.say("sorry, please tell me again")
            self.start_recording(reset=True)
            self.analyze_content()

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
            os.mkdir('./audio_record', 0o755)
        # 复制录下的音频到自己的电脑上
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/audio/recog.wav ./audio_record"
        os.system(cmd)
        print('\033[0;32m [Kamerider I] Record ended start recognizing \033[0m')
        self.recog_result = baidu_recognition_text.main("./audio_record/recog.wav").lower()
        print "===============", self.recog_result

    def take_picture(self):
        self.PhotoCap.takePictures(3, '/home/nao/picture', 'party')
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/picture/party_0.jpg ./person_image"
        os.system(cmd)
        self.gender, self.age, self.skin_color = face_feature.gender("./person_image/party_0.jpg")
        if self.gender == "none":
            self.say("please look at my eyes")
            self.take_picture()
        else:
            return self.gender

    def amcl_pose_cb(self, msg):
        position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.cal_distence(position)
        self.amcl_pose_sb.unregister()

    def start(self):
        # 抬头
        self.angle = -.5
        self.Motion.setAngles("Head", [0., self.angle], .2)
        self.TextToSpe.say("Dear operator.")
        # self.TextToSpe.say("Please call my name pepper, before each question")
        self.TextToSpe.say("Please talk to me after my eyes' color turn to white ")
        time.sleep(1)
        # 开始和operator对话
        self.start_recording(reset=True)
        self.analyze_content()
        self.to_find_person_name = self.audio_recog_result
        self.say("ok, I will find ")
        for i in range(len(self.to_find_person_name)):
            self.say(str(self.to_find_person_name[i]) + " ")
        # 走进屋子，开始找人
        self.go_to_waypoint(self.point_dataset["point2"], "point2", "first")

        person_found_num = 0
        while person_found_num != 1:
            self.find_person()
            self.amcl_pose_sb = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb)
            if self.dialog_with_people() == "succe":
                person_found_num += 1
                self.Motion.moveTo(0, 0, 0.52)
            else:
                continue
        # 回到operator那里
        self.go_to_waypoint(self.point_dataset["point1"], "point2", "first")
        for i in range(len(self.position_result)):
            self.say(self.position_result[i])

    def stop_motion(self):
        self.goal_cancel_pub.publish(GoalID())
        self.set_velocity(0, 0, 0)

    def get_car_position(self):
        curr_pos = PoseStamped()
        car_position = MoveBaseGoal()
        car_position.target_pose.header.frame_id = '/map'
        car_position.target_pose.header.stamp = curr_pos.header.stamp
        car_position.target_pose.header.seq = curr_pos.header.seq
        car_position.target_pose.pose.position.x = self.car_pose.target_pose.pose.position.x
        car_position.target_pose.pose.position.y = self.car_pose.target_pose.pose.position.y
        car_position.target_pose.pose.position.z = self.car_pose.target_pose.pose.position.z
        car_position.target_pose.pose.orientation.x = self.car_pose.target_pose.pose.orientation.x
        car_position.target_pose.pose.orientation.y = self.car_pose.target_pose.pose.orientation.y
        car_position.target_pose.pose.orientation.z = self.car_pose.target_pose.pose.orientation.z
        car_position.target_pose.pose.orientation.w = self.car_pose.target_pose.pose.orientation.w
        self.car_pose = car_position

    def dialog_with_people(self):
        # 抬头
        self.angle = -.5
        self.Motion.setAngles("Head", [0., self.angle], .1)
        time.sleep(1)
        # 拍照识别性别
        self.take_picture()
        self.show_person_image()
        self.say("Hi, my name is pepper, what is your name?")
        self.start_recording(reset=True)
        self.analyze_content()
        if self.type == "NAME":
            for i in range(len(self.to_find_person_name)):
                print "self.to_find_person_name", self.to_find_person_name
                # 如果这个人在要找的list里面
                print "========================match========================"
                if re.search(self.audio_recog_result[0].lower(), self.to_find_person_name[i].lower()) != None:
                    self.say("ok, I have remembered your position")
                    if self.gender == "male":
                        sentence = "the person named " + self.to_find_person_name[i].lower() + " is " + self.gender + ". And he is wearing " + self.upper_color + " " + self.upper_wear + ". And he is next to " + self.position
                    else:
                        sentence = "the person named " + self.to_find_person_name[i].lower() + " is " + self.gender + ". And she is wearing " + self.upper_color + " " + self.upper_wear + ". And she is next to " + self.position
                    print "---------------------------------------------------result---------------------------------------------------"
                    print sentence
                    self.position_result.append(sentence)
                    # 清空上一个人的数据
                    self.upper_color = self.upper_wear = self.lower_wear = self.lower_color = self.position = self.gender = 'none'
                    # 找到了我们要找的人，返回“succe”
                    return "succe"
            # 最终没有找到我们要的人，返回“wrong”
            return "wrong"

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

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            self.if_need_record = False

    def find_person(self):
        first_detect_face = True
        self.angle = -.35
        AL_kQVGA = 1
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        width = 320
        height = 240
        image = np.zeros((height, width, 3), np.uint8)

        if_turn_finished = False

        while self.get_image_switch:
            result = self.VideoDev.getImageRemote(nameId)
            if result == None:
                print 'cannot capture.'
            elif result[6] == None:
                print 'no image data string.'
            else:
                values = map(ord, list(str(bytearray(result[6]))))
                i = 0
                for y in range(0, height):
                    for x in range(0, width):
                        image.itemset((y, x, 0), values[i + 0])
                        image.itemset((y, x, 1), values[i + 1])
                        image.itemset((y, x, 2), values[i + 2])
                        i += 3
                # print image
                cv2.imshow("pepper-top-camera-640*480px", image)
                cv2.waitKey(1)
                # dlib检测人脸
                rects = self.detector(image, 2)
                # 检测到人脸
                if len(rects) != 0:
                    # 转向完成，开始接近人
                    if if_turn_finished:
                        # 第一次看到人，就进行一次特征识别
                        while (self.upper_wear and self.upper_color == "none"):
                            image_name = "/home/fansa/Src/pepper_example/RoboCup2019/person_body.jpg"
                            cv2.imwrite(image_name, image)
                            # self.position = self.object_detection.main(image_name)
                            _, _, self.upper_color, self.upper_wear = body_feature.feature(image_name)
                        # 接近脸最大的那个人
                        image_max = 0
                        for rect in rects:
                            cv2.rectangle(image, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255),
                                          2, 8)
                            cv2.imshow("yess", image)
                            cv2.imwrite("./person.jpg", image)
                            if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                                image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())

                        print  float(image_max) / float(320 * 410)
                        if float(image_max) / float(320 * 410) > .018:
                            self.get_image_switch = False
                            self.if_stop = True
                        else:
                            self.Motion.moveTo(0.2, 0, 0)
                        cv2.waitKey(1)
                    # 开始转向人
                    else:
                        image_max = 0
                        for rect in rects:
                            cv2.rectangle(image, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255),
                                          2, 8)
                            cv2.imshow("yess", image)
                            cv2.imwrite("./person.jpg", image)
                            if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                                image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                                self.center = (rect.left() + rect.right()) / 2
                        Error_dist = width / 2 - self.center
                        if abs(Error_dist) <= 10:
                            if_turn_finished = True
                            continue
                        self.Motion.moveTo(0, 0, 0.002*Error_dist)
                        cv2.waitKey(1)
                # 没有检测到人脸就旋转
                else:
                    self.Motion.moveTo(0, 0, 0.52)
        return "succe"

    def go_to_waypoint(self, Point, destination, label="none"): # Point代表目标点 destination代表目标点的文本 label
        self.angle = .3
        self.nav_as.send_goal(Point)
        self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            if count_time == 8:
                self.map_clear_srv()
                count_time = 0
        # self.TextToSpe.say("I have arrived at " + destination)
        if label == "none":
            return

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
                elif command == 'get':
                    self.Motion.moveTo(1,-1,0)
                elif command == 'c':
                    break
                elif command == 'dwp':
                    self.start()
                elif command == 'tp':
                    self.take_picture()
                else:
                    print("Invalid Command!")
            except Exception as e:
                print e


if __name__ == "__main__":
    params = {
        'ip': "192.168.3.93",
        'port': 9559
    }
    find_my_mate(params)