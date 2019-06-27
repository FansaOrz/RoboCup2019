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
from tf import transformations
from actionlib_msgs.msg import GoalID
from gender_predict import face_feature
from gender_predict import body_feature
from speech_recog import baidu_recognition_text
# from object_detection.darknet import object_detection
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion

#TODO
class where_is_this():

    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        rospy.init_node("where_is_this")
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
        self.place = ["TV", "sofa", "table", "bed", "tea table"]
        self.place_description = {"TV":"you need to turn round first then go ahead. and the TV is on your right", "sofa":"you need go out first and then turn right. Sofa is in front of you",
                                  "table":"you need to turn round first then go ahead. and the table is on your left", "bed":"you need go out first and then turn right. Bed is in front of you",
                                  "tea table":"you need to turn right first then go ahead. and the tea table is on your left"}
        # amcl_pose话题的订阅器
        self.amcl_pose_sb = None
        self.position_result = []
        self.get_image_switch = True
        self.to_find_person_name = []
        self.current_drink_name = []
        # 存储每次语音识别的类别
        self.type = "none"
        # 储存每次语音识别的结果
        self.audio_recog_result = "none"
        self.predefined_position = {
                'table':[0.8722, -0.3192, 0, 0, 0.950170776273, 0.311729844444],
                "sofa1":[2.629, -0.37998, 0, 0, 0.435683814323, 0.900099779989],
                "tea table":[2.8597, -1.0975, 0, 0, 0.435683814323, 0.900099779989],
                "sofa2":[3.82847, -1.03030, 0, 0, 0.435683814323, 0.900099779989],
                "chair":[4.66809, -1.6237, 0, 0, 0.435683814323, 0.900099779989],
                "bed":[3.14724, -3.78763, 0, 0, -0.709890396828, 0.704312164094],
                "TV":[1.34616, -2.70090, 0, 0, -0.896545546577, 0.4429515582]
            }
        # 保存当前人旁边有什么东西
        self.old_drink = "none"
        self.angle = -.1
        self.person_num = 1
        # laser计算的数量
        self.laser_num = 0
        self.head_fix = True
        # 判断门是不是开着的
        self.if_door_open = False
        self.if_need_record = False
        self.point_dataset = self.load_waypoint("waypoints_help.txt")
        # 人脸识别
        self.detector = dlib.get_frontal_face_detector()
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 关闭AutonomousLife模式-
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
        self.show_image("instructions.png")
        self.keyboard_control()

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        cv2.destroyAllWindows()

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def laser_callback(self, msg):
        # 检测中间的laser的信息判断门是不是开着的
        sum = 0
        # 前面的laser的数据
        for i in range(23, 37):
            sum += msg.ranges[i]
        print "current_sum=:",sum
        # 大于阈值就认定门开了
        if sum > 20:
            self.if_door_open = True
            self.laser_sb.unregister()

    def show_image(self, image_name):
        # cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        # os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImage("http://198.18.0.1/apps/boot-config/" + str(image_name))

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while True:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

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
                while self.recog_result == "00":
                    time.sleep(1)
                    continue
            else:
                return None
        else:
            print ('\033[0;32m [Kamerider I] Sound detected, but we don\'t need to record this audio \033[0m')

    def analyze_content(self):
        result = []
        # 记录是不是找到了地点
        place_found = False

        # 获取要找的人姓名
        for i in range(len(self.place)):
            if re.search(self.place[i].lower(), self.recog_result) != None:
                print "found one place:=", self.place[i].lower()
                place_found = True
                # 记下当前地点的名字
                result.append(self.place[i])

        self.recog_result = "00"
        print len(result), "=========================="
        if place_found and len(result) == 1:
            self.type = "PLACE"
            self.audio_recog_result = result
            return

        self.TextToSpe.say("sorry, please tell me again")
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
        self.gender, self.age, self.skin_color = face_feature.gender("./person_image/party_0.jpg", self.upper_wear, self.upper_color, self.person_num)
        if self.gender == "none":
            self.TextToSpe.say("please look at my eyes")
            self.take_picture()
        else:
            return self.gender

    def add_name_to_img(self, img_name, position, person_name, posture):
        img = cv2.imread(img_name)
        cv2.putText(img, "Name:"+str(person_name), (160, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.putText(img, "Position:"+posture+" next to the "+str(position), (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imwrite(img_name, img)
        # 将当前人的信息传输到Pepper上
        cmd = "sshpass -p kurakura326 scp ./person_result" + str(self.person_num) + ".jpg nao@" + str(
            self.ip) + ":~/.local/share/PackageManager/apps/boot-config/html"
        os.system(cmd)

    def start(self):
        # 订阅laser的话题，判断门是不是开着的
        self.laser_sb = rospy.Subscriber('/pepper_robot/naoqi_driver/laser', LaserScan, self.laser_callback)
        print "start"
        while not self.if_door_open:
            time.sleep(.5)
            # print "waiting the door"
        self.TextToSpe.say("the door is open")
        # 走进屋子
        self.go_to_waypoint(self.point_dataset["point33"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point13"], "point2", "first")
        # 抬头
        self.angle = -.4
        self.Motion.setAngles("Head", [0., self.angle], .2)
        num = 0
        while num != 1:
            self.TextToSpe.say("Where do you want to go? Please tell me")
            self.start_recording(reset=True)
            self.analyze_content()
            self.navigate_to_place(self.audio_recog_result[0])
            num += 1

    def stop_motion(self):
        self.goal_cancel_pub.publish(GoalID())
        self.set_velocity(0, 0, 0)

    def navigate_to_place(self, place):
        self.TextToSpe.say("I know where is the " + place )
        self.TextToSpe.say(self.place_description[place])
        self.TextToSpe.say("please follow me")
        print place
        if place == "bed":
            self.go_to_waypoint(self.point_dataset["point31"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point7"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point5"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point4"], "point2", "first")
        elif place == "table":
            self.go_to_waypoint(self.point_dataset["point35"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point41"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point38"], "point2", "first")
            # self.go_to_waypoint(self.point_dataset["point4"], "point2", "first")
        elif place == "TV":
            self.go_to_waypoint(self.point_dataset["point34"], "point2", "first")
        elif place == "tea table":
            self.go_to_waypoint(self.point_dataset["point14"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point42"], "point2", "first")
        elif place == "sofa":
            self.go_to_waypoint(self.point_dataset["point31"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point7"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point5"], "point2", "first")
            self.go_to_waypoint(self.point_dataset["point2"], "point2", "first")
        self.TextToSpe.say("here is the " + place)
        # 抬头
        self.angle = -.4
        self.Motion.setAngles("Head", [0., self.angle], .2)
        self.go_to_waypoint(self.point_dataset["point2"], "point2", "first")

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

    def go_to_waypoint(self, Point, destination, label="none"): # Point代表目标点 destination代表目标点的文本 label
        # 设置头的角度
        self.angle = .3
        self.Motion.setAngles("Head", [0., self.angle], .2)
        # 发送目标点
        self.nav_as.send_goal(Point)
        # 清空局部地图
        self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            if count_time == 8:
                # self.map_clear_srv()
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
                elif command == 'fp':
                    self.find_person()
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
    where_is_this(params)