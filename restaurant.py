#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import re
import sys
import cv2
import time
import dlib
import rospy
import atexit
import thread
import actionlib
import numpy as np
from threading import Thread
from std_srvs.srv import Empty
from object_detection import waving_detection
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from tf import transformations
from speech_recog import baidu_recognition_text

class restaurant():
    def __init__(self, params):
        rospy.init_node("restaurant")
        atexit.register(self.__del__)
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        self.Leds = self.session.service("ALLeds")
        self.Memory = self.session.service("ALMemory")
        self.Motion = self.session.service("ALMotion")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.AutonomousLife = self.session.service("ALAutonomousLife")

        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;33m\t[Kamerider W] : You don't need stop record\033[0m")
        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # YOLO相关
        self.waving_detector = waving_detection.object_detection()
        self.detector = dlib.get_frontal_face_detector()
        self.get_image_switch = True
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        # 声明一些变量
        self.object = ["coke cole", "wine", "beer", "photo chips", "green tea", "water"]
        self.current_drink_name = []
        self.scan_msg = []
        self.if_save_switch = True
        self.scan_msg_time = 0
        self.angle = -.2
        self.f = open('./laser.txt', 'w')
        self.if_need_record = False
        self.head_fix = True
        self.bar_location = "none"
        self.point_dataset = []
        # 设置英语
        self.TextToSpe.setLanguage("English")
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

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')

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

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

    def approach_waving(self):
        self.angle = -.2
        # 记录旋转的次数
        turn_num = 3
        self.Motion.setAngles("Head", [0., self.angle], .2)
        AL_kQVGA = 2
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        # 后面调整角度时候用到
        current_top = current_bottom = current_left = current_right = 0
        width = 640
        height = 480
        image = np.zeros((height, width, 3), np.uint8)
        if_turn_finished = False
        face_detect_num = 0
        check_second_time = False
        waving_dete = True
        while self.get_image_switch:
            print "---------------------------------------self.angle", self.angle
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
                cv2.imshow("waving_detection", image)
                cv2.waitKey(1)
                cv2.imwrite("/home/fansa/Src/pepper_example/RoboCup2019/object_detection/data/waving_detection.jpg", image)
                # 当前是否要检测挥手
                if waving_dete:
                    if self.waving_detector.main("/home/fansa/Src/pepper_example/RoboCup2019/object_detection/data/waving_detection.jpg") == "none":
                        # 第二次还是没识别到，就转身
                        if check_second_time:
                            check_second_time = False
                            if turn_num != 6:
                                self.Motion.moveTo(0, 0, 0.52)
                                turn_num += 1
                            else:
                                self.Motion.moveTo(0, 0, 3.14)
                                turn_num = 0
                            continue
                        # 如果第一次没识别到，就再识别一次
                        else:
                            check_second_time = True
                            continue
                    # 只要识别到了就接近人
                    else:
                        waving_dete = False
                        check_second_time = False
                # dlib检测人脸
                rects = self.detector(image, 2)
                # 检测到人脸
                if len(rects) != 0:
                    if_first = True
                    # 转向完成，开始接近人
                    if if_turn_finished:
                        # 接近脸最大的那个人
                        image_max = 0
                        for rect in rects:
                            cv2.rectangle(image, (rect.left(), rect.top()), (rect.right(), rect.bottom()),
                                          (0, 0, 255), 2, 8)
                            cv2.imshow("yess", image)
                            cv2.imwrite("./restaurant_person.jpg", image)
                            if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                                image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                                current_bottom, current_top, current_left, current_right = rect.bottom(), rect.top(), rect.left(), rect.right()
                        print  float(image_max) / float(width * height)
                        # 判断是不是要停下来了
                        if float(image_max) / float(width * height) > .04:
                            self.set_velocity(0, 0, 0)
                            self.get_image_switch = False
                        else:
                            # 判断是佛需要抬头或者低头
                            height_center = (current_top + current_bottom) / 2
                            if height_center < height / 2:
                                print "upupupupupupupupupupupupupupup", current_bottom
                                self.angle -= .1
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                            elif height_center > 2 * height / 3:
                                print "downdowndowndowndowndowndowndown", current_top
                                self.angle += .1
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                            # 直走
                            self.set_velocity(0.15, 0, 0)
                            # 判断是否需要微调
                            center = (current_left + current_right) / 2
                            if abs(width / 2 - center) > width / 10:
                                print "inininininininininininnnininininini"
                                Error_dist_ = width / 2 - center
                                self.set_velocity(0, 0, 0.001 * Error_dist_)
                                time.sleep(2)
                                self.set_velocity(0.15, 0, 0)
                        cv2.waitKey(1)
                    # 开始转向人
                    else:
                        image_max = 0
                        for rect in rects:
                            cv2.rectangle(image, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255), 2, 8)
                            cv2.imshow("yess", image)
                            cv2.imwrite("./person.jpg", image)
                            if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                                image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                                self.center = (rect.left() + rect.right()) / 2
                        Error_dist = width / 2 - self.center
                        if abs(Error_dist) <= 10:
                            if_turn_finished = True
                            continue
                        self.Motion.moveTo(0, 0, 0.002 * Error_dist)
                        cv2.waitKey(1)
                # 没有检测到人脸就旋转
                else:
                    if face_detect_num != 3:
                        face_detect_num += 1
                        continue
                    else:
                        face_detect_num = 0
                    if turn_num != 6:
                        self.Motion.moveTo(0, 0, 0.52)
                        turn_num += 1
                    else:
                        self.Motion.moveTo(0, 0, -3.14)
                        turn_num = 0
                    waving_dete = True
        return "succe"

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

    def start(self):
        print "start"
        # 检测吧台
        self.scan_sub = rospy.Subscriber('/pepper_robot/naoqi_driver/laser', LaserScan, self.scan_callback)
        while self.bar_location == "none":
            if self.scan_msg_time == 4:
                time_left = time_right = 0
                sum_left = sum_right = 0
                for i in range(4):
                    dis_left=[]
                    dis_right=[]
                    for j in range(len(self.scan_msg[i].ranges)):
                        print "-----------"
                        print j, self.scan_msg[i].ranges[j]
                    for j in range(0,15):
                        dis_right.append(self.scan_msg[i].ranges[j])
                        #sum_right += self.scan_msg[i].ranges[j]
                        # print "===================right", self.scan_msg[i].ranges[j]
                    dis_right.sort(cmp=None, key=None, reverse=False)
                    for k in range(0,4):
                        sum_right+=dis_right[k]
                    for j in range(50, 56):
                        dis_left.append(self.scan_msg[i].ranges[j])
                        #sum_left += self.scan_msg[i].ranges[j]
                        # print "-------------------left", self.scan_msg[i].ranges[j + 31]
                    dis_left.sort(cmp=None, key=None, reverse=False)
                    for k in range(0,4):
                        sum_left+=dis_left[k]
                    if sum_right < sum_left:
                        time_right += 1
                        print "1111111"
                    else:
                        time_left += 1
                        print "0000000"
                if time_right > time_left:
                    self.bar_location = "right"
                    print "right"
                else:
                    self.bar_location = "left"
                    print "left"
        self.TextToSpe.say("The bar table is on the " + self.bar_location)
        self.save_point()
        self.Motion.moveTo(1.5, 0, 0)

        self.TextToSpe.say("I am going to find the guest")
        # find guest function 抬头找人
        self.angle = -.2
        print "fro"
        ## add find person  wa###ve detection##
        self.approach_waving()
        print "qqqq"
        self.save_point()
        self.save_point_inverse(self.point_dataset[1])
        self.TextToSpe.say("hello, What can I do for you?")
        self.start_recording(reset=True)
        self.analyze_content()
        self.go_to_waypoint(self.point_dataset[2])
        self.go_to_waypoint(self.point_dataset[0])
        if len(self.current_drink_name) != 0:
            self.TextToSpe.say("hey the guest need ")
            for i in range(len(self.current_drink_name)):
                self.TextToSpe.say(" " + self.current_drink_name[i])
                ####add posture #######
        self.go_to_waypoint(self.point_dataset[1])

    def save_point_inverse(self, point):
        self.if_save_switch = True
        print "save_point"
        # amcl定位
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback_inverse)
        while self.if_save_switch:
            time.sleep(1)
        amcl_sub.unregister()

    def save_point(self):
        self.if_save_switch = True
        print "save_point"
        # amcl定位
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        while self.if_save_switch:
            time.sleep(1)
        amcl_sub.unregister()

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while True:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

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

    def analyze_content(self):
        result = []
        # 记录是否找到物品
        object_found = False
        # 获取要找的物品
        for i in range(len(self.object)):
            if re.search(self.object[i].lower(), self.recog_result) != None:
                print "found one person:=", self.object[i].lower()

                object_found = True
                # 记下当前物品的名字
                result.append(self.object[i])
        self.TextToSpe.say("OK, I will take ")
        for i in range(len(result)):
            self.TextToSpe.say(result[i] + " ")
        self.TextToSpe.say("to you. Please wait for a moment")
        self.recog_result = "00"
        if object_found:
            self.type = "FOUND"
            self.current_drink_name = result
            return
        else:
            self.TextToSpe.say("sorry, please tell me again")
            self.start_recording(reset=True)
            self.analyze_content()

    def scan_callback(self, msg):
        print "in scan callback"
        print msg
        self.scan_msg.append(msg)
        self.scan_msg_time += 1
        if self.scan_msg_time == 4:
            self.scan_sub.unregister()

    def amcl_callback(self, msg):
        # print "yyyyyyyyy"
        # print msg
        # curr_pos = PoseStamped()
        point_temp = MoveBaseGoal()
        point_temp.target_pose.header.frame_id = '/map'
        point_temp.target_pose.header.stamp = msg.header.stamp
        point_temp.target_pose.header.seq = msg.header.seq
        point_temp.target_pose.pose.position.x = msg.pose.pose.position.x
        point_temp.target_pose.pose.position.y = msg.pose.pose.position.y
        point_temp.target_pose.pose.position.z = msg.pose.pose.position.z
        point_temp.target_pose.pose.orientation.x = msg.pose.pose.orientation.x
        point_temp.target_pose.pose.orientation.y = msg.pose.pose.orientation.y
        point_temp.target_pose.pose.orientation.z = msg.pose.pose.orientation.z
        point_temp.target_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.point_dataset.append(point_temp)
        print('\033[0;32m [Kamerider I] Point saved successfully!! \033[0m')
        self.if_save_switch = False

    def amcl_callback_inverse(self, msg):
        # print "yyyyyyyyy"
        # print msg
        # curr_pos = PoseStamped()
        point_temp = MoveBaseGoal()
        qua = transformations.quaternion_inverse([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        point_temp.target_pose.header.frame_id = '/map'
        point_temp.target_pose.header.stamp = msg.header.stamp
        point_temp.target_pose.header.seq = msg.header.seq
        point_temp.target_pose.pose.position.x = msg.pose.pose.position.x
        point_temp.target_pose.pose.position.y = msg.pose.pose.position.y
        point_temp.target_pose.pose.position.z = msg.pose.pose.position.z
        point_temp.target_pose.pose.orientation.x = qua[0]
        point_temp.target_pose.pose.orientation.y = qua[1]
        point_temp.target_pose.pose.orientation.z = qua[2]
        point_temp.target_pose.pose.orientation.w = qua[3]
        self.point_dataset.append(point_temp)
        print('\033[0;32m [Kamerider I] Point saved successfully!! \033[0m')
        self.if_save_switch = False

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            self.if_need_record = False

    def go_to_waypoint(self, Point):
        self.angle = .3
        self.nav_as.send_goal(Point)
        #self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            # 每隔4s清除一次local map
            if count_time == 3:
                #self.map_clear_srv()
                count_time = 0

    def stop_motion(self):
        # self.cancel_plan()
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
                elif command == "save":
                    self.approach_waving()
                elif command == "print":
                    print self.point_dataset
                elif command == 'c':
                    break
                else:
                    print("Invalid Command!")
            except Exception as e:
                print e

        self.f.close()

if __name__ == "__main__":
    params = {
        'ip': "192.168.43.30",
        'port': 9559
    }
    res = restaurant(params)
    res.keyboard_control()



