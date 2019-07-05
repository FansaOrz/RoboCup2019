#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import re
import sys
import time
import rospy
import almath
import atexit
import thread
import actionlib
from threading import Thread
from follow import pepper_follow
from speech_recog import baidu_recognition_text
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class carry_my_luggage():

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
        self.BehaviorMan = self.session.service("ALBehaviorManager")
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
        # 调小安全距离
        self.Motion.setTangentialSecurityDistance(.05)
        self.Motion.setOrthogonalSecurityDistance(.1)

        # 停止录音
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
        self.if_need_hand = True
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        # 声明一些变量
        self.angle = -0.4
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
        self.stop = ["stop", "here is the car"]
        # 当前时间戳（订阅相机的名字，每个只能使用6次）
        ticks = time.time()
        # 0代表top相机 最后一个参数是fps
        self.VideoDev.subscribeCamera(str(ticks), 0, 2, 11, 40)
        # 设置dialog语言
        self.TextToSpe.setLanguage("English")
        # track的程序名
        self.track_behavior_name = "peopletrack-84f37e/behavior_1"
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
        self.set_volume(.7)
        self.show_image("instructions.png")
        self.keyboard_control()

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def start_foll(self):
        # 抬头
        self.angle = -.5
        self.Motion.setAngles("Head", [0., self.angle], .2)
        self.TextToSpe.say("Dear operator.")
        self.TextToSpe.say("Please talk to me after my eyes' color turn to white ")
        time.sleep(1)
        self.show_image("hand-me-the-bag.png")
        self.TextToSpe.say("I can not grasp the bag. Could you please pass me ?")
        # 做出伸手的动作
        self.stretch_out_hand()
        time.sleep(5)
        self.angle = -.2
        self.Motion.setAngles("Head", [0., self.angle], .05)
        self.pepper_follow_me.start_follow()

        while not self.if_stop_follow:
            rospy.sleep(.5)
            self.start_recording(reset=True)
            self.analyze_content()
        self.pepper_follow_me.stop_follow()

        # TODO
        self.if_need_hand = False
        time.sleep(2)
        names_R = ["RElbowRoll", "RElbowYaw", "RHand", "RShoulderPitch", "RShoulderRoll", "RWristYaw"]
        angleLists_R = [48.5 * almath.TO_RAD, 50.2 * almath.TO_RAD, 0.22 * almath.TO_RAD, 39.8 * almath.TO_RAD,
                        -5.7 * almath.TO_RAD, 102.3 * almath.TO_RAD]
        timeLists = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        isAbsolute = True
        self.Motion.angleInterpolation(names_R, angleLists_R, timeLists, isAbsolute)
        self.TextToSpe.say("please take away your bag")
        self.TextToSpe.say("I will go back to the living room")
        self.start_head_fix()


        # 回到livingroom
        self.Motion.moveTo(0, 0, 3)
        self.go_to_waypoint(self.point_dataset["point6"])
        self.go_to_waypoint(self.point_dataset["point33"])
        self.go_to_waypoint(self.point_dataset["point14"])
        self.go_to_waypoint(self.point_dataset["point16"])
        self.go_to_waypoint(self.point_dataset["point25"])
        self.TextToSpe.say("I have arrived at living room")

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def start_hand_close(self):
        arg = tuple([1])
        thread.start_new_thread(self.hand_close_thread, arg)

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

    def stretch_out_hand(self):
        names_R = ["RElbowRoll", "RElbowYaw", "RHand", "RShoulderPitch", "RShoulderRoll", "RWristYaw"]
        angleLists_R = [48.5 * almath.TO_RAD, 50.2 * almath.TO_RAD, 0.22 * almath.TO_RAD, 39.8 * almath.TO_RAD, -5.7 * almath.TO_RAD, 102.3 * almath.TO_RAD]
        timeLists = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        isAbsolute = True
        self.Motion.angleInterpolation(names_R, angleLists_R, timeLists, isAbsolute)
        time.sleep(4)
        self.start_hand_close()

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
        for i in range(len(self.stop)):
            if re.search(self.stop[i], self.recog_result) != None:
                # self.follow_start_pub.publish("off")
                # self.pepper_follow_me.stop_follow()
                self.recog_result = "None"
                self.TextToSpe.say("I will stop following you")
                self.if_stop_follow = True
                print('\033[0;32m [Kamerider I] Stop following the person  \033[0m')
                return "stop"
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
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

    def hand_close_thread(self, arg):
        self.Motion.setStiffnesses("Body", 1.0)
        names_R = ["RElbowRoll", "RElbowYaw", "RHand", "RShoulderPitch", "RShoulderRoll", "RWristYaw", "LElbowRoll", "LElbowYaw", "LHand", "LShoulderPitch", "LShoulderRoll", "LWristYaw"]
        angleLists_R = [47.5 * almath.TO_RAD, -44.1 * almath.TO_RAD, 0.07 * almath.TO_RAD, 91.1 * almath.TO_RAD, -14.2 * almath.TO_RAD, 72.2 * almath.TO_RAD,
                        -48.4 * almath.TO_RAD, 44.1 * almath.TO_RAD, 0.06 * almath.TO_RAD, 91.3 * almath.TO_RAD, 13.3 * almath.TO_RAD, -74.1 * almath.TO_RAD]
        angleLists_R = [88.8 * almath.TO_RAD, 13.7 * almath.TO_RAD, 0.07 * almath.TO_RAD, -116.6 * almath.TO_RAD, -25.8 * almath.TO_RAD, -49.2 * almath.TO_RAD,
                        -88.5 * almath.TO_RAD, -13.1 * almath.TO_RAD, 0.06 * almath.TO_RAD, -117.9 * almath.TO_RAD, 24.9 * almath.TO_RAD, 45.6 * almath.TO_RAD]
        timeLists = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        isAbsolute = True
        while True:
            if self.if_need_hand:
                self.Motion.angleInterpolation(names_R, angleLists_R, timeLists, isAbsolute)
                # self.Motion.angleInterpolation(names_L, angleLists_L, timeLists, isAbsolute)
            time.sleep(.1)

    def show_image(self, image_name):
        # cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        # os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImage("http://198.18.0.1/apps/boot-config/" + str(image_name))

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

    def go_to_waypoint(self, Point):
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
                elif command == 'go':
                    self.stretch_out_hand()
                elif command == 'st':
                    self.start_foll()
                elif command == 'c':
                    self.cancel_plan(); self.set_velocity(0, 0, 0); break
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"

def main():
    params = {
        'ip' : "192.168.43.167",
        'port' : 9559,
        'rgb_topic' : 'pepper_robot/camera/front/image_raw'
    }
    carry_my_luggage(params)

if __name__ == "__main__":
    main()
