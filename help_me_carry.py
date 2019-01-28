#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import sys
import time
import rospy
import atexit
import thread
import datetime
import actionlib
from actionlib_msgs.msg import GoalID
from .speech_recog import speech_recognition_text
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from json import dumps


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

        # 需要使用的naoqi api
        self.VideoDev = self.session.service("ALVideoDevice")
        self.FaceCha = self.session.service("ALFaceCharacteristics")
        self.FaceDet = self.session.service("ALFaceDetection")
        self.Memory = self.session.service("ALMemory")
        self.Dialog = self.session.service("ALDialog")
        self.AnimatedSpe = self.session.service("ALAnimatedSpeech")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.TabletSer = self.session.service("ALTabletService")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.Motion = self.session.service("ALMotion")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.Tracker = self.session.service("ALTracker")
        '''
        # topic的回调函数
        start_following_sub = self.Memory.subscribe("follow_switch")
        start_following_sub.signal.connect(self.callback_start_following)
        start_navigation_sub = self.Memory.subscribe("navigation_switch")
        start_navigation_sub.signal.connect(self.callback_start_navigation)
        face_detection_sub = self.Memory.subscribe("FaceDetected")
        face_detection_sub.signal.connect(self.callback_face_detection)
        '''
        # ROS
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()

        # 关闭 basic_awareness
        self.BasicAwa.setEnabled(False)

        # 设置tracker模式为头追踪
        # --------------------------可能没用---------------------------
        self.Tracker.setMode("Head")

        # 初始化平板
        self.TabletSer.cleanWebview()

        # 订阅相机
        self.rgb_top = self.VideoDev.subscribeCamera('rgb_t', 0, 2, 11, 40)
        self.depth = self.VideoDev.subscribeCamera('dep', 2, 1, 17, 20)

        # 设置dialog语言
        self.Dialog.setLanguage("English")
        '''
        # 加载pepper电脑里的topic
        self.Topic_path = '/home/nao/top/competetion_enu.top'

        # 以utf-8的格式编码
        self.Topic_path = self.Topic_path.decode('utf-8')

        # ========================测试删除utf-8行不行===========================================
        self.Topic_name = self.Dialog.loadTopic(self.Topic_path.encode('utf-8'))
        '''
        # 初始化头的位置
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., -0.25], .05)

        # 设置说话速度
        self.TextToSpe.setParameter("speed", 80.0)

        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)

        # 记忆的人脸
        self.learned_face_list = self.FaceDet.getLearnedFaceList()
        print ('\033[0;32m [Kamerider I] ========Learned Face List======== \033[0m')
        print (self.learned_face_list)

        #设置参数
        self.FaceDet.setRecognitionEnabled(False)

        # 声明一些要用的变量
        # 需不需要固定头
        self.state = True
        # 需不需要记录
        self.log_enabled = True

        # 记录的路径


        self.log_face = open('./data/' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M') + '/log_faces.csv', 'w')
        self.log_people = open('./data/' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M') + '/log_people.csv', 'w')
        self.log_video = open('./data/' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M') + '/log_video.csv', 'w')

        # 调用成员函数
        self.init_waypoints()
        self.start_head_fix()
        self.set_volume(.7)
        self.start_dialog()

    def __del__(self):
        print("shutting down")

    def init_waypoints(self):
        # 厨房
        self.Point_Kitchen = MoveBaseGoal()
        self.Point_Kitchen.target_pose.header.frame_id = '/map'
        self.Point_Kitchen.target_pose.header.stamp = self.PepperPosition.header.stamp
        self.Point_Kitchen.target_pose.header.seq = self.PepperPosition.header.seq
        self.Point_Kitchen.target_pose.pose.position.x =-51.8532496383
        self.Point_Kitchen.target_pose.pose.position.y =55.1571428016
        self.Point_Kitchen.target_pose.pose.position.z = .0
        self.Point_Kitchen.target_pose.pose.orientation.x = .0
        self.Point_Kitchen.target_pose.pose.orientation.y = .0
        self.Point_Kitchen.target_pose.pose.orientation.z =0.395172208021
        self.Point_Kitchen.target_pose.pose.orientation.w =0.918607057456
        self.Point_Bedroom = MoveBaseGoal()
        self.Point_Bedroom.target_pose.header.frame_id = '/map'
        self.Point_Bedroom.target_pose.header.stamp = self.PepperPosition.header.stamp
        self.Point_Bedroom.target_pose.header.seq = self.PepperPosition.header.seq
        self.Point_Bedroom.target_pose.pose.position.x =-51.8532496383
        self.Point_Bedroom.target_pose.pose.position.y =55.1571428016
        self.Point_Bedroom.target_pose.pose.position.z = .0
        self.Point_Bedroom.target_pose.pose.orientation.x = .0
        self.Point_Bedroom.target_pose.pose.orientation.y = .0
        self.Point_Bedroom.target_pose.pose.orientation.z =0.395172208021
        self.Point_Bedroom.target_pose.pose.orientation.w =0.918607057456
        self.waypoint = {'Kitchen':self.Point_Kitchen, 'Bedroom':self.Point_Bedroom}

    def start_head_fix(self):
        # 声明一个元组
        arg = tuple([1])
        self.state = True
        self.head_fix = True
        # 声明一个线程
        thread.start_new_thread(self.head_fix_thread(), args=arg)

    def head_fix_thread(self):
        while self.head_fix:
            self.Motion.setStiffnesses("head", 1.0)
            self.Motion.setAngles("Head", [0., self.angle], .05)
            time.sleep(2)

    def show_image(self, item_name):
        self.TabletSer.hideImage()
        # 平板显示特定的图片（必须实现存在pepper的电脑里）
        # =================================查找别的方法使平板可以实时的显示图片========================
        self.TabletSer.showImage("http://198.18.0.1/apps/boot-config/" + item_name)

    def start_dialog(self):
        self.Dialog.subscribe("competetion")

    def start_find_person(self):
        print('\033[0;32m [Kamerider I] start finding person \033[0m')
        # 设置参数
        self.FaceDet.setRecognitionEnabled(True)

    def stop_dialog(self):
        try:
            self.Dialog.unsubscribe("competition")
        except RuntimeError:
            print ("[Kamerider E ] : the event \'my_subscribe_test\' hasn't been subscribed")

    def callback_face_detection(self, msg):
        print('\033[0;32m [Kamerider I] ========face detection message======== \033[0m')
        print (msg)
        if len(msg) > 0:
            detectionTimestamp = msg[0]
            cameraPose_InTorsoFrame = msg[2]
            cameraPose_InRobotFrame = msg[3]
            cameraId = msg[4]
            if len(msg[1]) > 0:
                #    - [] when nothing new.
                #    - [4] when a face has been detected but not recognized during the first 8s.
                #    - [2, [faceName]] when one face has been recognized.
                #    - [3, [faceName1, faceName2, ...]] when several faces have been recognized.
                timeFilteredResult = msg[1][len(msg[1]) - 1]
                if len(timeFilteredResult) == 1:
                    if timeFilteredResult[0] == 4:
                        print ('\033[0;32m [Kamerider I] detected unknown face for more than 8s \033[0m')
                        pass
                elif len(timeFilteredResult) == 2:
                    if timeFilteredResult[0] in [2, 3]:
                        for s in timeFilteredResult[1]:
                            print ("Recognized: ", s)
        face_array = []
        try:
            faceInfoArray = msg[1][0:len(msg[1]) - 1]
            for faceInfo in faceInfoArray:
                # First Field = Shape info.
                faceShapeInfo = faceInfo[0]
                # Second Field = Extra info .
                faceExtraInfo = faceInfo[1]
                print ("  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2]))
                print ("  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4]))
                print ('  ID:', faceExtraInfo[0], ', score= ', round(faceExtraInfo[1], 3), ', label= ', faceExtraInfo[2])

                pose = {'alpha': round(faceShapeInfo[1], 2), 'beta': round(faceShapeInfo[2], 2),
                        'width': round(faceShapeInfo[3], 2), 'height': round(faceShapeInfo[4], 2)}
                face = {'timestamp': currentimestamp, 'faceid': faceExtraInfo[0], 'user': faceExtraInfo[2],
                        'confidence': round(faceExtraInfo[1], 3), 'pose': pose,
                        'camerapose_robot': cameraPose_InRobotFrame, 'camerapose_torso': cameraPose_InTorsoFrame}
                json_face = dumps(face)
                print ('json_face:: ', json_face)


        except:
            print ("faces detected, but it seems getData is invalid.")

    def callback_start_following(self, msg):
        # =============查看topic里面的参数是否会传进来===========================
        print (msg)

        print('\033[0;32m [Kamerider I] start following people \033[0m')

        # 先识别人脸
        self.personid = self.Memory.getData("EngagementZones/PersonEnteredZone1")

        # 设置track的模式 navigation是follow人并且可以自动避障
        self.Tracker.setMode("Navigate")

        # x y z threshold_x threshold_y threshold_z follow的阈值
        self.Tracker.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])

        # 把人的id目前的target匹配起来
        self.Tracker.registerTarget("People", self.personid)

        # 开始track
        self.Tracker.track("People")

        # 实时输出目前和目标的距离
        print ('\033[0;32m [Kamerider I] ======current relative position======= \033[0m')
        print (self.Tracker.getRelativePosition())

    def callback_start_navigation(self, msg):
        self.stop_dialog()
        print('\033[0;32m [Kamerider I] start navigating to the' + msg + '\033[0m')
        text = "I will go to the " + msg
        self.say(text)
        self.go_to_waypoint(self.waypoint[msg])
        text = "I have arrived at " + msg
        self.say(text)
        text = "I am looking for the people who could help me"
        self.say(text)
        self.start_find_person()

    def say(self, text):
        print ('\033[0;32m [Kamerider say]:' + text + "\033[0m")
        self.TextToSpe.say(text)
        time.sleep(.15)

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

    def arrive_at_car(self):
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()
        self.say("I have remembered the location of car")
        self.say("Now, please put the bag onto my hand.")
        self.say("where do you want me to?")

    def go_to_waypoint(self, Point):
        # 设置pepper头的角度
        self.angle = .1
        self.nav_as.send_goal(Point)
        self.map_clear_srv()
        # 如果没到，就每过5s清理一次local map
        while self.nav_as.get_state() != 3:
            time.sleep(5)
            self.map_clear_srv()
        self.angle = -.33
        return

def main():
    params = {
        'ip' : "127.0.0.1",
        'port' : 36057,
        'rgb_topic' : 'pepper_robot/camera/front/image_raw'
    }
    pio = help_me_carry(params)

if __name__ == "__main__":
    main()
