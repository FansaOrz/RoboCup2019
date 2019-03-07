#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import time
import cv2
from threading import Thread
from geometry_msgs.msg import Twist


class gender_predict:
    def __init__(self, session):
        self.FaceDet = session.service("ALFaceDetection")
        self.Memory = session.service("ALMemory")
        self.FaceCha = session.service("ALFaceCharacteristics")

        # face detection 回调函数
        self.face_id = []
        self.FaceDet.subscribe("HumanGreeter")
        #self.Face_Dete = self.Memory.subscriber("FaceDetected")
        self.Face_Dete = self.Memory.subscriber("PeoplePerception/PeopleDetected")
        self.Face_Dete.signal.connect(self.callback_face_dete)

        self.face_gender = Thread(target=self.start_gender_recog(), args=[])
        self.face_gender.daemon = False
        self.face_gender.start()
        self.face_gender.join()

    def callback_face_dete(self, msg):
        try:

            val = self.Memory.getData("FaceDetected")

            faceInfoArray = val[1]

            face_id = msg[1][0][1][0]
            faceInfoArray = msg[1]
            for j in range( len(faceInfoArray)-1 ):
                #print len(faceInfoArray)-1
                faceInfo = faceInfoArray[j]
                faceExtraInfo = faceInfo[1]
                #print "face num", len(faceInfoArray) - 1
                if faceExtraInfo[0] not in self.face_id:
                    self.face_id.append(face_id)
                print self.face_id
        except IndexError:
            print "IndexError"



    def start_gender_recog(self):
        print "------"
        time.sleep(3)
        print "face_id", self.face_id

        while self.face_id != []:
            print "----------"
            for i in range(len(self.face_id)):
                print self.face_id[i]
                if self.FaceCha.analyzeFaceCharacteristics(self.face_id[i]):
                    print "Person NO." + str(i) + " analyze succeed!"
                else:
                    print "Person NO." + str(i) + " analyze failed!"
            print self.Memory.getData("PeoplePerception/Person/" + str(self.face_id[i]) + "/GenderProperties")





