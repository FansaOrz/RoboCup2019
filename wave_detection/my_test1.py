import qi
import sys
import time
import dlib
import numpy as np
import cv2


class my_test:
    def __init__(self,params, session):
        self.session = session
        # self.ip = params["ip"]
        # self.port = params["port"]
        self.count = 0
        # self.session = qi.Session()
        self.switch = True
        # try:
        #     self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        # except RuntimeError:
        #     print("connection Error!!")
        #     sys.exit(1)
        self.SoundLoc= self.session.service("ALSoundLocalization")
        self.motion = self.session.service("ALMotion")
        self.Memo = self.session.service("ALMemory")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.if_stop = False
        self.detector = dlib.get_frontal_face_detector()
        self.RobotPos.goToPosture("Stand", .5)
        self.get_image_switch = True

        self.switch = True
        self.sound_localization()
        while True:
            if self.count>=3:
                self.stop_loc()
                break
        # while not self.if_stop:
        #
        #    #take_pic
        #    #if judge_face == no big:
        #     self.motion.moveTo(0.2,0,0)
        #    #elif judge_face == big:
        #       #break


    def sound_localization(self):
        self.SoundLoc.subscribe("SoundLocated")
        self.SoundLoc.setParameter("Sensitivity", 0.7)
        self.sound_localization_sub = self.Memo.subscriber("ALSoundLocalization/SoundLocated")
        self.sound_localization_sub.signal.connect(self.callback_sound_localization)

    def callback_sound_localization(self, msg):
        if self.switch == False:
            return
        self.switch = False

        time.sleep(.2)

        self.sound_loc = self.Memo.getData("ALSoundLocalization/SoundLocated")
        print("----Located!----", self.sound_loc[1][2])
        print("Energy:", self.sound_loc[1][3])
        # time.sleep(0.5)
        if self.sound_loc[1][2] > .5:
            self.motion.moveTo(0, 0, self.sound_loc[1][0])
        self.count += 1
        self.switch = True

    def stop_loc(self):
        self.SoundLoc.unsubscribe("SoundLocated")
        self.find_person()
        print "Topic unsubscribe!"

    def find_person(self):
        AL_kQVGA = 1
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        width = 320
        height = 240
        image = np.zeros((height, width, 3), np.uint8)
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
                self.wave_detection(image)
        return "succe"

    def wave_detection(self, frame):
        # print "9999"
        frame_copy = frame.copy()
        rects = self.detector(frame_copy, 2)
        if len(rects) != 0:
            # print "yyyyyyyyyyy"
            image_max = 0
            for rect in rects:
                cv2.rectangle(frame_copy, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255), 2, 8)
                cv2.imshow("yess", frame_copy)
                cv2.imwrite("./persom.jpg", frame_copy)
                if (rect.right() - rect.left())*(rect.bottom() - rect.top()) > image_max:
                    image_max = (rect.right() - rect.left())*(rect.bottom() - rect.top())
            print  float(image_max) / float(320*410)
            if float(image_max) / float(320*410) > .018 :
                self.get_image_switch = False
                self.if_stop = True
            else:
                self.motion.moveTo(0.2, 0, 0)
            cv2.waitKey(1)



def main(session):
    params = {
        'ip': "192.168.3.60",
        'port': 9559,
        'rgb_topic': 'pepper_robot/camera/front/image_raw'
    }
    pio = my_test(params, session)




if __name__ == "__main__":
    main()

