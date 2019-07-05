    def find_person(self):
        self.angle = -.2
        self.Motion.setAngles("Head", [0., self.angle], .2)
        AL_kQVGA = 2
        current_right = current_left = current_bottom = current_top = 0
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        width = 640
        height = 480
        image = np.zeros((height, width, 3), np.uint8)
        if_turn_finished = False
        if_first = True
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
                cv2.imshow("pepper-top-camera-640*480px", image)
                cv2.imwrite("./human_detection.jpg", image)
                cv2.waitKey(1)
                # dlib检测人脸
                # rects = self.detector(image, 2)
                rects = self.human_detector.main("./human_detection.jpg")
                print rects
                # 检测到人脸
                if len(rects) != 0:
                    if_first = True
                    # 转向完成，开始接近人
                    if if_turn_finished:
                        # 第一次看到人，就进行一次特征识别
                        if (self.upper_wear == "none" and self.upper_color == "none"):

                            image_name = "/home/fansa/Src/pepper_example/RoboCup2019/person_body.jpg"
                            # cv2.imwrite(image_name, image)
                            # self.position = self.object_detection.main(image_name)
                            self.split_person_from_img(image, image_name, rects)
                            _, _, self.upper_color, self.upper_wear = body_feature.feature(image_name)
                            if self.upper_color == "none" and self.upper_wear == "none":
                                self.Motion.moveTo(-.5, 0, 0)
                                self.angle -= .1
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                                continue
                        # 人的宽度所占的比例
                        print "rate=====================", float(rects[2] - rects[0]) / float(width)
                        if float(rects[2] - rects[0]) / float(width) > .48:
                            self.set_velocity(0, 0, 0)
                            self.get_image_switch = False
                        else:
                            # 通过人体框的上边框判断是否需要抬头
                            if rects[1] < height * 0.204:
                                print "upupupupupupupupupupupupupupup", current_bottom
                                self.angle -= .05
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                            elif rects[1] > height * 0.4125:
                                print "downdowndowndowndowndowndowndown", current_top
                                self.angle += .05
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                            self.set_velocity(.15, 0, 0)
                            # 再旋转
                            center = (rects[0] + rects[2]) / 2
                            if abs(width / 2 - center) > width / 10:
                                print "inininininininininininnnininininini"
                                Error_dist_ = width / 2 - center
                                self.set_velocity(0, 0, 0.001*Error_dist_)
                                time.sleep(1.8)
                                self.set_velocity(0.1, 0, 0)
                        cv2.waitKey(1)
                    # 开始转向人
                    else:
                        human_center = (rects[2] + rects[0]) / 2
                        Error_dist = width / 2 - human_center
                        if abs(Error_dist) <= 10:
                            if_turn_finished = True
                            continue
                        self.Motion.moveTo(0, 0, 0.002*Error_dist)
                        cv2.waitKey(1)
                # 没有检测到人脸就旋转
                else:
                    if if_first:
                        if_first = False
                        continue
                    self.Motion.moveTo(0, 0, 3.14 / 4)
        return "succe"
