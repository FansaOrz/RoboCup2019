#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import cv2
from keras.models import load_model
import numpy as np
import dlib

switch = True
global frame


def preprocess_input(x, v2=True):
    x = x.astype('float32')
    x = x / 255.0
    if v2:
        x = x - 0.5
        x = x * 2.0
    return x


if __name__ == '__main__':
    # 加载模型
    gender_model_path = './model/simple_CNN.81-0.96.hdf5'
    gender_classifier = load_model(gender_model_path, compile=False)
    gender_target_size = gender_classifier.input_shape[1:3]

    # 框住人脸的矩形边框颜色
    color = (0, 255, 0)

    gender_labels = {0: 'woman', 1: 'man'}
    # 人脸识别分类器本地存储路径
    cascade_path = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"
    detector = dlib.get_frontal_face_detector()
    # 循环检测识别人脸
    num = 0
    if switch == True:
        frame = cv2.imread("/home/jiashi/Desktop/Link to RoboCup2019/person_image/4.jpg")
        # 图像灰化，降低计算复杂度
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        faceRects = detector(frame_gray, 0)
        '''
        # 使用人脸识别分类器，读入分类器
        cascade = cv2.CascadeClassifier(cascade_path)
        # 利用分类器识别出哪个区域为人脸
        faceRects = cascade.detectMultiScale(frame_gray, scaleFactor=1.2, minNeighbors=3, minSize=(32, 32))
        '''
        if len(faceRects) > 0:
            for faceRect in faceRects:
                print(faceRect)
                num += 1
                x, y, w, h = faceRect.left(), faceRect.top(), faceRect.right(), faceRect.bottom()
                # 截取脸部图像提交给模型识别这是谁
                image = frame_gray[y - 10: h + 10, x - 10: w + 10]
                gray_face = cv2.resize(image, (gender_target_size))
                gray_face = np.expand_dims(gray_face, 0)
                #gray_face = np.expand_dims(gray_face, -1)
                gray_face = preprocess_input(gray_face, False)
                faceRects = gender_classifier.predict(gray_face)
                gender_label_arg = np.argmax(faceRects)
                gender_text = gender_labels[gender_label_arg]
                cv2.rectangle(frame, (x - 10, y  - 10), (w + 10, h + 10), color, thickness=2)
                text = "female"
                # 文字提示是谁
                cv2.putText(frame, gender_text, (x + 30, y + 30),  # 坐标
                            cv2.FONT_HERSHEY_SIMPLEX,  # 字体
                            1,  # 字号
                            (255, 0, 255),  # 颜色
                            2)  # 字的线宽
        cv2.namedWindow("aaa", cv2.WINDOW_NORMAL)
        cv2.imwrite("aaa.jpg", frame)
        cv2.imshow("aaa", frame)
        # 等待10毫秒看是否有按键输入
        cv2.waitKey()
        # 如果输入q则退出循环
    # 释放摄像头并销毁所有窗口
    #cap.release()
    cv2.destroyAllWindows()
