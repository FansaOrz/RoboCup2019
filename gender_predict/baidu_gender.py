# -*- coding: utf-8 -*-

import urllib, urllib2, sys
import ssl
import cv2
import base64
import json

class gender_predict:
    def __init__(self, img_name):

        # client_id 为官网获取的AK， client_secret 为官网获取的SK
        host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=P5KGCrLD9Rlx3WXr3XOjBgCk&client_secret=CnXVsiGU85bobCgiD6gyGmnnqlrpkoFW'
        request = urllib2.Request(host)
        request.add_header('Content-Type', 'application/json; charset=UTF-8')
        response = urllib2.urlopen(request)
        content1 = response.read()
        self.gender(img_name)

    def gender(self, img_name):
        cv2.namedWindow("aaa", cv2.WINDOW_NORMAL)
        request_url = "https://aip.baidubce.com/rest/2.0/face/v3/detect"
        f = open(img_name, 'rb')
        image = base64.b64encode(f.read())
        image64 = str(image).encode("utf-8")
        image_type = "BASE64"
        params = {"image":''+image64+'',"image_type":"BASE64","face_field":"gender,faceshape", "max_face_num":10}
        params = urllib.urlencode(params).encode("utf-8")
        access_token = content1.split("\"")[13]
        request_url = request_url + "?access_token=" + access_token
        request = urllib2.Request(url=request_url, data=params)
        request.add_header('Content-Type', 'application/json')
        response = urllib2.urlopen(request)
        content = response.read()
        dict_info = json.loads(content)
        male_num = 0
        female_num = 0
        face_list = dict_info["result"]["face_list"]
        for i in range(len(face_list)):
            left = int(face_list[i]["location"]["left"])
            top = int(face_list[i]["location"]["top"])
            right = int(face_list[i]["location"]["left"] + face_list[i]["location"]["width"])
            bottom = int(face_list[i]["location"]["top"] + face_list[i]["location"]["height"])
            if face_list[i]["gender"]["type"] == "male":
                male_num += 1
            else:
                female_num += 1
            cv2.putText(img, face_list[i]["gender"]["type"], (left-10, top-20), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 3)
            cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
        cv2.imwrite("result", img)
        return male_num, female_num
