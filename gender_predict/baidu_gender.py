# -*- coding: utf-8 -*-

import urllib, urllib2, sys
import ssl
import cv2
import base64
import json


def gender_pre(img_name):
    max_re = 0
    max_rectangle_geder = "none"
    context = ssl._create_unverified_context()
    # client_id 为官网获取的AK， client_secret 为官网获取的SK
    host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=P5KGCrLD9Rlx3WXr3XOjBgCk&client_secret=CnXVsiGU85bobCgiD6gyGmnnqlrpkoFW'
    request = urllib2.Request(host)
    request.add_header('Content-Type', 'application/json; charset=UTF-8')
    response = urllib2.urlopen(request, context=context)
    content1 = response.read()
    img = cv2.imread(img_name)
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
    response = urllib2.urlopen(request, context=context)
    content = response.read()
    dict_info = json.loads(content)
    print "================================="
    print dict_info
    print "================================="
    male_num = 0
    female_num = 0
    try:
        face_list = dict_info["result"]["face_list"]
    except:
        return 0, 0
    for i in range(len(face_list)):
        left = int(face_list[i]["location"]["left"])
        top = int(face_list[i]["location"]["top"])
        right = int(face_list[i]["location"]["left"] + face_list[i]["location"]["width"])
        bottom = int(face_list[i]["location"]["top"] + face_list[i]["location"]["height"])
        if face_list[i]["gender"]["type"] == "male":
            male_num += 1
        else:
            female_num += 1
        cv2.putText(img, face_list[i]["gender"]["type"], (left-10, bottom+10), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 1)
        cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
        if (right-left) * (bottom-top) > max_re:
            max_rectangle_geder = face_list[i]["gender"]["type"]
    cv2.imwrite("./person_image/result.jpg", img)
    return male_num, female_num, max_rectangle_geder
