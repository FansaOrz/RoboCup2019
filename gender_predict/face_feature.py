# -*- coding: utf-8 -*-

import urllib, urllib2, sys
import ssl
import cv2
import base64
import json


def gender(img_name):
    max_re = 0
    emotion = None
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
    params = {"image":''+image64+'',"image_type":"BASE64","face_field":"expression,gender,faceshape,age,race,emotion", "max_face_num":10}
    params = urllib.urlencode(params).encode("utf-8")
    access_token = content1.split("\"")[13]
    request_url = request_url + "?access_token=" + access_token
    request = urllib2.Request(url=request_url, data=params)
    request.add_header('Content-Type', 'application/json')
    response = urllib2.urlopen(request, context=context)
    content = response.read()
    dict_info = json.loads(content)
    print "================================="
    print content
    print "================================="
    male_num = 0
    female_num = 0
    try:
        face_list = dict_info["result"]["face_list"]
    except:
        return "none", 0, 0
    for i in range(len(face_list)):
        left = int(face_list[i]["location"]["left"])
        top = int(face_list[i]["location"]["top"])
        right = int(face_list[i]["location"]["left"] + face_list[i]["location"]["width"])
        bottom = int(face_list[i]["location"]["top"] + face_list[i]["location"]["height"])
        if face_list[i]["gender"]["type"] == "male":
            male_num += 1
        else:
            female_num += 1
        cv2.putText(img, face_list[i]["gender"]["type"], (left-10, bottom+40), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)
        cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
        if (right-left) * (bottom-top) > max_re:
            max_rectangle_geder = face_list[i]["gender"]["type"]
        print "The gender of person", i + 1, "is", face_list[i]["gender"]["type"]
        print "The age of person", i + 1, "is", face_list[i]["age"]

        # When the result of racial detection is arabs, we think the person's skin is black
        if face_list[i]["race"]["type"] == "arabs":
            face_list[i]["race"]["type"] = "brown"

        print "The color of skin of person", i + 1, "is", face_list[i]["race"]["type"]

        expression = judge_expression(face_list[i]["expression"]["type"])# Judge the complexion of a person
        print "The complexion of person", i + 1, "is", face_list[i]["expression"]["type"]

        emotion = "The emotion of person " + str(i + 1) + " is " + str(face_list[i]["emotion"]["type"])
        print emotion
        print "================================="
    cv2.imwrite("./gender_result.jpg", img)
    return face_list[i]["gender"]["type"], face_list[i]["age"], face_list[i]["race"]["type"]


def judge_expression(n):
    e = None
    if n == "None":
        e = 0
    elif n == "smile":
        e = 1
    elif n == "laugh":
        e = 2
    return e


if __name__ == '__main__':
    gender("./capture.jpg")
