# -*- coding: utf-8 -*-

import urllib, urllib2, sys
import ssl
import cv2
import base64
import json
from PIL import Image
from io import BytesIO
import time

def frame2base64(frame):
    img = Image.fromarray(frame) #将每一帧转为Image
    output_buffer = BytesIO() #创建一个BytesIO
    img.save(output_buffer, format='JPEG') #写入output_buffer
    byte_data = output_buffer.getvalue() #在内存中读取
    base64_data = base64.b64encode(byte_data) #转为BASE64
    return base64_data


def gender_pre(img_name):
    context = ssl._create_unverified_context()
    cv2.namedWindow("wave", cv2.WINDOW_NORMAL)
    # client_id 为官网获取的AK， client_secret 为官网获取的SK
    host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=mpIzjdsc1j3GmRZLrINZ1Qpy&client_secret=1p54tkFSPUG4Oy0ED2YSHBKlbCLqYklT'
    request = urllib2.Request(host)
    request.add_header('Content-Type', 'application/json; charset=UTF-8')
    response = urllib2.urlopen(request, context=context)
    content1 = response.read()
    #time.sleep(2)
    capture = cv2.VideoCapture(3)
    success, frame = capture.read()
    request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/gesture"
    while success:
        cv2.imwrite(img_name, frame)
        f = open(img_name, 'rb')

        #image64 = frame2base64(frame)
        #print type(frame)
        #image = base64.b64encode(frame.encode())
        image = base64.b64encode(f.read())
        image64 = str(image).encode("utf-8")
        params = {"image":''+image64+''}
        params = urllib.urlencode(params).encode("utf-8")
        access_token = content1.split("\"")[13]
        request_url = request_url + "?access_token=" + access_token
        request = urllib2.Request(url=request_url, data=params)
        request.add_header('Content-Type', 'application/json')
        try:
            response = urllib2.urlopen(request, context=context)
        except:
            print "urlopen error"
            continue
        content = response.read()
        dict_info = json.loads(content)
        #print dict_info
        try:
            result_list = dict_info['result']
        except:
            continue
        print "========"
        print result_list
        print "========"
        for i in range(len(result_list)):
            left = int(result_list[i]["left"])
            top = int(result_list[i]["top"])
            right = int(result_list[i]["left"] + result_list[i]["width"])
            bottom = int(result_list[i]["top"] + result_list[i]["height"])
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        cv2.imshow("wave", frame)
        if cv2.waitKey(1) >= 0:
            break
        success, frame = capture.read()
    cv2.destroyAllWindows()
    capture.release()



if __name__== "__main__":
    gender_pre("./five.png")