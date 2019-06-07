#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import base64
import urllib
import urllib2
import ssl


'''
百度AI-人脸注册
'''
def register_people(img_path, person_name):

    context = ssl._create_unverified_context()
    # client_id 为官网获取的AK， client_secret 为官网获取的SK
    host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=P5KGCrLD9Rlx3WXr3XOjBgCk&client_secret=CnXVsiGU85bobCgiD6gyGmnnqlrpkoFW'
    request = urllib2.Request(host)
    request.add_header('Content-Type', 'application/json; charset=UTF-8')
    response = urllib2.urlopen(request, context=context)
    content1 = response.read()
    access_token = content1.split("\"")[13]

    request_url = "https://aip.baidubce.com/rest/2.0/face/v3/faceset/user/add"

    f = open(img_path, 'rb')
    img = base64.b64encode(f.read())

    params = {"group_id":"store","image":img, "image_type":"BASE64", "user_id":person_name,"user_info":"userInfo5"}
    params = urllib.urlencode(params)

    request_url = request_url + "?access_token=" + access_token
    request = urllib2.Request(url=request_url, data=params)
    request.add_header('Content-Type', 'application/x-www-form-urlencoded')
    response = urllib2.urlopen(request)
    content = response.read()
    if content:
        print content

if __name__ == '__main__':
    register_people('/home/fansa/Src/pepper_example/convenience_store/people_image/001.jpg', "jiashi")