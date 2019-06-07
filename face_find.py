#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import base64
import urllib, urllib2
import ssl
import json

'''
人脸搜索
'''
def find_face(image_path):
    context = ssl._create_unverified_context()

    host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=P5KGCrLD9Rlx3WXr3XOjBgCk&client_secret=CnXVsiGU85bobCgiD6gyGmnnqlrpkoFW'
    request = urllib2.Request(host)
    request.add_header('Content-Type', 'application/json; charset=UTF-8')
    response = urllib2.urlopen(request, context=context)
    content1 = response.read()
    access_token = content1.split("\"")[13]
    request_url = "https://aip.baidubce.com/rest/2.0/face/v3/search"
    f = open(image_path, 'rb')

    img = base64.b64encode(f.read())
    image64 = str(img).encode("utf-8")
    params = {"image":image64,"image_type":"BASE64","group_id_list":"store","quality_control":"LOW","liveness_control":"NORMAL"}
    params = urllib.urlencode(params).encode("utf-8")
    # print params
    # access_token = '[调用鉴权接口获取的token]'
    request_url = request_url + "?access_token=" + access_token
    request = urllib2.Request(url=request_url, data=params)
    request.add_header('Content-Type', 'application/json')
    response = urllib2.urlopen(request)
    try:
        content = response.read()
        dict_info = json.loads(content)
        return dict_info["result"]["user_list"][0]["user_id"]
    except Exception as e:
        print e
        return None

if __name__ == '__main__':
    find_face('/home/fansa/Src/pepper_example/convenience_store/people_image/001.jpg')