#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import requests
import time
import urllib
import json
import hashlib
import base64

URL = 'http://api.xfyun.cn/v1/service/v1/iat'
APPID = '5c487dc0'
API_KEY = '588f3f2f7a18769bafc778fdca809a9b'

def main(path):
    if_succe = False
    curTime = str(int(time.time()))
    param = "{\"engine_type\": \"sms-en16k\", \"aue\": \"raw\"}"
    paramBase64 = base64.b64encode(param)

    m2 = hashlib.md5()
    m2.update(API_KEY + curTime + paramBase64)
    checkSum = m2.hexdigest()
    header = {
        'X-CurTime': curTime,
        'X-Param': paramBase64,
        'X-Appid': APPID,
        'X-CheckSum': checkSum,
        'Content-Type': 'application/x-www-form-urlencoded; charset=utf-8',
    }
    while not if_succe:
        f = open(path, 'rb')
        file_content = f.read()
        base64_audio = base64.b64encode(file_content)
        body = urllib.urlencode({'audio': base64_audio})

        r = requests.post(URL, headers=header, data=body)

        result = json.loads(r.content)
        print result

        if result["code"] == "0":
            if_succe = True
            if not result["data"] == '':
                data = str(result["data"]).lower()
                print('\033[0;32m [Kamerider I] I heard' + data + ' \033[0m')
                return data
            else:
                return "00"
        else:
            print("\033[0;33m [Kamerider W] : You don't need stop record\033[0m")
            time.sleep(5)
#if __name__ == '__main__':
#    main("../audio_record/test.wav")

