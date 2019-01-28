#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import urllib
import json
import hashlib
import base64
import urllib.request
import urllib.parse

def main(path):
    if_succe = False
    # 初始化参数
    url = 'http://api.xfyun.cn/v1/service/v1/iat'
    api_key = '588f3f2f7a18769bafc778fdca809a9b'
    param = {"engine_type": "sms-en16k", "aue": "raw"}
    x_appid = '5c487dc0'
    x_param = base64.b64encode(json.dumps(param).replace(' ', '').encode('utf-8'))
    x_param = str(x_param, 'utf-8')
    x_time = int(time.time())
    x_checksum = hashlib.md5((api_key + str(x_time) + x_param).encode('utf-8')).hexdigest()
    x_header = {'X-Appid': x_appid,
                'X-CurTime': x_time,
                'X-Param': x_param,
                'X-CheckSum': x_checksum}
    while not if_succe:
        f = open(path, 'rb')  # rb表示二进制格式只读打开文件
        file_content = f.read()
        base64_audio = base64.b64encode(file_content)  # base64.b64encode()参数是bytes类型，返回也是bytes类型
        body = urllib.parse.urlencode({'audio': base64_audio})
        req = urllib.request.Request(url=url, data=body.encode('utf-8'), headers=x_header, method='POST')
        result = urllib.request.urlopen(req)
        result = result.read().decode('utf-8')
        result = result.split("\"")
        if result[7] != "":
            if_succe = True
            print('\033[0;32m [Kamerider I] I heard' + result[7] + ' \033[0m')
            return result
        else:
            print("\033[0;32;40m\t[Kamerider W] : You don't need stop record\033[0m")
            if 0xFF == ord('q'):
                return

