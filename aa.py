#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import re

start = ["follow me", "follow", "following", "start"]
for i in range(len(start)):
    if re.search(start[i], "pepper follow me") != None:
        print("successful")
    else:
        print("nothing")