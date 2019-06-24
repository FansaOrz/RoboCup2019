#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
from tf import transformations
rospy.init_node("test")
predefined_position = {
    'table': [0.8722, -0.3192, 0, 0, 0.950170776273, 0.311729844444],
    "sofa1": [2.629, -0.37998, 0, 0, 0.435683814323, 0.900099779989],
    "tea table": [2.8597, -1.0975, 0, 0, 0.435683814323, 0.900099779989],
    "sofa2": [3.82847, -1.03030, 0, 0, 0.435683814323, 0.900099779989],
    "chair": [4.66809, -1.6237, 0, 0, 0.435683814323, 0.900099779989],
    "bed": [3.14724, -3.78763, 0, 0, -0.709890396828, 0.704312164094],
    "TV": [1.34616, -2.70090, 0, 0, -0.896545546577, 0.4429515582]
}
ori = {
    "1":[ 0.0,0.0, 0.424973256776, 0.905205905319],
    "2":[ 0.0,0.0,-0.0615786932214,0.998102231508],
    "3":[ 0.0,0.0,-0.339118199178,0.940743773291],
    "4":[ 0.0,0.0,-0.648651255858,0.761085769328],
    "5":[ 0.0,0.0,-0.913288289143,0.407313762244],
    "6":[ 0.0,0.0,-0.999999365247,0.00112672329273],
    "7":[ 0.0,0.0,0.94660077928,0.322408071653],
    "8":[ 0.0,0.0,0.720491321286,0.693463954328]
}
goal = [0,0,1.3]
for i in ori.keys():
    # print predefined_position[i][2]
    # print predefined_position[i][3]
    # print predefined_position[i][4]
    # print predefined_position[i][5]
    temp = transformations.euler_from_quaternion([ori[i][0], ori[i][1], ori[i][2], ori[i][3]])
    if (temp[2] > 0 and goal[2] > 0) or (temp[2]< 0 and goal[2] < 0):
        if abs(temp[2] - goal[2]) > 3.14 / 2:
            continue
        else:
            print i
    elif (temp[2] > 0 and goal[2] < 0) or (temp[2]< 0 and goal[2] > 0):
        if (6.28 - (abs(temp[2]) + abs(goal[2]))) < 3.14/2 or (abs(temp[2]) + abs(goal[2])) < 3.14/2:
            print i
        else:
            continue
