

import copy

import numpy as np
numFramevt = 21
vetVt = np.array([
    [57, 43, 60, 47, -18, 7, -17, 7],
    [50, 43, 53, 47, -5, 7, -5, 7],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [43, 47, 47, 50, 7, 0, 7, 0],
    [43, 54, 47, 58, 7, -13, 7, -13],
    [43, 60, 47, 65, 7, -25, 7, -25],
    [43, 66, 47, 71, 7, -35, 7, -35],
    [43, 63, 47, 67, 7, -30, 7, -29],#10
    [43, 57, 47, 60, 7, -18, 7, -17],
    [43, 50, 47, 53, 7, -5, 7, -5],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [43, 43, 47, 47, 7, 7, 7, 7],
    [47, 43, 50, 47, 0, 7, 0, 7],
    [54, 43, 58, 47, -13, 7, -13, 7],
    [60, 43, 65, 47, -25, 7, -25, 7],
    [66, 43, 71, 47, -35, 7, -35, 7],
    [63, 43, 67, 47, -30, 7, -29, 7],
],dtype=np.float32)

import random
import socket
import time

dest_addr = ('127.0.0.1', 8080)  # 服务端地址和端口
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Init
j=0
vectorPre = np.array([0, 0, 0, 0, 0, 0, 0, 0])
vectorPre =vectorPre/180*3.14
numFrame = numFramevt
vecTarget = vetVt
timeUpdate = 1/60

while True:
    i = j % numFrame
    vecCur = copy.deepcopy(vecTarget[i])
    vecCur[1] = -vecCur[1]
    vecCur[2] = -vecCur[2]
    vecCur[5] = -vecCur[5]
    vecCur[6] = -vecCur[6]
    vecCur = vecCur/180*3.14
    stepSmooth = 2
    for k in range(stepSmooth):
        diff = vecCur - vectorPre
        vecOut = vectorPre + (k+1)/stepSmooth * diff
        vectorPre = vecOut
        #strVec = ','.join(str(i) for i in vecOut)
        print(vecOut.tostring())
        udp_socket.sendto(vecOut.tostring(),dest_addr)
        time.sleep(timeUpdate)
    j  = j + 1
    if  j == 21:
   		 break
udp_socket.close()
