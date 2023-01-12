import copy
import time

import numpy as np

base = np.array([ 0,  # left_back_shoulder_joint
                    0,  # left_back_knee_joint
                    0,  # left_front_shoulder_joint
                    0,  # left_front_knee_joint
                    0,  # right_back_shoulder_joint
                    0,  # right_back_knee_joint
                    0,  # right_front_shoulder_joint
                    0], # right_front_knee_joint
                    dtype= np.float32)

#np.save("bittle_posvec.npy",base)
#posvec = np.load("bittle_posvec.npy")
#print(posvec)


stand = np.array([ 0.7,  # left_back_shoulder_joint     -:fwd   +:back
                  0.,  # left_back_knee_joint           -:up    +:down
                  0.7,  # left_front_shoulder_joint     -:fwd   +:back
                  0.,  # left_front_knee_joint
                  -0.7,  # right_back_shoulder_joint    -:back      +:fwd
                  -0.,  # right_back_knee_joint         -:cw/down   +:ccw/up
                  -0.7,  # right_front_shoulder_joint   -:back      +:fwd
                  -0.], # right_front_knee_joint        -:cw/down   +:ccw/up
                  dtype= np.float32)

tail  = np.array([ 0.5,  # left_back_shoulder_joint
                  0.5,  # left_back_knee_joint
                  0.7,  # left_front_shoulder_joint
                  0.3,  # left_front_knee_joint
                  -0.5,  # right_back_shoulder_joint
                  -0.5,  # right_back_knee_joint
                  -0.7,  # right_front_shoulder_joint
                  -0.3], # right_front_knee_joint
                 dtype= np.float32)

#[0,-75,0,-75,0,75,0,75]
crouch = np.array([ 0,  # left_back_shoulder_joint
                    -75,  # left_back_knee_joint
                    0,  # left_front_shoulder_joint
                    -75,  # left_front_knee_joint
                    0,  # right_back_shoulder_joint
                    75,  # right_back_knee_joint
                    0,  # right_front_shoulder_joint
                    75], # right_front_knee_joint
                  dtype=np.float32)


increment = 1

import random
import socket
import time

address = ('127.0.0.1', 8080)  # 服务端地址和端口
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def move(currentvec,targetvec, increment = 0.01):
    posvec = copy.deepcopy(currentvec)
    for i in range(60):
        #currentvec = np.load("bittle_posvec.npy")
        #print(currentvec.dtype,targetvec.dtype)
        for idx, (cpos, tpos) in enumerate(zip(posvec, targetvec)):
            diff = tpos - cpos
            if abs(diff) > increment:
                posvec[idx] = cpos + (np.sign(diff) + increment)
                print(cpos,(np.sign(diff) + increment))
            else:
                posvec[idx] = tpos
            print("posvec:",posvec)
            #send it
            s.sendto(posvec.tostring(),address)
            data, addr = s.recvfrom(4096)
            print('[Recieved] {} {}'.format(np.fromstring(data,np.float32),addr))


        #np.save("bittle_posvec.npy",posvec)
        time.sleep(1/60)



move(base,crouch)
#move(tail)