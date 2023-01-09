import copy
import time

import numpy as np

currentvec = np.array([ 0,  # left_back_shoulder_joint
                    0,  # left_back_knee_joint
                    0,  # left_front_shoulder_joint
                    0,  # left_front_knee_joint
                    0,  # right_back_shoulder_joint
                    0,  # right_back_knee_joint
                    0,  # right_front_shoulder_joint
                    0], # right_front_knee_joint
                    dtype= np.float32)

np.save("bittle_posvec.npy",currentvec)
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

crouch = np.array([ 1.0,  # left_back_shoulder_joint
                    -0.9,  # left_back_knee_joint
                    1.0,  # left_front_shoulder_joint
                    -0.9,  # left_front_knee_joint
                    -1.0,  # right_back_shoulder_joint
                    0.9,  # right_back_knee_joint
                    -1.0,  # right_front_shoulder_joint
                    0.9], # right_front_knee_joint
                  dtype=np.float32)


increment = 0.01

def move(targetvec, increment = 0.01):
    for i in range(60):
        currentvec = np.load("bittle_posvec.npy")
        #print(currentvec.dtype,targetvec.dtype)
        posvec = copy.deepcopy(currentvec)
        for idx, (cpos, tpos) in enumerate(zip(currentvec, targetvec)):
            diff = tpos - cpos
            if abs(diff) > increment:
                posvec[idx] = cpos + (np.sign(diff) + increment)
                # print(cpos,(np.sign(diff) + increment))
            else:
                posvec[idx] = tpos
            print(posvec)
        np.save("bittle_posvec.npy",posvec)
        time.sleep(1/60)


move(crouch)
move(tail)