import sys
import time
import carb
import numpy as np
from omni.isaac.kit import SimulationApp



# This sample loads an articulation and prints its information
import omni
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.nucleus import get_assets_root_path




JOINTS = [
    "left_back_shoulder_joint",
    "left_back_knee_joint",
    "left_front_shoulder_joint",
    "left_front_knee_joint",
    "right_back_shoulder_joint",
    "right_back_knee_joint",
    "right_front_shoulder_joint",
    "right_front_knee_joint",
]



# start simulation
omni.timeline.get_timeline_interface().play()


dc = _dynamic_control.acquire_dynamic_control_interface()
# Get handle to articulation
art = dc.get_articulation("/bittle")
if art == _dynamic_control.INVALID_HANDLE:
    print("*** '%s' is not an articulation" % "/bittle")

import asyncio
async def my_task():
    print(f"my task begin")
    # # Simulate robot for a fixed number of frames and specify a joint position target
    for i in range(100):
        try:
            #posvec = np.load("C:\\Users\\Administrator\\SynologyDrive\\sim4real\\sim2sim\\bittle_posvec.npy")
            posvec = np.array([0,-75,0,-75,0,75,0,75])
            posvec = posvec/180.0 * 3.14
            for joint, pos in zip(JOINTS, posvec):
                dof_ptr = dc.find_articulation_dof(art,joint)
                print("joint:",joint,pos,dof_ptr)
                #This should be called each frame of simulation if state on the articulation is being changed.
                dc.wake_up_articulation(art)
                #Set joint position target
                dc.set_dof_position_target(dof_ptr, pos)
        except Exception as e:
            print(str(e))
        await asyncio.sleep(0.1) #must ， gui not block

asyncio.ensure_future(my_task())