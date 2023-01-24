import math
import carb
import numpy as np
from omni.isaac.kit import SimulationApp
# This sample loads an articulation and prints its information
import omni
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.nucleus import get_assets_root_path

# start simulation
omni.timeline.get_timeline_interface().play()


dc = _dynamic_control.acquire_dynamic_control_interface()
# Get handle to articulation
art = dc.get_articulation("/cartpole")
if art == _dynamic_control.INVALID_HANDLE:
    print("*** '%s' is not an articulation" % "/cartpole")


import asyncio
import socket
import time
async def my_task():
    print(f"my task begin")
    # # Simulate robot for a fixed number of frames and specify a joint position target
    for i in [1.,1.,0.7100,0.3972,-0.1217,-0.0897,-0.0149,0.1657,0.2411,-0.5281,-0.1649,-0.2994,0.0115,-0.6716,0.1800,0.0545,-0.2786]:
        try:
            dof_ptr = dc.find_articulation_dof(art, "cartJoint")
            dc.wake_up_articulation(art)
            dc.set_dof_effort(dof_ptr,i)
        except Exception as e:
            print(str(e))
        print("force:",i*400)
        #await asyncio.sleep(0.01)  # must ， gui not block

asyncio.ensure_future(my_task())