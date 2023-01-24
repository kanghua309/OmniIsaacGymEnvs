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

cart_dof_ptr = dc.find_articulation_dof(art, "cartJoint")
pole_dof_ptr = dc.find_articulation_dof(art, "poleJoint")

import asyncio
import socket
import time

async def my_task(host):
    print(f"my task begin")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(0)
    s.bind(host)
    for i in range(10000):
        if i % 100 == 0:
            print("wait :", i)
        try:
            data, addr = s.recvfrom(4096)
        except:
            await asyncio.sleep(0.01)  # must,gui not block
            continue

        print("receive data:", data, addr)
        if data == b'':
            print("client comming on ... ")
        else:
            acts = np.fromstring(data, np.float32)
            print('[Recieved] {} {}'.format(acts, addr))

            dc.wake_up_articulation(art)
            dc.set_dof_effort(cart_dof_ptr,acts[0]*400)
            print("force:",acts[0])

        print("send obs now")
        cart_pos = dc.get_dof_position(cart_dof_ptr)
        cart_vel = dc.get_dof_velocity(cart_dof_ptr)
        pole_pos = dc.get_dof_position(pole_dof_ptr)
        pole_vel = dc.get_dof_velocity(pole_dof_ptr)
        obs = np.array([cart_pos, cart_vel, pole_pos, pole_vel], dtype=np.float32)
        print("Obs:",obs.tostring())
        s.sendto(obs.tostring(), addr)
        await asyncio.sleep(0.01)  # must ， gui not block

addr = ('', 8080)
asyncio.ensure_future(my_task(addr))