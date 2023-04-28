import math
import carb
import numpy as np
from omni.isaac.kit import SimulationApp

# This sample loads an articulation and prints its information
import omni
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.torch.rotations import *

import torch
from torch import cpu

###########################################################################

JOINTS = [
    "left_back_shoulder_joint",
    "left_front_shoulder_joint",
    "right_back_shoulder_joint",
    "right_front_shoulder_joint",
    "left_back_knee_joint",
    "left_front_knee_joint",
    "right_back_knee_joint",
    "right_front_knee_joint",
]

# start simulation
omni.timeline.get_timeline_interface().play()
gravity_vec = np.array([0.0, 0.0, -1.0])  # FIX IT
default_dof_pos = np.array([
    0.523,
    0.623,
    -0.523,
    -0.623,
    -1.047,
    -1.047,
    1.047,
    1.047,
], dtype=np.float32)

cur_dof_pos = np.zeros(8, dtype=np.float32)
cur_dof_vel = np.zeros(8, dtype=np.float32)

###########################################################################

print(cur_dof_pos, cur_dof_vel)
dc = _dynamic_control.acquire_dynamic_control_interface()
# Get handle to articulation
art = dc.get_articulation("/bittle")
if art == _dynamic_control.INVALID_HANDLE:
    print("*** '%s' is not an articulation" % "/bittle")
    exit(-1)

body = dc.find_articulation_body(art, "base_frame_link")
if body == _dynamic_control.INVALID_HANDLE:
    print("*** '%s' is not an articulation body" % "base_frame_link")
    exit(-1)

print("art & body:", art, body)

###########################################################################

lower_limits = np.zeros(8)
upper_limits = np.zeros(8)

for i, joint in enumerate(JOINTS):
    dof_ptr = dc.find_articulation_dof(art, joint)
    prop = dc.get_dof_properties(dof_ptr)
    lower_limits[i] = prop.lower
    upper_limits[i] = prop.upper

print("lower_limits:", lower_limits)
print("upper_limits:", upper_limits)

###########################################################################

# FIX IT - 动态加入imu ？？
import omni.kit.commands
from omni.isaac.sensor import _sensor
import carb
from pxr import Gf, UsdGeom

stage = omni.usd.get_context().get_stage()
curr_prim = stage.GetPrimAtPath("/bittle/base_frame_link")
print(curr_prim.GetChildren())
is_need_create_imu = True
for prim in curr_prim.GetChildren():
    if prim.GetPrimPath() == "/bittle/base_frame_link/sensor":
        is_need_create_imu = False
        break

if is_need_create_imu:
    result, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="/sensor",
        parent="/bittle/base_frame_link",
        sensor_period=1 / 500.0,
        translation=Gf.Vec3d(0, 0, 0),
        orientation=Gf.Quatd(1, 0, 0, 0),
        visualize=True,
    )
_is = _sensor.acquire_imu_sensor_interface()

###########################################################################
# imu 滤波器
from ahrs.filters import Madgwick
madgwick = Madgwick()

###########################################################################

from sim4real.utils.rotation import tensor_get_euler_positions


#
# def quaternion_to_euler(carter_navigation_params.yaml, y, z, w):
#     # print(carter_navigation_params.yaml)
#     t0 = +2.0 * (w * carter_navigation_params.yaml + y * z)
#     t1 = +1.0 - 2.0 * (carter_navigation_params.yaml * carter_navigation_params.yaml + y * y)
#     # print(t1)
#     roll = torch.atan2(t0, t1)
#     # print("roll:",roll)
#     t2 = +2.0 * (w * y - z * carter_navigation_params.yaml)
#     # t2 = +1.0 if t2 > +1.0 else t2
#     # t2 = -1.0 if t2 < -1.0 else t2
#     t2 = torch.where(t2 > +1.0, +1.0, t2)
#     t2 = torch.where(t2 < -1.0, -1.0, t2)
#     pitch = torch.asin(t2)
#     t3 = +2.0 * (w * z + carter_navigation_params.yaml * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw = torch.atan2(t3, t4)
#     # print(yaw,pitch,roll)
#     return torch.stack([roll, pitch, yaw], dim=0).T
#
#
# def get_euler_positions(torso_rotation):
#     carter_navigation_params.yaml = torso_rotation[:, 1]
#     y = torso_rotation[:, 2]
#     z = torso_rotation[:, 3]
#     w = torso_rotation[:, 0]
#     ang = torch.Tensor(quaternion_to_euler(carter_navigation_params.yaml, y, z, w))
#     return ang


import asyncio
import socket
import time

async def my_task(host):
    print(f"my task begin")
    joint_angles_history = np.zeros((8, 8), dtype=np.float32)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(0)
    s.bind(host)
    for i in range(5000):
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
            acts = default_dof_pos - default_dof_pos  # FIX IT
            current_targets = default_dof_pos.copy()
            PQ = [1.0, 0.0, 0.0, 0.0]
        else:
            acts = np.fromstring(data, np.float32)  # It is diff action, not action
            print('[Recieved] {} {}'.format(acts, addr))
            _current_targets = current_targets + 13.5 * acts * 1 / 100
            current_targets[:] = np.clip(_current_targets, lower_limits, upper_limits)
            print("current_targets:", current_targets)

        for idx, (joint, pos) in enumerate(zip(JOINTS, current_targets)):
            dof_ptr = dc.find_articulation_dof(art, joint)
            print("joint1:", idx, joint, pos, dof_ptr)
            # This should be called each frame of simulation if state on the articulation is being changed.
            dc.wake_up_articulation(art)
            # Set joint position target
            dc.set_dof_position_target(dof_ptr, pos)

        print("prepare send obs now")
        # 获得rotation,return tensor？
        reading = _is.get_sensor_readings("/bittle/base_frame_link/sensor")
        if reading.shape[0]:
            print("imu ratation:",reading[-1]["orientation"],
                  reading[-1]["lin_acc_x"],reading[-1]["lin_acc_y"],reading[-1]["lin_acc_z"],
                  reading[-1]["ang_vel_x"], reading[-1]["ang_vel_y"], reading[-1]["ang_vel_z"],
                  )
            NQ = madgwick.updateIMU(PQ,
                                    gyr=np.array([reading[-1]["ang_vel_x"], reading[-1]["ang_vel_y"], reading[-1]["ang_vel_z"]]),
                                    acc=np.array([reading[-1]["lin_acc_x"],reading[-1]["lin_acc_y"],reading[-1]["lin_acc_z"]]))
            PQ = NQ

        print("torse_rotation's rotaion0:", torch.Tensor(NQ))
        torso_rotation = torch.Tensor(
            [NQ[0],NQ[1],NQ[2],NQ[3]])  # FIX IT w index change  w is idx first
        torso_rotation = torch.unsqueeze(torso_rotation, 0)
        # torso_rotation = torch.unsqueeze( transform.r, 0)
        print("torse_rotation's rotaion1:", torso_rotation)
        ratation_angs = tensor_get_euler_positions(torso_rotation)
        print("ratation_angs:", ratation_angs)
        simple_obs = np.concatenate((ratation_angs.cpu().detach().numpy()), axis=None)
        print("obs----------------------------------------------- 0\n:", simple_obs)
        s.sendto(simple_obs.tostring(), addr)
        await asyncio.sleep(0.05)  # must ， gui not block


addr = ('', 8080)
asyncio.ensure_future(my_task(addr))
