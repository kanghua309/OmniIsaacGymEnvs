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

lower_limits = np.zeros(8)
upper_limits = np.zeros(8)

for i,joint in enumerate(JOINTS):
    dof_ptr = dc.find_articulation_dof(art, joint)
    prop = dc.get_dof_properties(dof_ptr)
    lower_limits[i] = prop.lower
    upper_limits[i] = prop.upper

print("lower_limits:",lower_limits)
print("upper_limits:",upper_limits)


# for joint, pos in zip(JOINTS, default_dof_pos):
#     dof_ptr = dc.find_articulation_dof(art, joint)
#     print("joint:", joint, pos, dof_ptr)
#     # This should be called each frame of simulation if state on the articulation is being changed.
#     dc.wake_up_articulation(art)
#     # Set joint position target
#     dc.set_dof_position_target(dof_ptr, pos)

# transform = dc.get_rigid_body_pose(body)
# print("torso_rotation:", transform.r)
# print("torse_rotation's rotaion:",torch.Tensor(transform.r))
# torso_rotation = torch.Tensor([transform.r.w,transform.r.x, transform.r.y, transform.r.z]) #FIX IT w index change
# torso_rotation = torch.unsqueeze(torso_rotation, 0)
# print("torse_rotation's rotaion1:",torso_rotation)

# # 获得body的角速度和线速度？，return np
# velocity = torch.unsqueeze(torch.Tensor(dc.get_rigid_body_linear_velocity(body)),0)
# ang_velocity = torch.unsqueeze(torch.Tensor(dc.get_rigid_body_angular_velocity(body)),0)
# print("velocity & ang_velocity:", velocity, ang_velocity)
# print("velocity's ang_velocity:",torch.Tensor(velocity),torch.Tensor(ang_velocity))
# _gravity_vec = torch.unsqueeze(torch.Tensor(gravity_vec),0)
# print("gravity_vec:",_gravity_vec)

# # 获得关节的位置和速度，return np
# dof_pos = cur_dof_pos
# dof_vel = cur_dof_vel
# print("dof_pos & dof_vel:", dof_pos, dof_vel)


# # rotation * 速度的到实际base 线速度？ 数据类型是否满足？
# # FIX IT must [[]]
# #
# base_lin_vel = torch.squeeze(quat_rotate_inverse(torso_rotation, velocity),0) * 2.0  # self.lin_vel_scale
# print("base_lin_vel:", base_lin_vel,base_lin_vel.type)
# base_ang_vel = torch.squeeze(quat_rotate_inverse(torso_rotation, ang_velocity),0) * 0.25  # self.ang_vel_scale
# print("base_ang_vel:", base_ang_vel)
# projected_gravity = torch.squeeze(quat_rotate(torso_rotation,_gravity_vec),0)


# print("projected_gravity:", projected_gravity)
# dof_pos_scaled = (dof_pos - default_dof_pos) * 1.0  # self.dof_pos_scale
# dof_vel_scaled = dof_vel * 0.25  # self.dof_vel_scale
# commands_scaled = np.array([0.0, 1.0, 0.0],dtype=np.float32) \
#                   * np.array([2.0, 2.0, 0.25],dtype=np.float32)
# print("commands_scaled:", commands_scaled,commands_scaled.dtype)
# print("blv type:",base_lin_vel.cpu().detach().numpy().dtype)
# print("dof_pos_scaled:",dof_pos_scaled.dtype)
# pre_actions = default_dof_pos
# print("pre_actions:", pre_actions,pre_actions.dtype)
# # 转numpy -- torch_tensor.cpu().detach().numpy()
# # obs = np.array([cart_pos, cart_vel, pole_pos, pole_vel], dtype=np.float32)
# # 合并多个array 到一个 np.concatenate((a, b), axis=None)
# # base_lin_vel,
# # base_ang_vel,
# # projected_gravity,
# # commands_scaled,
# # dof_pos_scaled,
# # dof_vel * self.dof_vel_scale,
# # self.actions,
# obs = np.concatenate((base_lin_vel.cpu().detach().numpy(),
#                       base_ang_vel.cpu().detach().numpy(),
#                       projected_gravity.cpu().detach().numpy(),
#                       commands_scaled,
#                       dof_pos_scaled,
#                       dof_vel_scaled,
#                       pre_actions), axis=None)
# print("obs----------------------------------------------- 0 :",obs)
##3=print("Obs:", obs.tostring())

def quaternion_to_euler(x, y, z, w):
    #print(x)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    #print(t1)
    roll = torch.atan2(t0, t1)
    #print("roll:",roll)
    t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    t2 = torch.where(t2 > +1.0, +1.0, t2)
    t2 = torch.where(t2 < -1.0, -1.0, t2)
    pitch = torch.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = torch.atan2(t3, t4)
    #print(yaw,pitch,roll)
    return torch.stack([roll, pitch, yaw],dim=0).T

def get_euler_positions(torso_rotation):
    x = torso_rotation[:, 1]
    y = torso_rotation[:, 2]
    z = torso_rotation[:, 3]
    w = torso_rotation[:, 0]
    ang = torch.Tensor(quaternion_to_euler(x, y, z, w))
    return ang


# https://quaternion.readthedocs.io/en/latest/

import asyncio
import socket
import time

acts = np.zeros(8, dtype=np.float32)
current_targets = np.zeros(8, dtype=np.float32)
joint_angles_history = np.zeros((8,8))

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
            acts = default_dof_pos - default_dof_pos # FIX IT
            current_targets = default_dof_pos.copy()
        else:
            acts = np.fromstring(data, np.float32) #It is diff action, not action
            print('[Recieved] {} {}'.format(acts, addr))

            _current_targets = current_targets + 13.5 * acts * 1/100
            current_targets[:] = np.clip(_current_targets,lower_limits,upper_limits)
            # current_targets = self.current_targets + self.action_scale * self.actions * self.dt
            # self.current_targets[:] = tensor_clamp(current_targets, self.bittle_dof_lower_limits,
            #                                        self.bittle_dof_upper_limits)
            print("current_targets:",current_targets)

        for idx, (joint, pos) in enumerate(zip(JOINTS, current_targets)):
            dof_ptr = dc.find_articulation_dof(art, joint)
            print("joint1:", idx, joint, pos, dof_ptr)
            # dof_state = dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
            # print("joint2:", idx, joint, pos, dof_state)
            # cur_dof_pos[idx] = dof_state.pos
            # cur_dof_vel[idx] = dof_state.vel
            # This should be called each frame of simulation if state on the articulation is being changed.
            dc.wake_up_articulation(art)
            # Set joint position target
            dc.set_dof_position_target(dof_ptr, pos)
            # dof_state = dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
            # print("joint2:", idx, joint, pos, dof_state)
            # cur_dof_pos[idx] = dof_state.pos
            # cur_dof_vel[idx] = dof_state.vel

        print("prepare send obs now")
        # 获得rotation,return tensor？
        transform = dc.get_rigid_body_pose(body)
        print("torso_rotation:", transform.r)
        print("torse_rotation's rotaion0:", torch.Tensor(transform.r))
        torso_rotation = torch.Tensor(
            [transform.r.w, transform.r.x, transform.r.y, transform.r.z])  # FIX IT w index change
        torso_rotation = torch.unsqueeze(torso_rotation, 0)
        # torso_rotation = torch.unsqueeze( transform.r, 0)
        print("torse_rotation's rotaion1:", torso_rotation)
        ratation_angs = get_euler_positions(torso_rotation)
        print("ratation_angs:", ratation_angs)

        # 获得body的角速度和线速度？，return np
        # velocity = torch.unsqueeze(torch.Tensor(dc.get_rigid_body_linear_velocity(body)), 0)
        # ang_velocity = torch.unsqueeze(torch.Tensor(dc.get_rigid_body_angular_velocity(body)), 0)
        # print("velocity & ang_velocity:", velocity, ang_velocity)
        # print("velocity's ang_velocity:", torch.Tensor(velocity), torch.Tensor(ang_velocity))
        _gravity_vec = torch.unsqueeze(torch.Tensor(gravity_vec), 0)
        print("gravity_vec:", _gravity_vec)
        # 获得关节的位置和速度，return np
        dof_pos = np.array(dc.get_articulation_dof_position_targets(art),dtype=np.float32)
        # dof_vel = np.array(dc.get_articulation_dof_velocity_targets(art),dtype=np.float32)
        print("dof_pos :", dof_pos)
        # rotation * 速度的到实际base 线速度？ 数据类型是否满足？
        # FIX IT must [[]]
        #
        # base_lin_vel = torch.squeeze(quat_rotate_inverse(torso_rotation, velocity), 0) * 2.0  # self.lin_vel_scale
        # print("base_lin_vel:", base_lin_vel)
        # base_ang_vel = torch.squeeze(quat_rotate_inverse(torso_rotation, ang_velocity), 0) * 0.25  # self.ang_vel_scale
        # print("base_ang_vel:", base_ang_vel)
        projected_gravity = torch.squeeze(quat_rotate(torso_rotation, _gravity_vec), 0)

        print("projected_gravity:", projected_gravity)
        dof_pos_scaled = (dof_pos - default_dof_pos) * 1.0  # self.dof_pos_scale
        # dof_vel_scaled = dof_vel * 0.05  # self.dof_vel_scale
        print("dof_pos_scaled:", dof_pos_scaled)
        # print("dof_vel_scaled:", dof_vel_scaled)
        commands_scaled = np.array([0.0, 1.0, 0.0], dtype=np.float32) \
                          * np.array([2.0, 2.0, 0.25], dtype=np.float32)
        print("commands_scaled:", commands_scaled)

        pre_actions = acts
        print("pre_actions:", pre_actions)

        joint_angles_history = np.append(joint_angles_history, dof_pos)
        joint_angles_history = np.delete(joint_angles_history, np.s_[0:8])
        print("joint_angles_history:",joint_angles_history)
        obs = np.concatenate((
                              commands_scaled,
                              ratation_angs.cpu().detach().numpy(),
                              projected_gravity.cpu().detach().numpy(),
                              dof_pos_scaled,
                              joint_angles_history,
                              pre_actions), axis=None)
        print("obs----------------------------------------------- 0\n:", obs)
        s.sendto(obs.tostring(), addr)
        await asyncio.sleep(1)  # must ， gui not block


addr = ('', 8080)
asyncio.ensure_future(my_task(addr))
