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
gravity_vec = np.array([0.0, 0.0, -1.0])  # FIX IT
default_dof_pos = np.array([
    0.4,
    1.0,
    -0.4,
    -1.0,
    -1.0,
    -1.2,
    1.0,
    1.2,
])

dc = _dynamic_control.acquire_dynamic_control_interface()
# Get handle to articulation
art = dc.get_articulation("/bittle")
if art == _dynamic_control.INVALID_HANDLE:
    print("*** '%s' is not an articulation" % "/bittle")

import asyncio
import socket
import time
async def my_task(host):
    print(f"my task begin")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(0)
    s.bind(host)
    for i in range(1000):
        if i % 100 == 0:
            print("wait :", i)
        try:
            data, addr = s.recvfrom(4096)
        except:
            await asyncio.sleep(0.01)  # must,gui not block
            continue
        print("receive data:", data, addr)
        acts = [0,0,0,0,0,0,0,0] #FIX IT
        if data == b'':
            print("client comming on ... ")
        else:
            acts = np.fromstring(data, np.float32)
            print('[Recieved] {} {}'.format(acts, addr))
            for joint, pos in zip(JOINTS, acts):
                dof_ptr = dc.find_articulation_dof(art, joint)
                print("joint:", joint, pos, dof_ptr)
                # This should be called each frame of simulation if state on the articulation is being changed.
                dc.wake_up_articulation(art)
                # Set joint position target
                dc.set_dof_position_target(dof_ptr, pos)

        print("send obs now")
        '''
        torso_position, torso_rotation = self._bittles.get_world_poses(clone=False)
        root_velocities = self._bittles.get_velocities(clone=False)
        dof_pos = self._bittles.get_joint_positions(clone=False)
        dof_vel = self._bittles.get_joint_velocities(clone=False)

        velocity = root_velocities[:, 0:3]
        ang_velocity = root_velocities[:, 3:6]

        base_lin_vel = quat_rotate_inverse(torso_rotation, velocity) * self.lin_vel_scale
        base_ang_vel = quat_rotate_inverse(torso_rotation, ang_velocity) * self.ang_vel_scale
        projected_gravity = quat_rotate(torso_rotation, self.gravity_vec)
        dof_pos_scaled = (dof_pos - self.default_dof_pos) * self.dof_pos_scale

        commands_scaled = self.commands * torch.tensor(
            [self.lin_vel_scale, self.lin_vel_scale, self.ang_vel_scale],
            requires_grad=False,
            device=self.commands.device,
        )
        
        '''


        # cart_pos = dc.get_dof_position(cart_dof_ptr)
        # cart_vel = dc.get_dof_velocity(cart_dof_ptr)
        # pole_pos = dc.get_dof_position(pole_dof_ptr)
        # pole_vel = dc.get_dof_velocity(pole_dof_ptr)
        #
        # obs = np.array([cart_pos, cart_vel, pole_pos, pole_vel], dtype=np.float32)

        '''    
        # normalization
        linearVelocityScale: 2.0
        angularVelocityScale: 0.25
        dofPositionScale: 1.0
        dofVelocityScale: 0.05
        '''
        # gravity_vec = torch.tensor([0.0, 0.0, -1.0], device=cpu) #FIX IT
        # default_dof_pos = np.array([
        #    0.4,
        #    1.0,
        #    -0.4,
        #    -1.0,
        #    -1.0,
        #    -1.2,
        #    1.0,
        #    1.2,
        # ])



        torso_position, torso_rotation = dc.get_world_poses(clone=False)
        velocity = dc.get_angular_velocity()
        ang_velocity = dc.get_linear_velocity()
        dof_pos = dc.get_joint_positions()
        dof_vel = dc.get_joint_velocities()
        base_lin_vel = quat_rotate_inverse(torso_rotation, velocity) * 2.0          #self.lin_vel_scale
        base_ang_vel = quat_rotate_inverse(torso_rotation, ang_velocity) * 0.25     #self.ang_vel_scale
        projected_gravity = quat_rotate(torso_rotation, gravity_vec)
        dof_pos_scaled = (dof_pos - default_dof_pos) * 1.0                          #self.dof_pos_scale
        dof_vel_scaled = dof_vel * 0.25                                             #self.dof_vel_scale
        commands_scaled = np.array([0.0,1.0,0.0]) \
                          * np.array([2.0, 2.0, 0.25])
        pre_actions = acts
        #转numpy -- torch_tensor.cpu().detach().numpy()
        #obs = np.array([cart_pos, cart_vel, pole_pos, pole_vel], dtype=np.float32)
        #合并多个array 到一个 np.concatenate((a, b), axis=None)
        # base_lin_vel,
        # base_ang_vel,
        # projected_gravity,
        # commands_scaled,
        # dof_pos_scaled,
        # dof_vel * self.dof_vel_scale,
        # self.actions,

        obs = np.concatenate((base_lin_vel.cpu().detach().numpy(),
                        base_ang_vel.cpu().detach().numpy(),
                        projected_gravity.cpu().detach().numpy(),
                        commands_scaled,
                        dof_pos_scaled,
                        dof_vel_scaled,
                        pre_actions), axis=None)
        print("Obs:", obs.tostring())
        s.sendto(obs.tostring(), addr)
        await asyncio.sleep(0.01)  # must ， gui not block


addr = ('', 8080)
asyncio.ensure_future(my_task(addr))