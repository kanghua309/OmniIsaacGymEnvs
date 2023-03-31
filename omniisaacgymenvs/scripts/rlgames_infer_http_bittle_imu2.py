# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import json
import math
import datetime
import os
import random
import socket
import time
import torch
import numpy as np
import requests

from omniisaacgymenvs.utils.hydra_cfg.hydra_utils import *
from omniisaacgymenvs.utils.hydra_cfg.reformat import omegaconf_to_dict, print_dict
from omniisaacgymenvs.utils.rlgames.rlgames_utils import RLGPUAlgoObserver, RLGPUEnv
from omniisaacgymenvs.utils.task_util import initialize_task
from omniisaacgymenvs.utils.config_utils.path_utils import retrieve_checkpoint_path
from omniisaacgymenvs.envs.vec_env_rlgames import VecEnvRLGames

import hydra
from omegaconf import DictConfig

from rl_games.common import env_configurations, vecenv
from rl_games.torch_runner import Runner

# import omni
# from omni.isaac.core.utils.torch.rotations import *
from sim4real.utils.rotation import point_rotation_by_quaternion, euler_from_quaternion

#
# from ahrs.filters import Madgwick
# madgwick = Madgwick()

#
# def get_quaternion_from_euler(roll, pitch, yaw):
#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     return [ qw, qx, qy, qz ]
#
# def euler_from_quaternion(x, y, z, w):
#     """
#     Convert a quaternion into euler angles (roll, pitch, yaw)
#     roll is rotation around x in radians (counterclockwise)
#     pitch is rotation around y in radians (counterclockwise)
#     yaw is rotation around z in radians (counterclockwise)
#     """
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll_x = math.atan2(t0, t1)
#
#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch_y = math.asin(t2)
#
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw_z = math.atan2(t3, t4)
#
#     return [roll_x, pitch_y, yaw_z]  # in radians
#
# def quaternion_mult(q,r):
#     return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
#             r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
#             r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
#             r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]
#
# def point_rotation_by_quaternion(point,q):
#     r = [0]+point
#     q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
#     return quaternion_mult(quaternion_mult(q,r),q_conj)[1:]

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

lower_limits = np.deg2rad(np.array([-40,   -40,    -40,    -40,    -60,    -60,    0,  0]))
upper_limits = np.deg2rad(np.array([40,    40,   40,  40, 0,  0,  60, 60]))



device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print("Device:", device)
gravity_vec = [0.0, 0.0, -1.0]  # FIX IT
address = ('127.0.0.1', 8080)  # 服务端地址和端口

driver_location ='http://192.168.1.10/drive'
imu_location ='http://192.168.1.10/attitude'

class RLGTrainer():
    def __init__(self, cfg, cfg_dict):
        self.cfg = cfg
        self.cfg_dict = cfg_dict

    def launch_rlg_hydra(self, env):
        # `create_rlgpu_env` is environment construction function which is passed to RL Games and called internally.
        # We use the helper function here to specify the environment config.
        self.cfg_dict["task"]["test"] = self.cfg.test

        # register the rl-games adapter to use inside the runner
        vecenv.register('RLGPU',
                        lambda config_name, num_actors, **kwargs: RLGPUEnv(config_name, num_actors, **kwargs))
        env_configurations.register('rlgpu', {
            'vecenv_type': 'RLGPU',
            'env_creator': lambda **kwargs: env
        })
        self.rlg_config_dict = omegaconf_to_dict(self.cfg.train)

    def run(self,env):
        # create runner and set the settings
        print("rlg_config_dict:",self.rlg_config_dict)
        runner = Runner(RLGPUAlgoObserver())
        runner.load(self.rlg_config_dict)
        runner.reset()

        # dump config dict
        experiment_dir = os.path.join('runs', self.cfg.train.params.config.name)
        os.makedirs(experiment_dir, exist_ok=True)
        with open(os.path.join(experiment_dir, 'config.yaml'), 'w') as f:
            f.write(OmegaConf.to_yaml(self.cfg))

        # runner.run({
        #     'train': not self.cfg.test,
        #     'play': self.cfg.test,
        #     'checkpoint': self.cfg.checkpoint,
        #     'sigma': None
        # })
        print("cfg:",self.cfg)
        print("=======================:",self.cfg.checkpoint)
        agent = runner.create_player()
        agent.restore(self.cfg.checkpoint)

        #obs = env.reset()
        num_steps = 0
        # s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #连接服务端
        # addr = ("127.0.0.1", 8080)
        # s.sendto(b'', addr) #client send first must
        current_targets = default_dof_pos.copy()
        acts = default_dof_pos - default_dof_pos
        joint_angles_history = np.zeros((8, 8), dtype=np.float32)
        #PQ = [1.0, 0.0, 0.0, 0.0]
        while(True):

            #data, addr = s.recvfrom(4096)
            #euler = np.fromstring(data, np.float32)
            response = requests.get(url=imu_location)
            print(response.status_code)
            imu_data = json.loads(response.text)
            reading = imu_data
            print("imu ratation:",
                      reading["W"], reading["X"], reading["Y"],reading["Z"]
                      )
            # NQ = madgwick.updateIMU(PQ,
            #                         gyr=np.array([reading[-1]["GX"]/100_000, reading[-1]["GY"]/100_000,
            #                                           reading[-1]["GZ"]/100_000]),
            #                         acc=np.array([reading[-1]["AX"]/100_000, reading[-1]["AY"]/100_000,
            #                                           reading[-1]["AZ"]/100_000]))
            # PQ = NQ
            NQ = [reading["W"], reading["X"], reading["Y"],reading["Z"]]
            euler = euler_from_quaternion(NQ[0],NQ[1],NQ[2],NQ[3])
            print("Receive Obs Raw:",euler)
            commands_scaled = np.array([0.0, -1.0, 0.0], dtype=np.float32) \
                              * np.array([2.0, 2.0, 0.25], dtype=np.float32)
            print("commands_scaled:", commands_scaled)
            _gravity_vec = gravity_vec
            torso_rotation = NQ #get_quaternion_from_euler(euler[0],euler[1],euler[2])
            print("torso_rotation:",torso_rotation)
            print("_gravity_vec:",_gravity_vec)
            projected_gravity = point_rotation_by_quaternion(_gravity_vec,torso_rotation)
            print("projected_gravity:",projected_gravity)

            _current_targets = current_targets + 13.5 * acts * 1 / 100
            current_targets = np.clip(_current_targets, lower_limits, upper_limits)
            print("current_targets:",current_targets)
            dof_pos = current_targets
            dof_pos_scaled = (dof_pos - default_dof_pos) * 1.0  # self.dof_pos_scale
            print("dof_pos_scaled:",dof_pos_scaled)
            pre_actions = acts
            print("pre_actions:", pre_actions)

            joint_angles_history = np.append(joint_angles_history, dof_pos)
            joint_angles_history = np.delete(joint_angles_history, np.s_[0:8])
            print("joint_angles_history:", joint_angles_history)
            obs = np.concatenate((
                              commands_scaled,
                              euler,
                              projected_gravity,
                              dof_pos_scaled,
                              joint_angles_history,
                              pre_actions), axis=None)

            obs = torch.from_numpy(obs.astype(np.float32)).to(device).clone()
            #print("Receive Obs 1:",obs)
            # 模型训练obs 是env num个，所以这里obs这里要升纬度 [[obs]]
            obs = torch.unsqueeze(obs, 0)
            print("Receive Obs:",obs)
            _acts = agent.get_action(obs)
            # FIX IT acts shape is envs.action_space
            _acts = _acts.to('cpu').detach().numpy().copy()
            # print("Action 1:",acts,env.action_space)
            # acts = acts.flatten()
            print("Action :",_acts)
            acts = _acts
            param = {
                'la1': round(np.rad2deg(acts[0])),
                'ra1': round(np.rad2deg(acts[1])),
                'ra2': round(np.rad2deg(acts[2])),
                'la2': round(np.rad2deg(acts[3])),

                'll1': -round(np.rad2deg(acts[4])),
                'rl1': -round(np.rad2deg(acts[5])),
                'rl2': -round(np.rad2deg(acts[6])),
                'll2': -round(np.rad2deg(acts[7])),
            }
            response = requests.get(url=driver_location, params=param)
            #s.sendto(acts.tostring(), addr)
            num_steps += 1
            print('Num steps: ', num_steps)


@hydra.main(config_name="config", config_path="../cfg")
def parse_hydra_configs(cfg: DictConfig):

    time_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    headless = cfg.headless
    env = VecEnvRLGames(headless=headless, sim_device=cfg.device_id)

    # ensure checkpoints can be specified as relative paths
    if cfg.checkpoint:
        cfg.checkpoint = retrieve_checkpoint_path(cfg.checkpoint)
        if cfg.checkpoint is None:
            quit()

    cfg_dict = omegaconf_to_dict(cfg)
    print_dict(cfg_dict)

    # sets seed. if seed is -1 will pick a random one
    from omni.isaac.core.utils.torch.maths import set_seed
    cfg.seed = set_seed(cfg.seed, torch_deterministic=cfg.torch_deterministic)
    cfg_dict['seed'] = cfg.seed

    task = initialize_task(cfg_dict, env)

    rlg_trainer = RLGTrainer(cfg, cfg_dict)
    rlg_trainer.launch_rlg_hydra(env)
    rlg_trainer.run(env)

    env.close()


if __name__ == '__main__':
    parse_hydra_configs()
