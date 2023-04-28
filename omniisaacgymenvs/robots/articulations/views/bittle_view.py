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
import torch
from typing import Optional
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView
from sim4real.utils.rotation import tensor_quaternion_to_euler,tensor_get_euler_positions

# def quaternion_to_euler(carter_navigation_params.yaml, y, z, w):

#         import math
#         t0 = +2.0 * (w * carter_navigation_params.yaml + y * z)
#         t1 = +1.0 - 2.0 * (carter_navigation_params.yaml * carter_navigation_params.yaml + y * y)
#         X = math.degrees(math.atan2(t0, t1))

#         t2 = +2.0 * (w * y - z * carter_navigation_params.yaml)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         Y = math.degrees(math.asin(t2))

#         t3 = +2.0 * (w * z + carter_navigation_params.yaml * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         Z = math.degrees(math.atan2(t3, t4))

#         return X, Y, Z
import math
import numpy as np

#roll：绕x轴
#pitch：绕y轴
#yaw：绕z轴

# def quaternion_to_euler(w, carter_navigation_params.yaml, y, z):
#     #print(carter_navigation_params.yaml)
#     t0 = +2.0 * (w * carter_navigation_params.yaml + y * z)
#     t1 = +1.0 - 2.0 * (carter_navigation_params.yaml * carter_navigation_params.yaml + y * y)
#     #print(t1)
#     roll = torch.atan2(t0, t1)
#     #print("roll:",roll)
#     t2 = +2.0 * (w * y - z * carter_navigation_params.yaml)
#     # t2 = +1.0 if t2 > +1.0 else t2
#     # t2 = -1.0 if t2 < -1.0 else t2
#     t2 = torch.where(t2 > +1.0, +1.0, t2)
#     t2 = torch.where(t2 < -1.0, -1.0, t2)
#     pitch = torch.asin(t2)
#     t3 = +2.0 * (w * z + carter_navigation_params.yaml * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw = torch.atan2(t3, t4)
#     #print(yaw,pitch,roll)
#     return torch.stack([roll, pitch, yaw],dim=0).T

class BittleView(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        name: Optional[str] = "BittleView",
        track_contact_forces=False,
        prepare_contact_sensors=False
    ) -> None:
        """[summary]
        """

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False
        )
        self._knees = RigidPrimView(prim_paths_expr="/World/envs/.*/bittle/.*_knee_*",
            name="knees_view", reset_xform_properties=False, track_contact_forces=track_contact_forces, prepare_contact_sensors=prepare_contact_sensors)
        self._base = RigidPrimView(prim_paths_expr="/World/envs/.*/bittle/base_frame_link",
            name="base_view", reset_xform_properties=False, track_contact_forces=track_contact_forces, prepare_contact_sensors=prepare_contact_sensors)

    def get_knee_transforms(self):
        return self._knees.get_world_poses()

    def is_knee_below_threshold(self, threshold, ground_heights=None):
        knee_pos, _ = self._knees.get_world_poses()
        knee_heights = knee_pos.view((-1, 4, 3))[:, :, 2]
        if ground_heights is not None:
            knee_heights -= ground_heights
        #print(knee_heights)
        return (knee_heights[:, 0] < threshold) | (knee_heights[:, 1] < threshold) | (knee_heights[:, 2] < threshold) | (knee_heights[:, 3] < threshold)

    def is_base_below_threshold(self, threshold, ground_heights):
        base_pos, _ = self._base.get_world_poses()
        base_heights = base_pos[:, 2]
        base_heights -= ground_heights
        return (base_heights[:] < threshold)

    def is_orientation_below_threshold(self,threshold):
        _, base_orientation = self._base.get_world_poses()
        x = base_orientation[:, 1]
        y = base_orientation[:, 2]
        z = base_orientation[:, 3]
        w = base_orientation[:, 0]
        #print("orient:",carter_navigation_params.yaml,y)
        ang = tensor_quaternion_to_euler(w,x,y,z)
        #print("ang:",ang)
        return (ang[:,1] > threshold) | (ang[:,1] < -1 * threshold) | (ang[:,0] > threshold) | (ang[:,0] < -1* threshold)

    def get_euler_positions(self):
        torso_position, torso_rotation = self.get_world_poses(clone=False)
        # carter_navigation_params.yaml = torso_rotation[:, 1]
        # y = torso_rotation[:, 2]
        # z = torso_rotation[:, 3]
        # w = torso_rotation[:, 0]
        ang = torch.Tensor(tensor_get_euler_positions(torso_rotation))
        return ang