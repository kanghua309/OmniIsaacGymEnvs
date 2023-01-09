# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import sys
import carb
import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

# This sample loads an articulation and prints its information
import omni
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.nucleus import get_assets_root_path



stage = simulation_app.context.get_stage()
JOINTS = [
    "left_back_knee_joint",
    "left_front_shoulder_joint",
    "left_front_knee_joint",
    "right_back_shoulder_joint",
    "right_back_knee_joint",
    "right_front_shoulder_joint",
    "left_back_shoulder_joint",
    "right_front_knee_joint",
]
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "usd/bittle.usd"
omni.usd.get_context().open_stage(asset_path)
# start simulation
omni.timeline.get_timeline_interface().play()

# perform timestep
simulation_app.update()

dc = _dynamic_control.acquire_dynamic_control_interface()
# Get handle to articulation
art = dc.get_articulation("/bittle")
if art == _dynamic_control.INVALID_HANDLE:
    print("*** '%s' is not an articulation" % "/bittle")
else:
    # Print information about articulation
    root = dc.get_articulation_root_body(art)
    print(str("Got articulation handle %d \n" % art) + str("--- Hierarchy\n"))

    body_states = dc.get_articulation_body_states(art, _dynamic_control.STATE_ALL)
    print(str("--- Body states:\n") + str(body_states) + "\n")

    dof_states = dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
    print(str("--- DOF states:\n") + str(dof_states) + "\n")

    dof_props = dc.get_articulation_dof_properties(art)
    print(str("--- DOF properties:\n") + str(dof_props) + "\n")

# Simulate robot coming to a rest configuration
for i in range(100):
    simulation_app.update()

# Simulate robot for a fixed number of frames and specify a joint position target
while True:
    try:
        posvec = np.load("bittle_posvec.npy")
        for joint, pos in zip(JOINTS, posvec):
            dof_ptr = dc.find_articulation_dof(art,joint)
            # This should be called each frame of simulation if state on the articulation is being changed.
            dc.wake_up_articulation(art)
            # Set joint position target
            dc.set_dof_position_target(dof_ptr, pos)
    except Exception as e:
        print(str(e))
    simulation_app.update()

simulation_app.close()
