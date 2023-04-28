# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.quadruped.robots import Anymal
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path, get_prim_children
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom

import omni.appwindow  # Contains handle to keyboard
import numpy as np
import carb

import carb
import asyncio
#####

import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import SimulationContext
from pxr import Sdf

from omni.isaac.core.utils.extensions import enable_extension
import omni.graph.core as og

enable_extension("omni.isaac.ros2_bridge")

# need lock ?
base_command = np.zeros(3)


class Anymal_runner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        """
        Summary

        creates the simulation world with preset physics_dt and render_dt and creates an anymal robot inside the warehouse

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
        ##physics_prim_path= "/World/physicsScene")
        # assets_root_path = get_assets_root_path()
        # if assets_root_path is None:
        #     carb.log_error("Could not find Isaac Sim assets folder")
        # print("assets_root_path:",assets_root_path)

        stage_path = "omniverse://localhost/Users/king/my_warehouse_navigation.usd"
        scenario_path = stage_path
        print("scenario_path:", scenario_path)
        # async def load_stage(path):
        #     await omni.usd.get_context().open_stage_async(path)
        # asyncio.ensure_future(load_stage(scenario_path))
        # omni.usd.get_context().open_stage(scenario_path)

        # spawn warehouse scene
        prim = get_prim_at_path("/World")
        print("prim ------  ------  ------- :", prim)
        # print("all:",get_prim_children(prim))
        if not prim.IsValid():
            prim = define_prim("/World", "Xform")
            #     asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
            prim.GetReferences().AddReference(scenario_path)
        # print("carter_navigation_params.yaml:",carter_navigation_params.yaml)
        # prim = define_prim("/", "Xform")
        # self._world.scene.add()
        # spawn warehouse scene
        # prim = get_prim_at_path("/World/GroundPlane")
        # if not prim.IsValid():
        #     prim = define_prim("/World/GroundPlane", "Xform")
        #     asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        #     prim.GetReferences().AddReference(asset_path)

        self._anymal = self._world.scene.add(
            Anymal(
                prim_path="/World/Anymal_ROS",
                name="Anymal",
                usd_path="omniverse://localhost/Users/king/Anymal_ROS.usd",
                position=np.array([-6.0, -1.0, 0.70]),  # pos as cater demo
            )
        )
        print("all2:", get_prim_children(prim))
        # self._anymal = self._world.scene.get_object(name="/World/Anymal_ROS")
        print("--------------------1:", self._anymal)
        self._world.reset()
        self._enter_toggled = 0
        # self._base_command = np.zeros(3)

        # import time
        # time.sleep(1000)
        # # bindings for keyboard to command
        # self._input_keyboard_mapping = {
        #     # forward command
        #     "NUMPAD_8": [1.0, 0.0, 0.0],
        #     "UP": [1.0, 0.0, 0.0],
        #     # back command
        #     "NUMPAD_2": [-1.0, 0.0, 0.0],
        #     "DOWN": [-1.0, 0.0, 0.0],
        #     # left command
        #     "NUMPAD_6": [0.0, -1.0, 0.0],
        #     "RIGHT": [0.0, -1.0, 0.0],
        #     # right command
        #     "NUMPAD_4": [0.0, 1.0, 0.0],
        #     "LEFT": [0.0, 1.0, 0.0],
        #     # yaw command (positive)
        #     "NUMPAD_7": [0.0, 0.0, 1.0],
        #     "N": [0.0, 0.0, 1.0],
        #     # yaw command (negative)
        #     "NUMPAD_9": [0.0, 0.0, -1.0],
        #     "M": [0.0, 0.0, -1.0],
        # }
        self.needs_reset = False

    def setup(self) -> None:
        """
        [Summary]

        Set up keyboard listener and add physics callback

        """
        print("setup")
        self._appwindow = omni.appwindow.get_default_app_window()
        # self._input = carb.input.acquire_input_interface()
        # self._keyboard = self._appwindow.get_keyboard()
        # self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._world.add_physics_callback("anymal_advance", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        print(
            "on_physics_step-----------------------------------------------------------------------------------------")
        """
        [Summary]

        Physics call back, switch robot mode and call robot advance function to compute and apply joint torque

        """
        if self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
        # self._anymal.advance(step_size, self._base_command)
        print("cmd:", base_command)
        self._anymal.advance(step_size, base_command)

    def run(self) -> None:
        print("Run---------------------------------------------------------------------------------------------------:")
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
            if not self._world.is_simulating():
                self.needs_reset = True
        return

    # def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
    #     """
    #     [Summary]

    #     Keyboard subscriber callback to when kit is updated.

    #     """
    #     # reset event
    #     self._event_flag = False
    #     # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
    #     if event.type == carb.input.KeyboardEventType.KEY_PRESS:
    #         # on pressing, the command is incremented
    #         if event.input.name in self._input_keyboard_mapping:
    #             self._base_command[0:3] += np.array(self._input_keyboard_mapping[event.input.name])

    #     elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
    #         # on release, the command is decremented
    #         if event.input.name in self._input_keyboard_mapping:
    #             self._base_command[0:3] -= np.array(self._input_keyboard_mapping[event.input.name])
    #     return True


def callback(msg):
    print("callback:----------------------------------------------------------------")
    # 处理Twist消息的回调函数
    # linear_velocity = msg.linear.carter_navigation_params.yaml
    # angular_velocity = msg.angular.z
    # 进行一些处理，例如控制机器人
    print("Twist msg:", msg)
    base_command[0:3] = np.array([msg.linear.x, msg.linear.y, msg.angular.z])


def main():
    """
    [Summary]

    Parse arguments and instantiate the ANYmal runner

    """
    print("xfddsafdsafdsasfdasfdsafadsfdsaf.......-----------------------.")
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = Anymal_runner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner.setup()
    print("runner setup over!---------------------------------------------")
    ##############
    import rclpy
    rclpy.init()
    from geometry_msgs.msg import Twist
    import threading

    node = rclpy.create_node("anymal_twist")
    subscription = node.create_subscription(Twist, '/cmd_vel', callback, qos_profile=rclpy.qos.qos_profile_sensor_data)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    # publisher = node.create_publisher(Twist, "cmd_vel", 10)
    ##############

    print("runner to start")
    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()
    runner.run()
    simulation_app.close()
    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
