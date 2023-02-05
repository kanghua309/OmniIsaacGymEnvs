import omni.usd
import omni.kit.commands
from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools, PhysxSchema
import numpy as np

stage = omni.usd.get_context().get_stage()

joint_paths = []
for quadrant in ["left_front", "left_back", "right_front", "right_back"]:
    for component, sub in [("shoulder_link", "knee")]:
        joint_paths.append(f"{quadrant}_{component}/{quadrant}_{sub}_joint")
    joint_paths.append(f"base_frame_link/{quadrant}_shoulder_joint")
for joint_path in joint_paths:
    print("path:", joint_path)
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"/bittle/{joint_path}"), "angular")
    # print(drive)
    # drive.GetTargetVelocityAttr().Set(30)
    # drive.GetTargetPositionAttr().Set(1)
    drive.GetTargetPositionAttr().Set(0)
    drive.GetDampingAttr().Set(40)
    drive.GetStiffnessAttr().Set(400)
    drive.GetMaxForceAttr().Set(1000)


curr_prim = stage.GetPrimAtPath("/bittle")
for link_prim in curr_prim.GetChildren():
    print("link_prim:", link_prim)
    if link_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        print("link_prim 2:", link_prim)
        rb = PhysxSchema.PhysxRigidBodyAPI.Get(stage, link_prim.GetPrimPath())
        rb.GetDisableGravityAttr().Set(False)
        rb.GetRetainAccelerationsAttr().Set(False)
        rb.GetLinearDampingAttr().Set(0.0)
        rb.GetMaxLinearVelocityAttr().Set(1000.0)
        rb.GetAngularDampingAttr().Set(0.0)
        rb.GetMaxAngularVelocityAttr().Set(64 / np.pi * 180)

    if link_prim.HasAPI(UsdPhysics.MassAPI):
        print("link_prim 3:", link_prim)
        mass_api = UsdPhysics.MassAPI.Apply(link_prim)
        #mass_api.CreateMassAttr(10)
        #mass_api.GetMassAttr().Set(0.0)
        ### Alternatively set the density
        #mass_api.CreateDensityAttr(1000)
        #mass_api.GetDensityAttr().Set(0.0)
