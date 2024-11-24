import gc
from typing import List, Optional

import carb
import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import prims as prim_utils
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import PhysxSchema, UsdPhysics


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._world = World.instance()
        self._scene = self._world.scene
        self._asset_root = self._get_assets_root_path()

        self._setup_scene()

    def on_shutdown(self):
        self._world.scene.clear(registry_only=True)
        if prim_utils.get_prim_at_path("/World/RectLight"):
            prim_utils.delete_prim("/World/RectLight")

        if prim_utils.get_prim_at_path("/World/DistantLight"):
            prim_utils.delete_prim("/World/DistantLight")

        gc.collect()

    def _setup_scene(self):
        table_usd_path = (
            self._asset_root
            + "/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/common_assets/props_general/table01/table01.usd"
        )

        self._scene.add_default_ground_plane()

        if not prim_utils.get_prim_at_path("/World/RectLight"):
            prim_utils.create_prim(
                "/World/RectLight",
                "RectLight",
                position=np.array([0, 0, 0.5]),
                attributes={
                    "inputs:intensity": 1100,
                    "inputs:color": (1.0, 1.0, 1.0),
                },
            )

        if not prim_utils.get_prim_at_path("/World/DistantLight"):
            prim_utils.create_prim(
                "/World/DistantLight",
                "DistantLight",
                position=np.array([1.0, 1.0, 1.0]),
                attributes={
                    "inputs:intensity": 300,
                    "inputs:color": (1.0, 1.0, 1.0),
                },
            )

        if not self._scene.object_exists("Table01"):
            add_reference_to_stage(usd_path=table_usd_path, prim_path="/World/Table01")
            xform = XFormPrim("/World/Table01", name="Table01")
            UsdPhysics.CollisionAPI.Apply(xform.prim)
            self._scene.add(xform)

        # Add YCB object
        self._add_ycb_object(
            "MustardBottle", "MustardBottle01", [-0.1, -0.1, 1], [0, 0, -90]
        )
        self._add_ycb_object(
            "MustardBottle", "MustardBottle02", [0.2, -0.2, 1], [0, 0, -45]
        )
        self._add_ycb_object("Scissors", "Scissors01", [0.1, 0.1, 1])
        self._add_ycb_object(
            "TunaFishCan", "TunaFishCan01", [0.25, 0.1, 1], [-90, 0, 0]
        )

        self._add_ycb_object("Mug", "Mug01", [-0.2, 0.1, 1], [-90, 0, 0])

        self._add_ycb_object("LargeMarker", "LargeMarker01", [0, 0.1, 1], [0, 0, 0])
        self._world.reset()

        return

    def _get_assets_root_path(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        return assets_root_path

    def _add_ycb_object(
        self,
        object: str,
        name: str,
        position: List[float],
        rotation_deg: Optional[List[float]] = None,
    ):
        if self._scene.object_exists(name):
            return

        ycb_base_path = self._asset_root + "/Isaac/Props/YCB/Axis_Aligned/"
        object_path_map = {
            "MustardBottle": ycb_base_path + "006_mustard_bottle.usd",
            "Scissors": ycb_base_path + "037_scissors.usd",
            "TunaFishCan": ycb_base_path + "007_tuna_fish_can.usd",
            "Mug": ycb_base_path + "025_mug.usd",
            "Banana": ycb_base_path + "011_banana.usd",
            "Bowl": ycb_base_path + "024_bowl.usd",
            "SugarBox": ycb_base_path + "004_sugar_box.usd",
            "LargeMarker": ycb_base_path + "040_large_marker.usd",
        }

        usd_path = object_path_map[object]
        prim_path = f"/World/{name}"

        add_reference_to_stage(
            usd_path=usd_path,
            prim_path=prim_path,
        )

        rotation_quaternion = None
        if rotation_deg is not None:
            rotation_quaternion = euler_angles_to_quat(
                np.array(rotation_deg), degrees=True
            )

        xform = XFormPrim(
            prim_path,
            name=name,
            position=np.array(position),
            scale=np.array([1, 1, 1]),
            orientation=rotation_quaternion,
        )

        UsdPhysics.CollisionAPI.Apply(xform.prim)
        UsdPhysics.MassAPI.Apply(xform.prim)
        UsdPhysics.RigidBodyAPI.Apply(xform.prim)
        rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.prim)
        rigid_api.GetEnableCCDAttr().Set(True)

        self._scene.add(xform)
