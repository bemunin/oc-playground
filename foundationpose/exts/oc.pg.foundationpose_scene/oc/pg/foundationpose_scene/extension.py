import gc

import carb
import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import prims as prim_utils
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._world = World.instance()
        self._scene = self._world.scene

        self._setup_scene()

    def on_shutdown(self):
        self._world.scene.clear(registry_only=True)
        if prim_utils.get_prim_at_path("/World/RectLight"):
            prim_utils.delete_prim("/World/RectLight")

        if prim_utils.get_prim_at_path("/World/DistantLight"):
            prim_utils.delete_prim("/World/DistantLight")

        gc.collect()

    def _setup_scene(self):
        asset_root = self._get_assets_root_path()
        table_usd_path = (
            asset_root
            + "/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/common_assets/props_general/table01/table01.usd"
        )

        mustard_bottle_usd_path = (
            asset_root + "/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd"
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
            self._scene.add(XFormPrim("/World/Table01", name="Table01"))

        if not self._scene.object_exists("MustardBottle"):
            add_reference_to_stage(
                usd_path=mustard_bottle_usd_path,
                prim_path="/World/MustardBottle",
            )
            self._scene.add(
                XFormPrim(
                    "/World/MustardBottle",
                    name="MustardBottle",
                    position=np.array([0, 0, 1]),
                    scale=np.array([1, 1, 1]),
                )
            )

        return

    def _get_assets_root_path(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        return assets_root_path
