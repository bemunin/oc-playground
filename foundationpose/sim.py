# ruff:noqa: E402
import cv2
import isaacsim  # noqa: F401
from omni.isaac.kit import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": False}
sim_app = SimulationApp(CONFIG)

from omni.isaac.core import World


def init_world():
    world_settings = {
        "physics_dt": 1.0 / 60.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
    }
    World(**world_settings)
    world = World.instance()
    world.initialize_physics()


def running_sim(sim_app: SimulationApp):
    world = World.instance()
    world.play()
    while sim_app.is_running():
        world.step()


def clear_world():
    world = World.instance()
    world.stop()
    world.clear_all_callbacks()
    world.clear_instance()


if __name__ == "__main__":
    print(cv2.__version__)
    init_world()
    running_sim(sim_app)
    clear_world()
    sim_app.close()
