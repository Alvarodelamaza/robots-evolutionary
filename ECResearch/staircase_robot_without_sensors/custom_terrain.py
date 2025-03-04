"""Main script for the example."""

import math

from pyrr import Quaternion, Vector3
import sys
sys.path.append('../../ci_group')
sys.path.append('../../modular_robot')
sys.path.append('../../modular_robot_simulation')
sys.path.append('../../simulation')
sys.path.append('../../experimentation')
sys.path.append('../../simulators/mujoco_simulator')
from revolve2.ci_group.modular_robots_v1 import gecko_v1
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
from revolve2.simulation.scene import AABB, Color, Pose
from revolve2.simulation.scene.geometry import GeometryBox, GeometryPlane
from revolve2.simulation.scene.geometry.textures import Texture
from revolve2.simulators.mujoco_simulator import LocalSimulator


def staircase() -> Terrain:
    """
    Create a custom terrain.

    :returns: The created terrain.
    """
    # A terrain is a collection of static geometries.
    # Here we create a simple terrain uses some boxes.

    return Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(position=Vector3(), orientation=Quaternion()),
                mass=0.0,
                size=Vector3([20.0, 20.0, 0.0]),
                #color=Color(170, 170, 180, 255),
                texture=Texture(),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -0.5, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([3, 0.4, 0.04])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -0.9, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([3, 0.4, 0.08])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -1.3, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([2, 0.4, 0.12])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -1.7, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([2, 0.4, 0.16])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -2.1, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([1, 0.4, 0.2])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -2.5, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([1, 0.5, 0.25])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -3, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([1, 0.5, 0.3])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -3.5, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([1, 0.5, 0.36])),
            ),
            GeometryBox(
                pose=Pose(position=Vector3([0, -4, 0.0]), orientation=Quaternion()),
                mass=0.0,
                #color=Color(0, 255, 0, 255),
                texture=Texture(),
                aabb=AABB(size=Vector3([1, 0.5, 0.42])),
            ),
        ]
    )


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up the random number generator.
    rng = make_rng_time_seed()

    # Create a robot
    body = gecko_v1()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=staircase())
    scene.add_robot(robot)

    # Simulate the scene.
    simulator = LocalSimulator()
    simulate_scenes(
        simulator=simulator,
        batch_parameters=make_standard_batch_parameters(),
        scenes=scene,
    )


if __name__ == "__main__":
    main()