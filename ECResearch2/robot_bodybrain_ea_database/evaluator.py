"""Evaluator class."""
import sys
sys.path.append('../../ci_group')
sys.path.append('../../modular_robot')
sys.path.append('../../modular_robot_simulation')
sys.path.append('../../simulation')
sys.path.append('../../experimentation')
sys.path.append('../../simulators/mujoco_simulator')
from revolve2.ci_group import fitness_functions, terrains
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
import custom_terrain
from revolve2.simulators.mujoco_simulator import LocalSimulator


class Evaluator:
    """Provides evaluation of robots."""

    _simulator: LocalSimulator
    _terrain: Terrain

    def __init__(
        self,
        headless: bool,
        num_simulators: int,
    ) -> None:
        """
        Initialize this object.

        :param headless: `headless` parameter for the physics simulator.
        :param num_simulators: `num_simulators` parameter for the physics simulator.
        """
        self._simulator = LocalSimulator(
            headless=headless, num_simulators=num_simulators
        )

        self._terrain = custom_terrain.staircase()

    def evaluate(
        self,
        robots: list[ModularRobot],
    ) -> list[float]:
        """
        Evaluate multiple robots.

        Fitness is the distance traveled on the xy plane.

        :param robots: The robots to simulate.
        :returns: Fitnesses of the robots.
        """
        # Create the scenes.
        scenes = []
        for robot in robots:
            scene = ModularRobotScene(terrain=self._terrain)
            scene.add_robot(robot)
            scenes.append(scene)

        # Simulate all scenes.
        scene_states = simulate_scenes(
            simulator=self._simulator,
            batch_parameters=make_standard_batch_parameters(),
            scenes=scenes,
        )
        """
        # Calculate the xy displacements.
        xy_displacements = [
            fitness_functions.xyz_displacement(
                states[0].get_modular_robot_simulation_state(robot),
                states[-1].get_modular_robot_simulation_state(robot))
            for robot, states in zip(robots, scene_states)
        ]
        #print(scene_states[0][0].get_modular_robot_simulation_state(robots[0]).get_pose().position.y,scene_states[0][-1].get_modular_robot_simulation_state(robots[0]).get_pose().position.y)
        """
        all_states_robots=[]
        for robot, states in zip(robots, scene_states):
            all_states = []
            for state in states:
                all_states.append(state.get_modular_robot_simulation_state(robot))
            all_states_robots.append(all_states)

        xy_displacements = [
             fitness_functions.z_cumulative_displacement(states_)
         for states_ in all_states_robots
         ]




        return xy_displacements
