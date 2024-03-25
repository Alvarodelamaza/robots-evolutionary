import logging
import sys
sys.path.append('../../ci_group')
sys.path.append('../../modular_robot')
sys.path.append('../../modular_robot_simulation')
sys.path.append('../../simulation')
sys.path.append('../../experimentation')
sys.path.append('../../simulators/mujoco_simulator')
import config
import multineat
import numpy as np
import numpy.typing as npt
from base import Base
from evaluator import Evaluator
from experiment import Experiment
from generation import Generation
from genotype import Genotype
from individual import Individual
from population import Population
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng, seed_from_time, seed_from_string
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.ci_group import fitness_functions, terrains

from revolve2.modular_robot.body.base import ActiveHinge, ActiveHingeSensor
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot_simulation import (
    ModularRobotScene,
    Terrain,
    simulate_scenes,
)
from revolve2.simulators.mujoco_simulator import LocalSimulator

def main() -> None:
    """Run the simulation."""
    # Set up logging.
    # logging.info("----------------")
    # logging.info("Start experiment")

    rng_seed = seed_from_string("101")
    rng = make_rng(rng_seed)

    # evaluator = Evaluator(headless=True, num_simulators=config.NUM_SIMULATORS)

    # CPPN innovation databases.
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    # logging.info("Generating initial genotype.")
    initial_genotype = Genotype.random(
        innov_db_body=innov_db_body,
        innov_db_brain=innov_db_brain,
        rng=rng,
    )

    # Combine the body and brain into a modular robot.
    robot = initial_genotype.develop()
    # body = robot.body
    # brain = robot.brain
    # active_hinges = body.find_modules_of_type(ActiveHinge)
    # for active_hinge in active_hinges:
    #     active_hinge.sensor = ActiveHingeSensor()
    #
    # robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Simulate the scene.
    simulator = LocalSimulator()
    simulate_scenes(
        simulator=simulator,
        batch_parameters=make_standard_batch_parameters(simulation_time=100, simulation_timestep=0.05),
        scenes=scene,
    )


if __name__ == "__main__":
    main()