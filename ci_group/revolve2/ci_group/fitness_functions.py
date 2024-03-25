"""Standard fitness functions for modular robots."""

import math
import sys
sys.path.append('../../ci_group')
from revolve2.modular_robot_simulation import ModularRobotSimulationState


def xy_displacement(
    begin_state: ModularRobotSimulationState, end_state: ModularRobotSimulationState
) -> float:
    """
    Calculate the distance traveled on the xy-plane by a single modular robot.

    :param begin_state: Begin state of the robot.
    :param end_state: End state of the robot.
    :returns: The calculated fitness.
    """
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position
    return math.sqrt(
        (begin_position.x - end_position.x) ** 2
        + (begin_position.y - end_position.y) ** 2
    )


def z_cumulative_displacement(states) -> float:
    negative_threshold = -0.18
    positive_threshold = 0.22

    positive_weight = 1.5
    negative_weight = 1.5
    y_weight = 4

    # print('states',len(states))
    z_positions = [state.get_pose().position.z for state in states]
    # x_positions=[state.core_position[2] for state in states]
    # print(states[-1].core_position)

    last_y_positions = states[-1].get_pose().position.y
    differences = [z_positions[i] - z_positions[i - 1] for i in range(1, len(z_positions))]

    negative_values = [diff for diff in differences if diff < negative_threshold]
    #positive_values = [diff for diff in differences if diff > positive_threshold]
    #sum(positive_values) * negative_weight

    return sum(negative_values) * positive_weight + last_y_positions * y_weight
def xyz_displacement(
    begin_state: ModularRobotSimulationState, end_state: ModularRobotSimulationState
) -> float:
    """
    Calculate the distance traveled on the xy-plane by a single modular robot.

    :param begin_state: Begin state of the robot.
    :param end_state: End state of the robot.
    :returns: The calculated fitness.
    """
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position
    return -end_position.y