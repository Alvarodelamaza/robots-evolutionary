from __future__ import annotations

import multineat
import math
import numpy as np
import numpy.typing as npt

from ..._modular_robot_control_interface import ModularRobotControlInterface
from ...body.base import ActiveHinge, Body
from ...sensor_state._modular_robot_sensor_state import ModularRobotSensorState
from .._brain_instance import BrainInstance
from ._make_cpg_network_structure_neighbor import (
    active_hinges_to_cpg_network_structure_neighbor,
)


class BrainCpgInstance(BrainInstance):
    """
    Cpg network brain.

    A state array that is integrated over time following the differential equation `X'=WX`.
    W is a weight matrix that is multiplied by the state array.
    The outputs of the controller are defined by the `outputs`, a list of indices for the state array.
    """

    _initial_state: npt.NDArray[np.float_]
    _weight_matrix: npt.NDArray[np.float_]  # nxn matrix matching number of neurons
    _output_mapping: list[tuple[int, ActiveHinge]]

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
        body: Body,
        genotype: multineat.Genome
    ) -> None:
        """
        Initialize this object.

        :param initial_state: The initial state of the neural network.
        :param weight_matrix: The weight matrix used during integration.
        :param output_mapping: Marks neurons as controller outputs and map them to the correct active hinge.
        """
        assert initial_state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert initial_state.shape[0] == weight_matrix.shape[0]
        assert all([i >= 0 and i < len(initial_state) for i, _ in output_mapping])

        self._state = initial_state
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping

        self._active_hinges = body.find_modules_of_type(ActiveHinge)
        self.cpg_network_structure, self._output_mapping = active_hinges_to_cpg_network_structure_neighbor(
            self._active_hinges)
        self._connections = [
            (self._active_hinges[pair.cpg_index_lowest.index], self._active_hinges[pair.cpg_index_highest.index])
            for pair in self.cpg_network_structure.connections
        ]
        self._body = body
        self._genotype = genotype

        self._update_timer = 0.0
        self._total_time = 0.0
        self._update_interval = 10.0

    def update_weights(self, sensor_data: list[float]):
        # 创建 CPPN 网络并从 genotype 构建其表现型
        brain_net = multineat.NeuralNetwork()
        self._genotype.BuildPhenotype(brain_net)

        # 计算新的内部和外部权重
        # print(sensor_data)
        internal_weights, external_weights = self._calculate_weights(sensor_data)
        # print(internal_weights)
        # print(external_weights)

        self._weight_matrix = self.cpg_network_structure.make_connection_weights_matrix(
            {cpg: weight for cpg, weight in zip(self.cpg_network_structure.cpgs, internal_weights)},
            {pair: weight for pair, weight in zip(self.cpg_network_structure.connections, external_weights)}
        )
        # self._initial_state = self.cpg_network_structure.make_uniform_state(0.5 * math.sqrt(2))

        # print("NB!")
        # print(sensor_data)

    def _calculate_weights(self, sensor_data: list[float])-> tuple[list[float], list[float]]:
        brain_net = multineat.NeuralNetwork()
        self._genotype.BuildPhenotype(brain_net)

        internal_weights = []
        for active_hinge, sensor_value in zip(self._active_hinges, sensor_data):
            pos = self._body.grid_position(active_hinge)
            weight = self._evaluate_network(
                brain_net,
                [
                    1.0,
                    float(pos.x),
                    float(pos.y),
                    float(pos.z),
                    float(pos.x),
                    float(pos.y),
                    float(pos.z),
                    sensor_value,
                    sensor_value,
                ],
            )
            internal_weights.append(weight)

        external_weights = []
        for (active_hinge1, active_hinge2) in self._connections:
            pos1 = self._body.grid_position(active_hinge1)
            pos2 = self._body.grid_position(active_hinge2)
            sensor_value1 = sensor_data[self._active_hinges.index(active_hinge1)]
            sensor_value2 = sensor_data[self._active_hinges.index(active_hinge2)]
            weight = self._evaluate_network(
                brain_net,
                [
                    1.0,
                    float(pos1.x),
                    float(pos1.y),
                    float(pos1.z),
                    float(pos2.x),
                    float(pos2.y),
                    float(pos2.z),
                    sensor_value1,  # 添加第一个铰链的传感器数据
                    sensor_value2  # 添加第二个铰链的传感器数据
                ],
            )
            external_weights.append(weight)

        return internal_weights, external_weights


    @staticmethod
    def _evaluate_network(network: multineat.NeuralNetwork, inputs: list[float]) -> float:
        network.Input(inputs)
        network.ActivateAllLayers()
        return float(network.Output()[0])

    @staticmethod
    def _rk45(
        state: npt.NDArray[np.float_], A: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        A1: npt.NDArray[np.float_] = np.matmul(A, state)
        A2: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A1))
        A3: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A2))
        A4: npt.NDArray[np.float_] = np.matmul(A, (state + dt * A3))
        return state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot.

        Sets the active hinge targets to the values in the state array as defined by the mapping provided in the constructor.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """

        # Integrate ODE to obtain new state.
        self._state = self._rk45(self._state, self._weight_matrix, dt)

        # Set active hinge targets to match newly calculated state.
        for state_index, active_hinge in self._output_mapping:
            control_interface.set_active_hinge_target(
                active_hinge, float(self._state[state_index]) * active_hinge.range
            )

        sensors = [active_hinge.sensor for active_hinge in self._active_hinges if active_hinge.sensor is not None]
        assert len(sensors) == len(self._active_hinges), "One of the active hinges does not have a sensor set."
        current_positions = [sensor_state.get_active_hinge_sensor_state(sensor).position for sensor in sensors]
        # print("Current active hinge positions:", current_positions)

        self._total_time += dt
        self._update_timer +=dt
        if self._update_timer >= self._update_interval:
            # print(f"Time to print. Current total time: {self._total_time} 秒")
            self.update_weights(current_positions)
            self._update_timer = 0