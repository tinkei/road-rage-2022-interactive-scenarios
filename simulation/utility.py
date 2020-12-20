from typing import List

import matplotlib.pyplot as plt
from IPython import display
from commonroad.common.solution import PlanningProblemSolution, Solution, CommonRoadSolutionWriter, VehicleType, \
    VehicleModel, CostFunction
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.visualization.draw_dispatch_cr import draw_object


def create_trajectory_from_list_states(list_states: List[State]) -> Trajectory:
    """Returns a CR Trajectory for the given list of CR States"""

    list_states_new = list()
    time_step = 0
    for state in list_states:
        kwarg = {'position': state.position,
                 'velocity': state.velocity,
                 'orientation': state.orientation,
                 # todo: this should be changed with performing a more serious motion planning
                 'steering_angle': 0.0,
                 'time_step': time_step}
        list_states_new.append(State(**kwarg))
        time_step += 1

    return Trajectory(initial_time_step=list_states_new[0].time_step, state_list=list_states_new)


def create_obstacle_from_trajectory(trajectory: Trajectory, id_obstacle: int) -> DynamicObstacle:
    """Returns a dynamic obstacle for the given trajectory"""

    dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)

    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_initial_state = trajectory.state_list[0]
    dynamic_obstacle_prediction = TrajectoryPrediction(trajectory, dynamic_obstacle_shape)

    # create the dynamic obstacle
    dynamic_obstacle_id = id_obstacle
    dynamic_obstacle_type = ObstacleType.CAR
    obstacle_ego = DynamicObstacle(dynamic_obstacle_id,
                                   dynamic_obstacle_type,
                                   dynamic_obstacle_shape,
                                   dynamic_obstacle_initial_state,
                                   dynamic_obstacle_prediction)
    return obstacle_ego


def visualize_scenario_with_trajectory(scenario: Scenario,
                                       planning_problem_set: PlanningProblemSet,
                                       trajectory: Trajectory = None,
                                       discrete_time_step: bool = False,
                                       num_time_steps: int = None) -> None:
    if not num_time_steps:
        if trajectory:
            num_time_steps = len(trajectory.state_list)
        else:
            num_time_steps = 50

    flag_obstacle = False
    dynamic_obstacle = None
    if trajectory:
        flag_obstacle = True
        # create the ego vehicle prediction using the trajectory and the shape of the obstacle
        dynamic_obstacle_initial_state = trajectory.state_list[0]
        dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
        dynamic_obstacle_prediction = TrajectoryPrediction(trajectory, dynamic_obstacle_shape)

        # generate the dynamic obstacle according to the specification
        dynamic_obstacle_id = scenario.generate_object_id()
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                           dynamic_obstacle_type,
                                           dynamic_obstacle_shape,
                                           dynamic_obstacle_initial_state,
                                           dynamic_obstacle_prediction)

    # visualize scenario
    for i in range(0, num_time_steps):
        if not discrete_time_step:
            display.clear_output(wait=True)
        plt.figure(figsize=(10, 10))
        draw_object(scenario, draw_params={'time_begin': i})
        draw_object(planning_problem_set)
        if flag_obstacle:
            draw_object(dynamic_obstacle,
                        draw_params={'time_begin': i,
                                     'dynamic_obstacle': {'shape': {'facecolor': 'green'},
                                                          'trajectory': {'draw_trajectory': True,
                                                                         'facecolor': '#ff00ff',
                                                                         'draw_continuous': True,
                                                                         'z_order': 60,
                                                                         'line_width': 5}
                                                          }
                                     })

        plt.gca().set_aspect('equal')
        plt.show()


def save_solution(scenario: Scenario, planning_problem_id: int, trajectory: Trajectory,
                  output_path: str = './', overwrite: bool = False):
    """Saves the given trajectory as a solution to the planning problem

    :param scenario:
    :param planning_problem_id:
    :param trajectory:
    :param output_path:
    :param overwrite:
    """

    type_vehicle = VehicleType.BMW_320i
    model_vehicle = VehicleModel.KS
    cost_function = CostFunction.SM1

    # create solution object for benchmark
    pps = PlanningProblemSolution(planning_problem_id=planning_problem_id,
                                  vehicle_type=type_vehicle,
                                  vehicle_model=model_vehicle,
                                  cost_function=cost_function,
                                  trajectory=trajectory)

    solution = Solution(scenario.scenario_id, [pps])

    # write solution
    csw = CommonRoadSolutionWriter(solution)
    csw.write_to_file(output_path=output_path, overwrite=overwrite)
    print("Trajectory saved to solution file.")
