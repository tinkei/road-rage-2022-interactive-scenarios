"""
SUMO simulation specific helper methods
"""

__author__ = "Peter Kocsis, Edmond Irani Liu"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = []
__version__ = "0.1"
__maintainer__ = "Edmond Irani Liu"
__email__ = "edmond.irani@tum.de"
__status__ = "Integration"

import copy
import os
import sys
import pickle
from enum import unique, Enum
from math import sin, cos
from pprint import pp
from typing import Tuple, Dict, Optional

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import Solution
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from sumocr.interface.ego_vehicle import EgoVehicle
from sumocr.interface.sumo_simulation import SumoSimulation
from sumocr.maps.sumo_scenario import ScenarioWrapper
from sumocr.sumo_config.default import DefaultConfig
from sumocr.sumo_docker.interface.docker_interface import SumoInterface
from sumocr.visualization.video import create_video


path_notebook = os.getcwd()
sys.path.append(os.path.join(path_notebook, 'commonroad-search/'))
sys.path.append(os.path.join(path_notebook, 'commonroad-search/SMP/'))

from maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.utility import plot_primitives
from MotionPrimitiveGenerator import MotionPrimitiveGenerator as MPG
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.common.solution import CostFunction
from SMP.motion_planner.motion_planner import MotionPlanner, MotionPlannerType
from SMP.motion_planner.utility import create_trajectory_from_list_states, visualize_solution

@unique
class Algo(Enum):
    """Enum for search algo used."""
    ASTAR = 1
    GBFS = 2
algo = Algo.ASTAR
# algo = Algo.GBFS

# Primitives 389 (w/ mirror), or 270. Branching factor 5.565.
automaton = ManeuverAutomaton.generate_automaton('my_motion_primitives/V_0.0_42.0_Vstep_3.0_SA_-0.91_0.91_SAstep_0.15_T_0.5_Model_FORD_ESCORT.xml')

# Primitives 53 (w/ mirror), or 53. Branching factor 1.0.
# automaton = ManeuverAutomaton.generate_automaton('my_motion_primitives/V_0.0_42.0_Vstep_3.0_SA_-0.91_0.91_SAstep_0.15_T_0.1_Model_FORD_ESCORT.xml')

@unique
class SimulationOption(Enum):
    WITHOUT_EGO = "_without_ego"
    MOTION_PLANNER = "_planner"
    SOLUTION = "_solution"


def simulate_scenario(mode: SimulationOption,
                      conf: DefaultConfig,
                      scenario_wrapper: ScenarioWrapper,
                      scenario_path: str,
                      num_of_steps: int = None,
                      planning_problem_set: PlanningProblemSet = None,
                      solution: Solution = None,
                      use_sumo_manager: bool = False) -> Tuple[Scenario, Dict[int, EgoVehicle]]:
    """
    Simulates an interactive scenario with specified mode

    :param mode: 0 = without ego, 1 = with plugged in planner, 2 = with solution trajectory
    :param conf: config of the simulation
    :param scenario_wrapper: scenario wrapper used by the Simulator
    :param scenario_path: path to the interactive scenario folder
    :param num_of_steps: number of steps to simulate
    :param planning_problem_set: planning problem set of the scenario
    :param solution: solution to the planning problem
    :param use_sumo_manager: indicates whether to use the SUMO Manager
    :return: simulated scenario and dictionary with items {planning_problem_id: EgoVehicle}
    """

    if num_of_steps is None:
        num_of_steps = conf.simulation_steps

    sumo_interface = None
    if use_sumo_manager:
        sumo_interface = SumoInterface(use_docker=True)
        sumo_sim = sumo_interface.start_simulator()

        sumo_sim.send_sumo_scenario(conf.scenario_name,
                                    scenario_path)
    else:
        sumo_sim = SumoSimulation()

    # initialize simulation
    sumo_sim.initialize(conf, scenario_wrapper, None)

    if mode is SimulationOption.WITHOUT_EGO:
        # simulation without ego vehicle
        for step in range(num_of_steps):
            # set to dummy simulation
            sumo_sim.dummy_ego_simulation = True
            sumo_sim.simulate_step()

    elif mode is SimulationOption.MOTION_PLANNER:
        # simulation with plugged in planner

        trajectory_buffer = []
        pp_orig_init_states = {}
        pp_orig_time_starts = {}
        pp_orig_time_ends = {}

        def get_next_state(current_scenario, trajectory_buffer, pp_id, step, skip_step=33):
            """Run motion planner only ever n steps. Otherwise return previous trajectory."""

            if step % skip_step == 0:

                trajectory_buffer.clear()

                # Backup initial state before modification.
                if pp_id not in pp_orig_init_states:
                    pp_orig_init_states[pp_id] = planning_problem_set.planning_problem_dict[pp_id].initial_state
                    pp_orig_time_starts[pp_id] = planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.start
                    pp_orig_time_ends[pp_id]   = planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.end
                    planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.start += 1
                    planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.end   += 1

                # Construct motion planner.
                if algo == Algo.ASTAR:
                    type_motion_planner = MotionPlannerType.STUDENT
                elif algo == Algo.GBFS:
                    type_motion_planner = MotionPlannerType.STUDENT_GBFS

                motion_planner = MotionPlanner.create(scenario=current_scenario, 
                                                    # Each ego vehicle corresponds to a planning problem! See `init_ego_vehicles_from_planning_problem()`.
                                                    planning_problem=planning_problem_set.planning_problem_dict[pp_id],
                                                    automaton=automaton, 
                                                    motion_planner_type=type_motion_planner)

                list_paths_primitives, _, _ = motion_planner.execute_search()
                trajectory_ego = create_trajectory_from_list_states(list_paths_primitives)
                print(f'Length of trajectory states: {len(trajectory_ego.state_list)}')
                print(f'Solution trajectory time step: {trajectory_ego.state_list[-1].time_step}')

                if len(trajectory_ego.state_list) > 1:

                    trajectory_buffer.extend(trajectory_ego.state_list[1:])
                    # next_state = trajectory_ego.state_list[1]
                    next_state = trajectory_buffer[0]

                    next_initial_state = copy.deepcopy(trajectory_buffer[:skip_step][-1])
                    next_initial_state.time_step = 0
                    # Add back deleted unnecessary attributes of the initial state
                    if not hasattr(next_initial_state, 'yaw_rate'):
                        next_initial_state.yaw_rate = 0.0
                    if not hasattr(next_initial_state, 'slip_angle'):
                        next_initial_state.slip_angle = 0.0
                    if not hasattr(next_initial_state, 'acceleration'):
                        next_initial_state.acceleration = 0.0
                    planning_problem_set.planning_problem_dict[pp_id].initial_state = next_initial_state
                    planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.start -= skip_step
                    planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.end   -= skip_step

                else:
                    raise Exception('Length of trajectory is too short!')
                # print(f'State time step {next_state.time_step}')

            return trajectory_buffer[step % skip_step]



        def run_simulation():

            ego_vehicles = sumo_sim.ego_vehicles
            pps_ids = list(planning_problem_set.planning_problem_dict.keys())
            print('Planning problem IDs:', list(planning_problem_set.planning_problem_dict.keys()))
            print('Ego vehicles\' SUMO IDs:', list(ego_vehicles.keys()))
            # x = {pps_ids[0]: ego_v for _, ego_v in sumo_sim.ego_vehicles.items()}
            # print('Reassigned `ego_vehicles` dict:', x)
            # print(planning_problem_set.planning_problem_dict[pps_ids[0]].goal.state_list[0].time_step)
            for k, pp in planning_problem_set.planning_problem_dict.items():
                # pp.goal.state_list[0].time_step.end += 4
                # pp.goal.state_list[0].time_step.start -= 1
                pass
            # print(planning_problem_set.planning_problem_dict[pps_ids[0]].goal.state_list[0].time_step)

            for step in range(num_of_steps):
                # print()
                # print(f'Processing step {step+1:03d} in {num_of_steps} steps.')
                # print(f'Goal time step interval: {planning_problem_set.planning_problem_dict[pps_ids[0]].goal.state_list[0].time_step.start} {planning_problem_set.planning_problem_dict[pps_ids[0]].goal.state_list[0].time_step.end}')

                if use_sumo_manager:
                    ego_vehicles = sumo_sim.ego_vehicles

                # retrieve the CommonRoad scenario at the current time step, e.g. as an input for a prediction module
                current_scenario = sumo_sim.commonroad_scenario_at_time_step(sumo_sim.current_time_step, start_0=True)
                # print(type(current_scenario))
                # print(current_scenario)
                for idx, ego_vehicle in enumerate(ego_vehicles.values()): # Same as looping over planning problem set.
                    # retrieve the current state of the ego vehicle
                    state_current_ego = ego_vehicle.current_state
                    # print(idx, type(state_current_ego), state_current_ego)

                    # ====== plug in your motion planner here
                    if step == 0:
                        mod_init_state = copy.deepcopy(planning_problem_set.planning_problem_dict[pps_ids[idx]].initial_state)
                        mod_init_state.time_step = 1
                        trajectory_ego = [mod_init_state]
                        ego_vehicle.set_planned_trajectory(trajectory_ego)
                    # example motion planner which decelerates to full stop
                    # next_state = copy.deepcopy(state_current_ego)
                    # next_state.steering_angle = 0.0
                    # a = -4.0
                    # dt = 0.1
                    # if next_state.velocity > 0:
                    #     v = next_state.velocity
                    #     x, y = next_state.position
                    #     o = next_state.orientation

                    #     next_state.position = np.array([x + v * cos(o) * dt, y + v * sin(o) * dt])
                    #     next_state.velocity += a * dt
                    next_state = get_next_state(current_scenario, trajectory_buffer, pps_ids[idx], step)
                    # ====== end of motion planner

                    # update the ego vehicle with new trajectory with only 1 state for the current step
                    next_state.time_step = 1
                    trajectory_ego = [next_state]
                    ego_vehicle.set_planned_trajectory(trajectory_ego)

                if use_sumo_manager:
                    # set the modified ego vehicles to synchronize in case of using sumo_docker
                    sumo_sim.ego_vehicles = ego_vehicles

                sumo_sim.simulate_step()

            # Restore initial state after simulation.
            if use_sumo_manager:
                ego_vehicles = sumo_sim.ego_vehicles
            for idx, ego_vehicle in enumerate(ego_vehicles.values()):
                pp_id = pps_ids[idx]
                if pp_id in pp_orig_init_states:
                    next_initial_state = pp_orig_init_states.pop(pp_id)
                    if not hasattr(next_initial_state, 'yaw_rate'):
                        next_initial_state.yaw_rate = 0.0
                    if not hasattr(next_initial_state, 'slip_angle'):
                        next_initial_state.slip_angle = 0.0
                    if not hasattr(next_initial_state, 'acceleration'):
                        next_initial_state.acceleration = 0.0
                    planning_problem_set.planning_problem_dict[pp_id].initial_state = next_initial_state
                    planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.end   = pp_orig_time_ends.pop(pp_id)
                    planning_problem_set.planning_problem_dict[pp_id].goal.state_list[0].time_step.start = pp_orig_time_starts.pop(pp_id)

        run_simulation()

    elif mode is SimulationOption.SOLUTION:
        # simulation with given solution trajectory

        def run_simulation():
            ego_vehicles = sumo_sim.ego_vehicles

            for time_step in range(num_of_steps):
                if use_sumo_manager:
                    ego_vehicles = sumo_sim.ego_vehicles
                for idx_ego, ego_vehicle in enumerate(ego_vehicles.values()):
                    # update the ego vehicles with solution trajectories
                    trajectory_solution = solution.planning_problem_solutions[idx_ego].trajectory
                    next_state = copy.deepcopy(trajectory_solution.state_list[time_step])

                    if time_step == 0:
                        pps_ids = list(planning_problem_set.planning_problem_dict.keys())
                        mod_init_state = copy.deepcopy(planning_problem_set.planning_problem_dict[pps_ids[idx_ego]].initial_state)
                        mod_init_state.time_step = 1
                        trajectory_ego = [mod_init_state]
                        ego_vehicle.set_planned_trajectory(trajectory_ego)

                    next_state.time_step = 1
                    trajectory_ego = [next_state]
                    ego_vehicle.set_planned_trajectory(trajectory_ego)

                if use_sumo_manager:
                    # set the modified ego vehicles to synchronize in case of using SUMO Manager
                    sumo_sim.ego_vehicles = ego_vehicles

                sumo_sim.simulate_step()

        check_trajectories(solution, planning_problem_set, conf)
        run_simulation()

    # retrieve the simulated scenario in CR format
    simulated_scenario = sumo_sim.commonroad_scenarios_all_time_steps()

    # stop the simulation
    sumo_sim.stop()
    ego_vehicles = {list(planning_problem_set.planning_problem_dict.keys())[0]:
                         ego_v for _, ego_v in sumo_sim.ego_vehicles.items()}
    reindexed_ego_vehicles = {}
    for idx, (pp_id, _) in enumerate(planning_problem_set.planning_problem_dict.items()):
        reindexed_ego_vehicles[pp_id] = list(sumo_sim.ego_vehicles.values())[idx]

    if use_sumo_manager:
        sumo_interface.stop_simulator()

    print()

    return simulated_scenario, reindexed_ego_vehicles#ego_vehicles



def simulate_without_ego(interactive_scenario_path: str,
                         output_folder_path: str = None,
                         create_video: bool = False,
                         use_sumo_manager: bool = False,
                         num_of_steps=None) -> Tuple[Scenario, PlanningProblemSet]:
    """
    Simulates an interactive scenario without ego vehicle

    :param interactive_scenario_path: path to the interactive scenario folder
    :param output_folder_path: path to the output folder
    :param create_video: indicates whether to create a mp4 of the simulated scenario
    :param use_sumo_manager: indicates whether to use the SUMO Manager
    :param num_of_steps: max. number of simulated time steps
    :return: Tuple of the simulated scenario and the planning problem set
    """
    conf = load_sumo_configuration(interactive_scenario_path)
    scenario_file = os.path.join(interactive_scenario_path, f"{conf.scenario_name}.cr.xml")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    scenario_wrapper = ScenarioWrapper()
    scenario_wrapper.sumo_cfg_file = os.path.join(interactive_scenario_path, f"{conf.scenario_name}.sumo.cfg")
    scenario_wrapper.initial_scenario = scenario

    num_of_steps = conf.simulation_steps if num_of_steps is None else num_of_steps
    # simulation without ego vehicle
    simulated_scenario_without_ego, _ = simulate_scenario(SimulationOption.WITHOUT_EGO, conf,
                                                          scenario_wrapper,
                                                          interactive_scenario_path,
                                                          num_of_steps=num_of_steps,
                                                          planning_problem_set=planning_problem_set,
                                                          solution=None,
                                                          use_sumo_manager=use_sumo_manager)
    simulated_scenario_without_ego.scenario_id = scenario.scenario_id

    if create_video:
        create_video_for_simulation(simulated_scenario_without_ego, output_folder_path, planning_problem_set,
                                    {}, SimulationOption.WITHOUT_EGO.value)

    return simulated_scenario_without_ego, planning_problem_set


def simulate_with_solution(interactive_scenario_path: str,
                           output_folder_path: str = None,
                           solution: Solution = None,
                           create_video: bool = False,
                           use_sumo_manager: bool = False,
                           create_ego_obstacle: bool = False) -> Tuple[Scenario, PlanningProblemSet, Dict[int, EgoVehicle]]:
    """
    Simulates an interactive scenario with a given solution

    :param interactive_scenario_path: path to the interactive scenario folder
    :param output_folder_path: path to the output folder
    :param solution: solution to the planning problem
    :param create_video: indicates whether to create a mp4 of the simulated scenario
    :param use_sumo_manager: indicates whether to use the SUMO Manager
    :param create_ego_obstacle: indicates whether to create obstacles as the ego vehicles
    :return: Tuple of the simulated scenario and the planning problem set
    """
    if not isinstance(solution, Solution):
        raise Exception("Solution to the planning problem is not given.")

    conf = load_sumo_configuration(interactive_scenario_path)
    scenario_file = os.path.join(interactive_scenario_path, f"{conf.scenario_name}.cr.xml")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    scenario_wrapper = ScenarioWrapper()
    sumo_cfg_file = os.path.join(interactive_scenario_path, f"{conf.scenario_name}.sumo.cfg")
    scenario_wrapper.initialize(conf.scenario_name, sumo_cfg_file, scenario_file)
    scenario_with_solution, ego_vehicles = simulate_scenario(SimulationOption.SOLUTION, conf,
                                                             scenario_wrapper,
                                                             interactive_scenario_path,
                                                             num_of_steps=conf.simulation_steps,
                                                             planning_problem_set=planning_problem_set,
                                                             solution=solution,
                                                             use_sumo_manager=use_sumo_manager)
    scenario_with_solution.scenario_id = scenario.scenario_id

    if create_video:
        create_video_for_simulation(scenario_with_solution, output_folder_path, planning_problem_set,
                                    ego_vehicles, SimulationOption.SOLUTION.value)

    if create_ego_obstacle:
        for pp_id, planning_problem in planning_problem_set.planning_problem_dict.items():
            obstacle_ego = ego_vehicles[pp_id].get_dynamic_obstacle()
            scenario_with_solution.add_objects(obstacle_ego)

    return scenario_with_solution, planning_problem_set, ego_vehicles


def simulate_with_planner(interactive_scenario_path: str,
                          output_folder_path: str = None,
                          create_video: bool = False,
                          use_sumo_manager: bool = False,
                          create_ego_obstacle: bool = False) \
        -> Tuple[Scenario, PlanningProblemSet, Dict[int, EgoVehicle]]:
    """
    Simulates an interactive scenario with a plugged in motion planner

    :param interactive_scenario_path: path to the interactive scenario folder
    :param output_folder_path: path to the output folder
    :param create_video: indicates whether to create a mp4 of the simulated scenario
    :param use_sumo_manager: indicates whether to use the SUMO Manager
    :param create_ego_obstacle: indicates whether to create obstacles from the planned trajectories as the ego vehicles
    :return: Tuple of the simulated scenario, planning problem set, and list of ego vehicles
    """
    conf = load_sumo_configuration(interactive_scenario_path)
    scenario_file = os.path.join(interactive_scenario_path, f"{conf.scenario_name}.cr.xml")
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()

    scenario_wrapper = ScenarioWrapper()
    scenario_wrapper.sumo_cfg_file = os.path.join(interactive_scenario_path, f"{conf.scenario_name}.sumo.cfg")
    scenario_wrapper.initial_scenario = scenario

    scenario_with_planner, ego_vehicles = simulate_scenario(SimulationOption.MOTION_PLANNER, conf,
                                                            scenario_wrapper,
                                                            interactive_scenario_path,
                                                            num_of_steps=conf.simulation_steps,
                                                            planning_problem_set=planning_problem_set,
                                                            use_sumo_manager=use_sumo_manager)
    scenario_with_planner.scenario_id = scenario.scenario_id

    if create_video:
        create_video_for_simulation(scenario_with_planner, output_folder_path, planning_problem_set,
                                    ego_vehicles, SimulationOption.MOTION_PLANNER.value)

    if create_ego_obstacle:
        for pp_id, planning_problem in planning_problem_set.planning_problem_dict.items():
            obstacle_ego = ego_vehicles[pp_id].get_dynamic_obstacle()
            scenario_with_planner.add_objects(obstacle_ego)

    return scenario_with_planner, planning_problem_set, ego_vehicles


def load_sumo_configuration(interactive_scenario_path: str) -> DefaultConfig:
    with open(os.path.join(interactive_scenario_path, "simulation_config.p"), "rb") as input_file:
        conf = pickle.load(input_file)

    return conf


def check_trajectories(solution: Solution, pps: PlanningProblemSet, config: DefaultConfig):
    assert len(set(solution.planning_problem_ids) - set(pps.planning_problem_dict.keys())) == 0, \
        f"Provided solution trajectories with IDs {solution.planning_problem_ids} don't match " \
        f"planning problem IDs{list(pps.planning_problem_dict.keys())}"

    for s in solution.planning_problem_solutions:
        if s.trajectory.final_state.time_step < config.simulation_steps:
            raise ValueError(f"The simulation requires {config.simulation_steps} "
                             f"states, but the solution only provides"
                             f"{s.trajectory.final_state.time_step} time steps!")


def create_video_for_simulation(scenario_with_planner: Scenario, output_folder_path: str,
                                planning_problem_set: PlanningProblemSet,
                                ego_vehicles: Optional[Dict[int, EgoVehicle]],
                                suffix: str, follow_ego: bool = True):
    """Creates the mp4 animation for the simulation result."""
    if not output_folder_path:
        print("Output folder not specified, skipping mp4 generation.")
        return

    # create mp4 animation
    create_video(scenario_with_planner,
                 output_folder_path,
                 planning_problem_set=planning_problem_set,
                 trajectory_pred=ego_vehicles,
                 follow_ego=follow_ego,
                 suffix=suffix)
