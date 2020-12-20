""""
Minimal example to simulate interactive scenarios
"""
__author__ = "Edmond Irani Liu"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = []
__version__ = "0.5"
__maintainer__ = "Edmond Irani Liu"
__email__ = "edmond.irani@tum.de"
__status__ = "Integration"

import matplotlib as mpl

mpl.use('TkAgg')
from simulation.simulations import simulate_without_ego, simulate_with_solution, simulate_with_planner
from simulation.utility import save_solution
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad.scenario.scenario import Tag


def main():
    # specify required arguments
    name_scenario = "USA_US101-26_2_I-1-1"
    path_scenario = "../scenarios/NGSIM/US101/" + name_scenario

    # for simulation with a given solution trajectory
    name_solution = "KS2:SM1:USA_US101-26_2_T-1:2020a"
    # name_solution = "solution_KS2:SM1:USA_US101-26_2_I-1:2020a"

    path_solutions = "../outputs/solutions/"
    solution = CommonRoadSolutionReader.open(path_solutions + name_solution + ".xml")

    # path to store output GIFs
    path_gif = "../outputs/gifs/"

    # path to store simulated scenarios
    path_scenarios_simulated = "../outputs/simulated/"

    author = 'Max Mustermann'
    affiliation = 'Technical University of Munich, Germany'
    source = ''
    tags = {Tag.URBAN}

    simulation_without_ego = True
    simulation_with_planner = True
    simulation_with_solution = True

    if simulation_without_ego:
        # simulation without ego vehicle
        scenario_without_ego, pps = simulate_without_ego(interactive_scenario_path=path_scenario,
                                                         output_folder_path=path_gif,
                                                         create_GIF=True)
        # write simulated scenario to file
        fw = CommonRoadFileWriter(scenario_without_ego, pps, author, affiliation, source, tags)
        fw.write_to_file(f"{path_scenarios_simulated}{name_scenario}_no_ego.xml", OverwriteExistingFile.ALWAYS)

    if simulation_with_planner:
        # simulation with plugged-in motion planner
        scenario_with_planner, pps, traj_planner = simulate_with_planner(interactive_scenario_path=path_scenario,
                                                                         output_folder_path=path_gif,
                                                                         create_GIF=True)
        if scenario_with_planner:
            # write simulated scenario to file
            fw = CommonRoadFileWriter(scenario_with_planner, pps, author, affiliation, source, tags)
            fw.write_to_file(f"{path_scenarios_simulated}{name_scenario}_planner.xml", OverwriteExistingFile.ALWAYS)

            # saves trajectory to solution file
            save_solution(scenario_with_planner, list(pps.planning_problem_dict)[0], traj_planner,
                          path_solutions, overwrite=True)

    if simulation_with_solution:
        # simulation with solution trajectory
        scenario_with_solution, pps, traj_solution = simulate_with_solution(interactive_scenario_path=path_scenario,
                                                                            output_folder_path=path_gif,
                                                                            solution=solution,
                                                                            create_GIF=True)
        if scenario_with_solution:
            # write simulated scenario to file
            fw = CommonRoadFileWriter(scenario_with_solution, pps, author, affiliation, source, tags)
            fw.write_to_file(f"{path_scenarios_simulated}{name_scenario}_solution.xml", OverwriteExistingFile.ALWAYS)


if __name__ == '__main__':
    main()
