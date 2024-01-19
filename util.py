import pickle
import do_mpc as dp
import numpy as np
import pandas as pd
from do_mpc.data import save_results, load_results

import config
from mpc_cbf import MPC
from plotter import plot_path_comparisons, plot_cost_comparisons, plot_min_distance_comparison, plot_path_comparisons_by4controller, plot_path_comparisons_by_scale


# 保存成excel文件 每一行所需要的属性
robot_states = ["robot_x", "robot_y", "robot_theta"]
robot_controls = ["robot_v", "robot_w"]
obs0_states = ["obs_x0", "obs_y0", "obs_dx0", "obs_dy0"]
obs_list = [obs0_states]
tau_list = ["tau0"]     # for mpc-acbf

def save_mpc_results(controller):
    """Save results in pickle file."""
    if config.controller == "MPC-DC":
        filename = config.controller + '_' + config.control_type
    else:
        filename = config.controller + '_' + config.control_type + '_gamma' + str(config.gamma)
    """
        默认保存的位置是./result/ 表示文件同级的意思
        save_result()设置覆盖开关，对要保存的重名的pkl文件加上前缀
    """
    save_results([controller.mpc, controller.simulator], result_name=filename)


def load_mpc_results(filename):
    """Load results from pickle file."""
    return load_results('./results/' + filename + '.pkl')

def trans_mpc2excel(file_path:str ,infilename:str, outfilename:str):
    infile_path = file_path + infilename + '.pkl'
    outfile_path = file_path + outfilename + '.xlsx'
    data = dp.data.MPCData
    data_dict = {}
    with open(infile_path, 'rb') as f:
        data = pickle.load(f)['mpc']
        length = len(data['_time'])

        data_dict["Time"] = data['_time'].reshape(length)

        for i in range(len(robot_states)):
            data_dict[robot_states[i]] = data['_x'][:,i].reshape(length)

        for i in range(len(robot_controls)):
            data_dict[robot_controls[i]] = data['_u'][:,i].reshape(length)

        for i in range(len(obs0_states)):
            data_dict[obs0_states[i]] = data['_tvp'][:,i].reshape(length)

        if infilename == 'MPC-ACBF_setpoint':
            for i in range(len(tau_list)):
                data_dict[tau_list[i]] = data['_p'][:,i].reshape(length)

        # 补充controller_type
        data_dict['controller'] = infilename[4:-9]

        # 补充额外机器人与障碍物距离的数据
        data_dict['distance']=np.sqrt((data_dict[robot_states[0]]-data_dict['obs_x0'])**2 + 
                                    (data_dict[robot_states[1]]-data_dict['obs_y0'])**2) - config.r - config.moving_obs[0][-1]        

        df = pd.DataFrame(data_dict)
        df.to_excel(outfile_path, index=False)
        print(df)


def compare_controller_results(N, gamma):
    """Compares the total cost and min distances for each method over N experiments."""

    obs = [(1.0, 0.5, 0.1)]  # The obstacles used when creating the experiments

    # Get costs & min distances from results
    costs_cbf = []
    costs_dc = []
    min_distances_cbf = []
    min_distances_dc = []
    for i in range(1, N+1):
        # Filename prefix
        if len(str(i)) == 1:
            num = '00' + str(i)
        elif len(str(i)) == 2:
            num = '0' + str(i)
        else:
            num = str(i)

        # Get cbf result
        filename_cbf = num + "_MPC-CBF_setpoint_gamma" + str(gamma)
        results_cbf = load_mpc_results(filename_cbf)
        total_cost_cbf = sum(results_cbf['mpc']['_aux'][:, 1])
        costs_cbf.append(total_cost_cbf)
        positions = results_cbf['mpc']['_x']
        distances = []
        for p in positions:
            distances.append(((p[0]-obs[0][0])**2 + (p[1]-obs[0][1])**2)**(1/2) - (config.r + obs[0][2]))
        min_distances_cbf.append(min(distances))

        # Get dc result
        filename_dc = num + "_MPC-DC_setpoint"
        results_dc = load_mpc_results(filename_dc)
        total_cost_dc = sum(results_dc['mpc']['_aux'][:, 1])
        costs_dc.append(total_cost_dc)
        positions = results_dc['mpc']['_x']
        distances = []
        for p in positions:
            distances.append(((p[0]-obs[0][0])**2 + (p[1]-obs[0][1])**2)**(1/2) - (config.r + obs[0][2]))
        min_distances_dc.append(min(distances))

    # Plot cost comparisons
    plot_cost_comparisons(costs_dc, costs_cbf, gamma)

    # Plot min distances comparison
    plot_min_distance_comparison(min_distances_cbf, min_distances_dc, gamma)

    print("Average cost over all experiments: cbf={}, dc={}".format(sum(costs_cbf)/len(costs_cbf), sum(costs_dc)/len(costs_dc)))
    print("Average min distance over all experiments: cbf={}, dc={}".format(sum(min_distances_cbf)/len(min_distances_cbf),
                                                                            sum(min_distances_dc)/len(min_distances_dc)))

def run_sim():
    """Runs a simulation and saves the results."""
    controller = MPC()            # Define controller
    controller.run_simulation()   # Run closed-loop control simulation
    save_mpc_results(controller)  # Store results

def run_multiple_experiments(N):
    """Runs N experiments for each method."""

    # Run experiments
    cont_type = ["MPC-CBF", "MPC-DC"]
    for c in cont_type:
        config.controller = c
        for i in range(N):
            run_sim()

def run_sim_for_different_gammas(gammas):
    """Runs simulation for the MPC-DC and for each gamma for the MPC-CBF."""

    # Run simulation for the MPC-DC
    config.controller = "MPC-DC"
    run_sim()

    # Run simulations for each gamma for the MPC-CBF
    config.controller = "MPC-CBF"
    for gamma in gammas:
        config.gamma = gamma
        run_sim()


def run_sim_for_four_controller(scenario=8, gamma=0.3, safety_dist=0.2):
    """
        Runs simulation for the MPC-DC, MPC-SCBF, MPC-DCBF, MPC-ACBF.
    """
    config.scenario = scenario
    config.gamma = gamma
    config.safety_dist = safety_dist

    config.controller = "MPC-DC"
    run_sim()

    config.controller = "MPC-SCBF"
    run_sim()

    config.controller = "MPC-DCBF"
    run_sim()    

    config.controller = "MPC-ACBF"
    run_sim()

def compare_results_by_gamma():
    """Runs simulations and plots path for each method and different gamma values."""

    gammas = [0.1, 0.2, 0.3, 1.0]  # Values to test

    # Run simulations
    run_sim_for_different_gammas(gammas)

    # Load results
    results = [load_mpc_results("MPC-DC_setpoint")]
    for gamma in gammas:
        filename_cbf = "MPC-CBF_setpoint_gamma" + str(gamma)
        results.append(load_mpc_results(filename_cbf))

    # Plot path comparison
    plot_path_comparisons(results, gammas)

def compare_results_by_four_controller():
    controllers = ["SCBF", "DCBF", "ACBF"]
    ga = config.gamma
    results = [load_mpc_results("MPC-DC_setpoint")]
    for x in controllers:
        filename_cbf = "MPC-{}_setpoint_gamma".format(x)+str(ga)
        results.append(load_mpc_results(filename_cbf))
    
    plot_path_comparisons_by4controller(controllers, results, ga)
