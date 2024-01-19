import util
import seaborn as sns
import matplotlib as mpl
import matplotlib.pyplot as plt

import config

# -------------------------------------test 4 controller method
def run_sim_4controllers():
    util.run_sim_for_four_controller()                  # 生成pkl数据
    util.compare_results_by_four_controller()           # seaborn绘图    

def trans_dompc_data():
    infile_list = ["DC", "SCBF", "DCBF", "ACBF"]
    file_path = "./results/scenario1/"
    for i in range(len(infile_list)):
        infile_name = "MPC-" + infile_list[i] + "_setpoint"
        outfile_name = "Excel-" + infile_list[i]
        util.trans_mpc2excel(file_path, infile_name, outfile_name)
    pass

if __name__ == '__main__':
    run_sim_4controllers()
    # trans_dompc_data()
    pass
