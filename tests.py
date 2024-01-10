import util
import seaborn as sns
import matplotlib as mpl
import matplotlib.pyplot as plt

def seaborn_test01():
    sns.set_theme(style="ticks")

    diamonds = sns.load_dataset("diamonds")

    f, ax = plt.subplots(figsize=(7, 5))
    sns.despine(f)

    sns.histplot(
        diamonds,
        x="price", hue="cut",
        multiple="stack",
        palette="light:m_r",
        edgecolor=".3",
        linewidth=.5,
        log_scale=True,
    )
    ax.xaxis.set_major_formatter(mpl.ticker.ScalarFormatter())
    ax.set_xticks([500, 1000, 2000, 5000, 10000])
    plt.show()

if __name__ == '__main__':

    # util.compare_results_by_gamma()                   # Compares the path for each method and gamma value
    # util.run_multiple_experiments(N=50)               # Runs N experiments for each method
    # util.compare_controller_results(N=50, gamma=0.1)  # Compares total costs and min distances for each method
    # seaborn_test01()

    util.run_sim_for_four_controller()                # 生成pkl数据
    util.compare_results_by_four_controller()           # seaborn绘图