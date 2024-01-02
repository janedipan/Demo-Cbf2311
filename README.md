# Demo-CBF2311
Model Predictive Control with discrete-time Control Barrier Functions (MPC-CBF) for a wheeled mobile robot.

The MPC-CBF optimization problem is given by:

$$
\begin{aligned}
\min _{u_{t+N-1 \mid t}} & \frac{1}{2} \tilde{x}_N^T Q_x \tilde{x}_N+\sum_{k=0}^{N-1} \frac{1}{2} \tilde{x}_k^T Q_x \tilde{x}_k+\frac{1}{2} u_k^T Q_u u_k \\\text { s.t. } & x_{t+k+1 \mid t}=x_{t+k \mid t}+f\left(x_{t+k \mid t}, u_{t+k \mid t}\right) \cdot T_s, \quad k=0, \ldots, N-1, \\& x_{\min } \leq x_{t+k \mid t} \leq x_{\max }, \quad k=0, \ldots, N-1, \\& u_{\min } \leq u_{t+k \mid t} \leq u_{\max }, \quad k=0, \ldots, N-1, \\& x_{t \mid t}=x_t, \\& \Delta h\left(x_{t+k \mid t}, u_{t+k \mid t}\right) \geq-\gamma h\left(x_{t+k \mid t}\right), \quad k=0, \ldots, N-1
\end{aligned}
$$

可以将硬约束转变为软约束的形式，通常是引入松弛变量的方法的原理-原本的公式如以下:
$$
m(x,u,z,p_{\text{tv}}, p) \leq m_{\text{ub}}
$$

引入松弛变量$\epsilon$后，约束变为以下形式:
$$
\begin{aligned}
m(x,u,z,p_{\text{tv}}, p)-\epsilon \leq m_{\text{ub}} \\
0 \leq \epsilon \leq \epsilon_{\text{max}}
\end{aligned}
$$

松弛变量将添加到成本函数中，并与提供的惩罚项相乘。这种表述使约束变得软化，意味着可以容忍一定的违反，不会导致不可行性。通常建议对惩罚项使用较高的值，以避免对约束条件进行显著违反。

Quick Start
------------

To use this project, install it locally via:
```bash
git clone https://github.com/elena-ecn/mpc-cbf.git
```

The dependencies can be installed by running:
```bash
pip install -r requirements.txt
```

The controller configuration can be changed through the config.py.

To execute the code, run:
```bash
python3 main.py
```


Results
-------

### Scenario 1
**Path comparison for different values of γ for MPC-CBF and with MPC-DC**
<p align="center" width="100%">
    <img src="images/Display/path_comparisons.png" width="400">
    <br>Path comparison
</p>

References
----------
* [[1] J. Zeng, B. Zhang, K. Sreenath, Safety-critical model predictive control with discrete-time control barrier function, 2021 American Control Conference (ACC) (2020) 3882–3889.]()
* [[2] Z. Jian, Z. Yan, X. Lei, Z.-R. Lu, B. Lan, X. Wang, B. Liang, Dynamic control barrier function-based model predictive control to safety-critical obstacle-avoidance of mobile robot, 2023 IEEE International Conference on Robotics and Automation (ICRA) (2022) 3679–3685.]()

修改思路
---
1. 在`mpc_cbf.py`文件中，有静态障碍物，动态障碍物两个标志，影响`define_mpc()`中Add safety constraints的部分，一部分处理静态障碍物的情况，一部分处理动态障碍物的情况；
2. 需要在`config.py`文件中修改系统状态量为$[x,y,\theta] \to [x,y,\theta,\dot{x},\dot{y}]$，相应系统状态方程的系统矩阵，控制矩阵还有包括mpc的Q矩阵需要修改；
3. 在`mpc_cbf.py`文件的`def add_cbf_constraints()`函数进行修改，添加各种cbf约束的接口；
