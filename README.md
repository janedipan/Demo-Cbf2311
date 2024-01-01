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
* [[1] J. Zeng, B. Zhang, and K. Sreenath, “Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function,” in 2021 American Control Conference (ACC), May 2021, pp. 3882–3889.](https://ieeexplore.ieee.org/document/9483029)


