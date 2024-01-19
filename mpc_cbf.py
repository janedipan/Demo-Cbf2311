import do_mpc
from casadi import *

import config


class MPC:
    """MPC-CBF Optimization problem:

    min Σ_{k=0}{N-1} 1/2*x'_k^T*Q*x'_k + 1/2*u_k^T*R*u_k   over u
    s.t.
        x_{k+1} = x_k + B*u_k*T_s
        x_min <= x_k <= x_max
        u_min <= u_k <= u_max
        x_0 = x(0)
        Δh(x_k, u_k) >= -γ*h(x_k)

    where x'_k = x_{des_k} - x_k
    """
    def __init__(self):
        self.sim_time = config.sim_time          # Total simulation time steps
        self.Ts = config.Ts                      # Sampling time
        self.T_horizon = config.T_horizon        # Prediction horizon
        self.x0 = config.x01                      # Initial pose
        self.v_limit = config.v_limit            # Linear velocity limit
        self.omega_limit = config.omega_limit    # Angular velocity limit
        self.R = config.R                        # Controls cost matrix
        self.Q = config.Q                        # State cost matrix
        self.static_obstacles_on = config.static_obstacles_on  # Whether to have static obstacles
        self.moving_obstacles_on = config.moving_obstacles_on  # Whether to have moving obstacles
        self.obs_num = 0
        self.acbf_scale = config.scale            # for test scale of mpc-acbf
        if self.static_obstacles_on:
            self.obs = config.obs                # Static Obstacles
            self.obs_num = len(self.obs)
        if self.moving_obstacles_on:             # Moving obstacles
            self.moving_obs = config.moving_obs
            self.obs_num = len(self.moving_obs)
        self.r = config.r                        # Robot radius
        self.control_type = config.control_type  # "setpoint" or "traj_tracking"
        if self.control_type == "setpoint":      # Go-to-goal
            self.goal = config.goal1              # Robot's goal pose
        self.gamma = config.gamma                # CBF parameter
        self.safety_dist = config.safety_dist    # Safety distance
        self.controller = config.controller      # Type of control

        self.model = self.define_model()
        self.mpc = self.define_mpc()
        self.simulator = self.define_simulator()
        self.estimator = do_mpc.estimator.StateFeedback(self.model)
        self.set_init_state()

    def define_model(self):
        """Configures the dynamical model of the system (and part of the objective function).

        x_{k+1} = x_k + B*u_k*T_s
        Returns:
          - model(do_mpc.model.Model): The system model
        """

        model_type = 'discrete'
        model = do_mpc.model.Model(model_type)

        # States
        n_states = 5
        _x = model.set_variable(var_type='_x', var_name='x', shape=(n_states, 1))

        # Inputs
        n_controls = 2
        _u = model.set_variable(var_type='_u', var_name='u', shape=(n_controls, 1))

        # Uncertain Parameter
        if self.controller == "MPC-ACBF":
            if self.obs_num == 2:
                model.set_variable(var_type='_p', var_name='tau0')
                model.set_variable(var_type='_p', var_name='tau1')
            elif self.obs_num == 1:
                model.set_variable(var_type='_p', var_name='tau0')


        # State Space matrices
        A = np.zeros([5,5])
        A[:3,:3]=np.eye(3)
        B = self.get_sys_matrix_B(_x)

        # Set right-hand-side of ODE for all introduced states (_x).
        x_next = A@_x + B@_u
        model.set_rhs('x', x_next, process_noise=False)  # Set to True if adding noise

        # Optional: Define an expression, which represents the stage and terminal
        # cost of the control problem. This term will be later used as the cost in
        # the MPC formulation and can be used to directly plot the trajectory of
        # the cost of each state.
        model, cost_expr = self.get_cost_expression(model)
        model.set_expression(expr_name='cost', expr=cost_expr)

        # Moving obstacle (define time-varying parameter for its position)
        if self.moving_obstacles_on is True:
            for i in range(len(self.moving_obs)):
                model.set_variable('_tvp', 'x_moving_obs'+str(i))
                model.set_variable('_tvp', 'y_moving_obs'+str(i))
                model.set_variable('_tvp', 'dx_moving_obs'+str(i))
                model.set_variable('_tvp', 'dy_moving_obs'+str(i))
                model.set_variable('_tvp', 'act_moving_obs'+str(i))

        # Setup model
        model.setup()
        return model

    # @staticmethod
    def get_sys_matrix_B(self, x):
        """Defines the system input matrix B.

        Inputs:
          - x(casadi.casadi.SX): The state vector [3x1]
        Returns:
          - B(casadi.casadi.SX): The system input matrix B [3x2]
        """
        a = 1e-9  # Small positive constant so system has relative degree 1
        B = SX.zeros(5, 2)
        B[0, 0] = cos(x[2])*self.Ts
        B[0, 1] = -a*sin(x[2])*self.Ts
        B[1, 0] = sin(x[2])*self.Ts
        B[1, 1] = a*cos(x[2])*self.Ts
        B[2, 1] = self.Ts
        B[3, 0] = cos(x[2])
        B[4, 0] = sin(x[2])
        return B

    def get_cost_expression(self, model):
        """Defines the objective function wrt the state cost depending on the type of control.

        Inputs:
          - model(do_mpc.model.Model):         The system model
        Returns:
          - model(do_mpc.model.Model):         The system model
          - cost_expression(casadi.casadi.SX): The objective function cost wrt the state
        """

        if self.control_type == "setpoint":  # Go-to-goal
            # Define state error
            X = SX.zeros(3, 1)
            X[0] = model.x['x', 0] - self.goal[0]
            X[1] = model.x['x', 1] - self.goal[1]
            X[2] = model.x['x', 2] - self.goal[2]

        else:                                # Trajectory tracking
            # Set time-varying parameters for the objective function
            model.set_variable('_tvp', 'x_set_point')
            model.set_variable('_tvp', 'y_set_point')

            # Define state error
            theta_des = np.arctan2(model.tvp['y_set_point'] - model.x['x', 1], model.tvp['x_set_point'] - model.x['x', 0])
            X = SX.zeros(3, 1)
            X[0] = model.x['x', 0] - model.tvp['x_set_point']
            X[1] = model.x['x', 1] - model.tvp['y_set_point']
            X[2] = np.arctan2(sin(theta_des - model.x['x', 2]), cos(theta_des - model.x['x', 2]))

        cost_expression = transpose(X)@self.Q@X
        return model, cost_expression

    def define_mpc(self):
        """Configures the mpc controller.

        Returns:
          - mpc(do_mpc.model.MPC): The mpc controller
        """

        mpc = do_mpc.controller.MPC(self.model)
        # Set parameters
        setup_mpc = {'n_robust': 0,  # Robust horizon
                     'n_horizon': self.T_horizon,
                     't_step': self.Ts,
                     'state_discretization': 'discrete',
                     'store_full_solution': True,
                     # 'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
                     }
        mpc.set_param(**setup_mpc)

        # Configure objective function
        mterm = self.model.aux['cost']  # Terminal cost
        lterm = self.model.aux['cost']  # Stage cost
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(u=self.R)         # Input penalty (R diagonal matrix in objective fun)

        # State and input bounds
        max_u = np.array([self.v_limit, self.omega_limit])
        mpc.bounds['lower', '_u', 'u'] = -max_u
        mpc.bounds['upper', '_u', 'u'] = max_u

        # Set uncertaint parameter
        if self.controller == "MPC-ACBF":
            if self.obs_num == 2:
                mpc.set_uncertainty_values(tau0=np.zeros(1))
                mpc.set_uncertainty_values(tau1=np.zeros(1))
            elif self.obs_num == 1:
                mpc.set_uncertainty_values(tau0=np.zeros(1))

        # If trajectory tracking or moving obstacles: Define time-varying parameters
        # 调用mpc.set_tvp_fun()函数-时变变量
        if self.control_type == "traj_tracking" or self.moving_obstacles_on is True:
            mpc = self.set_tvp_for_mpc(mpc)

        # Add safety constraints
        if self.static_obstacles_on or self.moving_obstacles_on:
            if self.controller == "MPC-DC":
                # MPC-DC: Add obstacle avoidance constraints
                mpc = self.add_obstacle_constraints(mpc)
            elif self.controller == "MPC-SCBF":
                # MPC-CBF: Add CBF constraints
                mpc = self.add_scbf_constraints(mpc)
            elif self.controller == "MPC-DCBF":
                mpc = self.add_dcbf_constraints(mpc)
            elif self.controller == "MPC-ACBF":
                mpc = self.add_acbf_constraints(mpc)

        mpc.setup()
        return mpc

    def add_obstacle_constraints(self, mpc:do_mpc.controller.MPC):
        """Adds the obstacle constraints to the mpc controller. (MPC-DC)

        Inputs:
          - mpc(do_mpc.controller.MPC): The mpc controller
        Returns:
          - mpc(do_mpc.controller.MPC): The mpc controller with obstacle constraints added
        """
        

        if self.static_obstacles_on:
            i = 0
            for x_obs, y_obs, r_obs in self.obs:
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - x_obs)**2 + 
                                  (self.model.x['x'][1] - y_obs)**2)
                                
                obs_avoid = - (self.model.x['x'][0] - x_obs)**2 \
                            - (self.model.x['x'][1] - y_obs)**2 \
                            + (self.r + r_obs + self.safety_dist)**2
                obs_avoid = if_else(distance_p<=SX(config.detection_range), obs_avoid, SX(0.0))
                mpc.set_nl_cons('obstacle_constraint'+str(i), obs_avoid, ub=0)
                i += 1

        if self.moving_obstacles_on:
            for i in range(len(self.moving_obs)):
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - self.model.tvp['x_moving_obs'+str(i)])**2 + 
                                    (self.model.x['x'][1] - self.model.tvp['y_moving_obs'+str(i)])**2)

                obs_avoid = - (self.model.x['x'][0] - self.model.tvp['x_moving_obs'+str(i)])**2 \
                            - (self.model.x['x'][1] - self.model.tvp['y_moving_obs'+str(i)])**2 \
                            + (self.r + self.moving_obs[i][4] + self.safety_dist)**2
                obs_avoid = if_else(distance_p<=SX(config.detection_range), obs_avoid, SX(0.0))
                mpc.set_nl_cons('moving_obstacle_constraint'+str(i), obs_avoid, ub=0)

        return mpc

    def add_scbf_constraints(self, mpc:do_mpc.controller.MPC):
        """Adds the CBF constraints to the mpc controller. (MPC-CBF)

        Inputs:
          - mpc(do_mpc.controller.MPC): The mpc controller
        Returns:
          - mpc(do_mpc.controller.MPC): The mpc model with CBF constraints added
        """
        cbf_constraints = self.get_scbf_constraints()   # can choose scbf, dcbf, acbf, dc
        i = 0
        for cbc in cbf_constraints:
            mpc.set_nl_cons('cbf_constraint'+str(i), cbc, ub=0)
            i += 1
        return mpc

    def add_dcbf_constraints(self, mpc:do_mpc.controller.MPC):
        cbf_constraints = self.get_dcbf_constraints()   # can choose scbf, dcbf, acbf, dc
        i = 0
        for cbc in cbf_constraints:
            mpc.set_nl_cons('cbf_constraint'+str(i), cbc, ub=0)
            i += 1
        return mpc

    def add_acbf_constraints(self, mpc:do_mpc.controller.MPC):
        cbf_constraints = self.get_acbf_constraints()   # can choose scbf, dcbf, acbf, dc
        i = 0
        for cbc in cbf_constraints:
            mpc.set_nl_cons('cbf_constraint'+str(i), cbc, ub=0)
            # mpc.set_nl_cons('cbf_constraint'+str(i), cbc, ub=0, soft_constraint=True, penalty_term_cons=1500, maximum_violation=0.1)
            i += 1
        return mpc

    # 静态控制屏障函数
    def get_scbf_constraints(self):
        """Computes the CBF constraints for all obstacles.

        Returns:
          - cbf_constraints(list): The CBF constraints for each obstacle
        """
        # Get state vector x_{t+k+1}
        A = np.zeros([5,5])
        A[:3,:3]=np.eye(3)
        B = self.get_sys_matrix_B(self.model.x['x'])
        x_k1 = A@self.model.x['x'] + B@self.model.u['u']

        # Compute CBF constraints
        cbf_constraints = []
        # 静态障碍物
        if self.static_obstacles_on:
            for obs in self.obs:
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - self.obs[0])**2 + 
                                  (self.model.x['x'][1] - self.obs[1])**2)
                
                h_k1 = self.h(x_k1, obs)
                h_k = self.h(self.model.x['x'], obs)
                st_cbf = -h_k1 + (1-self.gamma)*h_k
                st_cbf = if_else(distance_p<=SX(config.detection_range), st_cbf, 0.0)
                cbf_constraints.append(st_cbf)
        # 动态障碍物
        if self.moving_obstacles_on:
            for i in range(len(self.moving_obs)):
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - self.model.tvp['x_moving_obs'+str(i)])**2 + 
                    (self.model.x['x'][1] - self.model.tvp['y_moving_obs'+str(i)])**2)

                obs = (self.model.tvp['x_moving_obs'+str(i)], self.model.tvp['y_moving_obs'+str(i)], self.moving_obs[i][4])
                h_k1 = self.h(x_k1, obs)
                h_k = self.h(self.model.x['x'], obs)
                st_cbf = -h_k1 + (1-self.gamma)*h_k
                st_cbf = if_else(distance_p<=SX(config.detection_range), st_cbf, 0.0)
                cbf_constraints.append(st_cbf)

        return cbf_constraints

    # 动态控制屏障函数
    def get_dcbf_constraints(self):
        # Get state vector x_{t+k+1}
        A = np.zeros([5,5])
        A[:3,:3]=np.eye(3)
        B = self.get_sys_matrix_B(self.model.x['x'])
        x_k1 = A@self.model.x['x'] + B@self.model.u['u']

        # Compute CBF constraints
        cbf_constraints = []
        # 静态障碍物
        if self.static_obstacles_on:
            for obs in self.obs:
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - self.obs[0])**2 + 
                                  (self.model.x['x'][1] - self.obs[1])**2)
                
                h_k1 = self.h(x_k1, obs)
                h_k = self.h(self.model.x['x'], obs)
                st_cbf = -h_k1 + (1-self.gamma)*h_k
                st_cbf = if_else(distance_p<=SX(config.detection_range), st_cbf, 0.0)
                cbf_constraints.append(st_cbf)
        # 动态障碍物
        if self.moving_obstacles_on:
            for i in range(len(self.moving_obs)):
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - self.model.tvp['x_moving_obs'+str(i)])**2 + 
                    (self.model.x['x'][1] - self.model.tvp['y_moving_obs'+str(i)])**2)
                obs = (self.model.tvp['x_moving_obs'+str(i)], 
                       self.model.tvp['y_moving_obs'+str(i)], 
                       self.moving_obs[i][4])

                obs1 = (self.model.tvp['x_moving_obs'+str(i)] + self.model.tvp['dx_moving_obs'+str(i)]*self.Ts*self.model.tvp['act_moving_obs'+str(i)],
                        self.model.tvp['y_moving_obs'+str(i)] + self.model.tvp['dy_moving_obs'+str(i)]*self.Ts*self.model.tvp['act_moving_obs'+str(i)],
                        self.moving_obs[i][4])
                h_k1 = self.h(x_k1, obs1)
                h_k = self.h(self.model.x['x'], obs)
                st_cbf = -h_k1 + (1-self.gamma)*h_k
                st_cbf = if_else(distance_p<=SX(config.detection_range), st_cbf, 0.0)
                cbf_constraints.append(st_cbf)

        return cbf_constraints

    # -------------------------------------------------------超前控制屏障函数(Ours)
    def get_acbf_constraints(self):
        # Get state vector x_{t+k+1}
        A = np.zeros([5,5])
        A[:3,:3]=np.eye(3)
        B = self.get_sys_matrix_B(self.model.x['x'])
        x_k1 = A@self.model.x['x'] + B@self.model.u['u']

        # Compute CBF constraints
        cbf_constraints = []
        # 静态障碍物
        if self.static_obstacles_on:
            for obs in self.obs:
                # 检测距离限制
                distance_p = sqrt((self.model.x['x'][0] - self.obs[0])**2 + 
                                  (self.model.x['x'][1] - self.obs[1])**2)
                
                h_k1 = self.h(x_k1, obs)
                h_k = self.h(self.model.x['x'], obs)
                st_cbf = -h_k1 + (1-self.gamma)*h_k
                st_cbf = if_else(distance_p<=SX(config.detection_range), st_cbf, 0.0)
                cbf_constraints.append(st_cbf)
        # 动态障碍物
        if self.moving_obstacles_on:
            for i in range(len(self.moving_obs)):
                # 检测距离限制
                _tau = self.model.p['tau'+str(i)]
                distance_p = sqrt((self.model.x['x'][0] - self.model.tvp['x_moving_obs'+str(i)])**2 + 
                    (self.model.x['x'][1] - self.model.tvp['y_moving_obs'+str(i)])**2)
                obs = (self.model.tvp['x_moving_obs'+str(i)], 
                       self.model.tvp['y_moving_obs'+str(i)], 
                       self.moving_obs[i][4])

                obs1 = (self.model.tvp['x_moving_obs'+str(i)] + self.model.tvp['dx_moving_obs'+str(i)]*self.Ts*self.model.tvp['act_moving_obs'+str(i)],
                        self.model.tvp['y_moving_obs'+str(i)] + self.model.tvp['dy_moving_obs'+str(i)]*self.Ts*self.model.tvp['act_moving_obs'+str(i)],
                        self.moving_obs[i][4])
                v_r = (self.model.tvp['dx_moving_obs'+str(i)]-self.model.x['x'][3], 
                       self.model.tvp['dy_moving_obs'+str(i)]-self.model.x['x'][4])
                h_k1 = self.h1(x_k1, obs1, v_r, _tau)
                h_k = self.h1(self.model.x['x'], obs, v_r, _tau)
                st_cbf = -h_k1 + (1-self.gamma)*h_k
                st_cbf = if_else(distance_p<=SX(config.detection_range), st_cbf, 0.0)
                cbf_constraints.append(st_cbf)

        return cbf_constraints

    def h1(self, x, obstacle:tuple, v_r:tuple, _tau=0.0):
        x_obs, y_obs, r_obs = obstacle
        h = sqrt((x_obs-x[0]+v_r[0]*_tau)**2 + (y_obs - x[1]+v_r[1]*_tau)**2) - (self.r + r_obs + self.safety_dist)
        return h

    def h(self, x, obstacle:tuple):
        """Computes the Control Barrier Function.
        
        Inputs:
          - x(casadi.casadi.SX): The state vector [3x1]
          - obstacle(tuple):     The obstacle position and radius
        Returns:
          - h(casadi.casadi.SX): The Control Barrier Function
        """
        x_obs, y_obs, r_obs = obstacle
        h = sqrt((x_obs-x[0])**2 + (y_obs - x[1])**2) - (self.r + r_obs + self.safety_dist)
        return h

    def set_tvp_for_mpc(self, mpc:do_mpc.controller.MPC):
        """Sets the trajectory for trajectory tracking and/or the moving obstacles' trajectory.

        Inputs:
          - mpc(do_mpc.controller.MPC): The mpc controller
        Returns:
          - mpc(do_mpc.controller.MPC): The mpc model with time-varying parameters added
        """
        tvp_struct_mpc = mpc.get_tvp_template()

        def tvp_fun_mpc(t_now):
            if self.control_type == "traj_tracking":
                # Trajectory to follow
                if config.trajectory == "circular":
                    x_traj = config.A*cos(config.w*t_now)-config.A
                    y_traj = config.A*sin(config.w*t_now)
                elif config.trajectory == "infinity":
                    x_traj = config.A*cos(config.w*t_now)/(sin(config.w*t_now)**2 + 1)
                    y_traj = config.A*sin(config.w*t_now)*cos(config.w*t_now)/(sin(config.w*t_now)**2 + 1)
                elif config.trajectory == "oneline":
                    x_traj = min(config.A, config.v_limit*t_now*1.1)
                    y_traj = 0.0                  
                else:
                    print("Select one of the available options for trajectory.")
                    exit()

                tvp_struct_mpc['_tvp', :, 'x_set_point'] = x_traj
                tvp_struct_mpc['_tvp', :, 'y_set_point'] = y_traj

            if self.moving_obstacles_on is True:
                # Moving obstacles trajectory
                for i in range(len(self.moving_obs)):
                    # 分配障碍物开始&结束运动的时间
                    if i == 0:
                        t_min = 0.0         # 0.0
                        t_max = 5.0         # 3.0: 4-5
                    elif i == 1:
                        t_min = 2.5         # 2.5
                        t_max = 6.0         # 7.0
                    
                    # 动态障碍物分段处理: 等待，结束，运行
                    if t_now < t_min:
                        dt = 0.0
                        tvp_struct_mpc['_tvp', :, 'act_moving_obs'+str(i)] = 0
                    elif t_now > t_max:
                        dt = t_max-t_min
                        tvp_struct_mpc['_tvp', :, 'act_moving_obs'+str(i)] = 0
                    else:
                        dt = t_now-t_min
                        tvp_struct_mpc['_tvp', :, 'act_moving_obs'+str(i)] = 1
                    tvp_struct_mpc['_tvp', :, 'x_moving_obs'+str(i)] = self.moving_obs[i][0]*dt + self.moving_obs[i][1]
                    tvp_struct_mpc['_tvp', :, 'y_moving_obs'+str(i)] = self.moving_obs[i][2]*dt + self.moving_obs[i][3]
                    tvp_struct_mpc['_tvp', :, 'dx_moving_obs'+str(i)] = 0.0 if dt!=t_now-t_min else self.moving_obs[i][0]
                    tvp_struct_mpc['_tvp', :, 'dy_moving_obs'+str(i)] = 0.0 if dt!=t_now-t_min else self.moving_obs[i][2]
            return tvp_struct_mpc
        mpc.set_tvp_fun(tvp_fun_mpc)
        return mpc

    def define_simulator(self):
        """Configures the simulator.

        Returns:
          - simulator(do_mpc.simulator.Simulator): The simulator
        """
        simulator = do_mpc.simulator.Simulator(self.model)
        simulator.set_param(t_step=self.Ts)

        # If trajectory tracking or moving obstacles: Add time-varying parameters——时变参数
        if self.control_type == "traj_tracking" or self.moving_obstacles_on is True:
            tvp_template = simulator.get_tvp_template()

            def tvp_fun(t_now):
                return tvp_template
            simulator.set_tvp_fun(tvp_fun)

        # 为simulator设置动态tau值——参数
        if self.controller == "MPC-ACBF":
            p_template = simulator.get_p_template()
            def p_fun(t_now):
                return p_template
            simulator.set_p_fun(p_fun)

        simulator.setup()

        return simulator

    def set_init_state(self):
        """Sets the initial state in all components."""
        self.mpc.x0 = self.x0
        self.simulator.x0 = self.x0
        self.estimator.x0 = self.x0
        self.mpc.set_initial_guess()

    # 启动仿真主循环
    def run_simulation(self):
        """Runs a closed-loop control simulation."""
        x0 = self.x0
        k = 0
        scale_s = []
        # while (np.linalg.norm(x0.reshape(5,)[:3]-np.array(self.goal).reshape(5,)[:3])>5e-2) & (k < self.sim_time):
        while(k < self.sim_time):
            # 更新系统参数
            print(k)
            if (self.controller == "MPC-ACBF"):
                obs_list = []
                if self.static_obstacles_on:
                    for i in range(self.obs_num):
                        obs_list.append(np.array([0.0, 
                                                  self.obs[i][0], 
                                                  0.0, 
                                                  self.obs[i][1], 
                                                  self.obs[2]]))
                elif self.moving_obstacles_on:
                    for i in range(self.obs_num):
                        if k == 0:
                            obs_k1 = np.array(self.moving_obs[i])
                            obs_k1[0] = obs_k1[2] = 0.0
                            obs_list.append(obs_k1)
                        else:
                            sign_act = self.mpc.data['_tvp', 'act_moving_obs'+str(i)][-1]
                            obs_k   = np.array([self.mpc.data['_tvp', 'dx_moving_obs'+str(i)][-1].sum(),
                                                self.mpc.data['_tvp', 'x_moving_obs'+str(i)][-1].sum(),
                                                self.mpc.data['_tvp', 'dy_moving_obs'+str(i)][-1].sum(),
                                                self.mpc.data['_tvp', 'y_moving_obs'+str(i)][-1].sum(),
                                                self.moving_obs[i][4]])
                            if sign_act == 0:
                                obs_k1 = obs_k
                            else:
                                obs_k1 = obs_k
                                obs_k1[1] = obs_k[1] + obs_k[0]*self.Ts
                                obs_k1[3] = obs_k[3] + obs_k[2]*self.Ts
                            obs_list.append(obs_k1)

                p_template = self.updata_parameter_for_mpc(scale_s, _k=k, _ro_state=np.array([i.sum() for i in x0]), _obs_state=obs_list)
                print(p_template)
                if self.obs_num == 2:
                    self.mpc.set_uncertainty_values(tau0=np.array([p_template[0]]), 
                                                    tau1=np.array([p_template[1]]))
                elif self.obs_num == 1:
                    self.mpc.set_uncertainty_values(tau0=np.array([p_template[0]]))
            # 输入当前状态量，求解最优控制量，只有执行以下步骤才会将mpc.data数据才会更新
            u0 = self.mpc.make_step(x0)
            y_next = self.simulator.make_step(u0)
            # y_next = self.simulator.make_step(u0, w0=10**(-4)*np.random.randn(3, 1))  # Optional Additive process noise
            x0 = self.estimator.make_step(y_next) 
            k = k+1
        # {'_time': 1, '_x': 5, '_y': 5, '_u': 2, '_z': 0, '_tvp': 0, '_p': 3, '_aux': 2
        print('-----------------mpc controller is: {} ------------------'.format(self.controller))
        print(self.mpc.data.data_fields)
        print(self.mpc.data['_p'])
        # print(self.mpc.data['_u'])
        # print(scale_s)

    def updata_parameter_for_mpc(self, list_s:list, _k:int, _ro_state:np.ndarray, _obs_state:list) -> list:
        tau_list = []
        scale_list = []
        ro_p = _ro_state[:2]
        ro_v = _ro_state[3:]
        # print('robot velocity: ',ro_v)
        for i in range(self.obs_num):
            obs_p   = _obs_state[i][1:4:2]
            obs_v   = _obs_state[i][0:4:2]
            p_r = obs_p-ro_p
            v_r = obs_v-ro_v
            # print('obs',i)
            # print('rob_v: ', ro_v)
            # print('obs_v: ', obs_v)
            # print('p_r: ',p_r)
            # print('v_r: ',v_r)
            # 障碍物的风险判断: 1.当且危险的障碍物 2.检测距离5.0米 3.碰撞时间为2s 会使用tau值，否则为0
            # if (np.dot(p_r, v_r) < 0.0) & (abs(np.linalg.norm(p_r)**2/np.dot(p_r, v_r))<2.0):
            if (np.dot(p_r, v_r) < 0.0) & ((np.linalg.norm(p_r)-config.r-config.moving_obs[0][-1])*np.linalg.norm(p_r)/(-np.dot(p_r, v_r))<2.0):
                # if (_k>0)&(_k<17):
                tau_max = (np.linalg.norm(p_r)-config.r-config.moving_obs[0][-1])*np.linalg.norm(p_r)/(-np.dot(p_r, v_r))                   
                # scale = self.acbf_scale
                tau_max = min(tau_max, 0.65)
                obs_scale = 1-(abs(np.dot(p_r,obs_v)/np.linalg.norm(p_r)/1.5)-1)**2
                ro_scale = 1-(np.linalg.norm(p_r)/5-1)**2
                scale = obs_scale * ro_scale
                result = round(tau_max*scale, 3)
                tau_list.append(result)
                
                scale_list.append(scale)
                # else:
                #     tau_list.append(0.0)
                #     scale_list.append(0.0)
            else:
                tau_list.append(0.0)
                scale_list.append(0.0)
        scale_list.extend(tau_list)
        list_s.append(scale_list)
        print((list_s))
        return tau_list