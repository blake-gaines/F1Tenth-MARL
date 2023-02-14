from math import sin, cos
# cvxopt
import numpy as np
import cvxopt

#https://github.com/nolfwin/cvxopt_quadprog/blob/master/cvxopt_qp.py
def quadprog(H, f, L=None, k=None, Aeq=None, beq=None, lb=None, ub=None):
    """
    Input: Numpy arrays, the format follows MATLAB quadprog function: https://www.mathworks.com/help/optim/ug/quadprog.html
    Output: Numpy array of the solution
    """
    n_var = H.shape[1]

    P = cvxopt.matrix(H, tc='d')
    q = cvxopt.matrix(f, tc='d')

    if L is not None or k is not None:
        assert(k is not None and L is not None)
        if lb is not None:
            L = np.vstack([L, -np.eye(n_var)])
            k = np.vstack([k, -lb])

        if ub is not None:
            L = np.vstack([L, np.eye(n_var)])
            k = np.vstack([k, ub])

        L = cvxopt.matrix(L, tc='d')
        k = cvxopt.matrix(k, tc='d')

    if Aeq is not None or beq is not None:
        assert(Aeq is not None and beq is not None)
        Aeq = cvxopt.matrix(Aeq, tc='d')
        beq = cvxopt.matrix(beq, tc='d')

    sol = cvxopt.solvers.qp(P, q, L, k, Aeq, beq)

    return np.array(sol['x'])

class CBF():
	"""docstring for CLF_CBF_QP"""
	def __init__(self, cbf_param, veh_param, controller_goal, straightlane, other_vehicles):
		self.param_opt = cbf_param
        self.param_sys = veh_param
        self.goal = controller_goal
        self.straightlane = straightlane
        self.other_vehicles = other_vehicles

    # output: acc_flag, input, e
    def get_optimal_input(self, state, last_input, lane_id, input_log, current_lane_id, direction_flag, acc_flag):
        # load parameter and control objective
        safety_factor = self.goal.safety_factor
        lim_speed = self.goal.lim_speed
        l_rc = self.param_sys.l_rc
        l_fc = self.param_sys.l_fc
        l_f = self.param_sys.l_f
        l_r = self.param_sys.l_r
        width = self.param_sys.width
        dt = self.param_opt.dt
        x, y, psi, v = self.unpack_state(state)
        acc, beta = self.unpack_input(last_input)
        alpha_y = self.param_opt.alpha_y
        alpha_v = self.param_opt.alpha_v
        alpha_yaw = self.param_opt.alpha_yaw
        gamma_1 = self.param_opt.gamma_1
        gamma_2 = self.param_opt.gamma_2
        gamma_3 = self.param_opt.gamma_3
        gamma_4 = self.param_opt.gamma_4
        gamma_5 = self.param_opt.gamma_5
        gamma_6 = self.param_opt.gamma_6
        target_y = self.goal.target_y

        # load surrounding vehicles
        current_lane_vehicles = []
        target_lane_vehicles = []
        num_veh = len(self.other_vehicles)
        # sort surrounding vehilces according to the lane id information
        for i in range(num_veh):
            # the vehicle is in the current lane
            if self.other_vehicles[i].lane_id == current_lane_id or \
                self.other_vehicles[i].lane_id == (current_lane_id - direction_flag * 0.5):
                # collect the vehicle in the current lane before the ego vehicle
                if self.other_vehicles[i].state[0] >= x:
                    current_lane_vehicles.append(self.other_vehicles[i])
            # the vehicle is across the dividing line
            elif self.other_vehicles[i].lane_id == (current_lane_id + direction_flag * 0.5):
                if self.other_vehicles[i].state[0] >= x:
                    current_lane_vehicles.append(self.other_vehicles[i])
                target_lane_vehicles.append(self.other_vehicles[i])
            elif self.other_vehicles[i].lane_id == (current_lane_id + direction_flag):
                target_lane_vehicles.append(self.other_vehicles[i])
            elif self.other_vehicles[i].lane_id == (current_lane_id + 1.5 * direction_flag):
                target_lane_vehicles.append(self.other_vehicles[i])
        
        car_fc = None # car_fc is the closest leading vehicle in the current lane (vehicle fc)
        carfc_range = x + 100 # this value can be tuned, it shows in which range the vehicle would be considered
        for j in range(len(current_lane_vehicles))
            if current_lane_vehicles[j].state[0] <= carfc_range:
                car_fc = current_lane_vehicles[j]
                carfc_range = current_lane_vehicles[j].state[0]
        
        car_bt = None # car_bt is the closeset vehicle in the target lane that is behind ego vehicle (vehicle bt)
        car_bt_range = x - 100
        car_ft =None # car_ft is the closet leading vehicle in the target lane (vehicle ft)
        carft_range = x + 100
        for i in range(len(target_lane_vehicles)):
            if target_lane_vehicles[i].state[0] <= x and target_lane_vehicles[i].state[0] >= car_bt_range:
                car_bt = target_lane_vehicles[i]
                car_bt_range = target_lane_vehicles[i].state[0]
            if target_lane_vehicles[i].state[0] >= x and target_lane_vehicles[i].state[0] <= carft_range:
                car_ft = target_lane_vehicles[i]
                carft_range = target_lane_vehicles[i].state[0]

        if lane_id == current_lane_id + direction_flag:
            acc_flag = 0 # indicates if ego vehicle is accelerating
        if acc_flag == 0:
            target_speed = self.goal.target_speed
        else:
            target_speed = lim_speed

        # CLF-CBF-QP formulation

        # lateral position CLF
        h_y = y - target_y
        V_y = h_y**2
        phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y
        phi1_y = [0, 2 * h_y * v * cos(psi)]

        # velocity CLF
        h_v = v - target_speed
        V_v = h_v**2
        phi0_v = alpha_v * V_v
        phi1_v = [2 * h_v * 1, 0]

        # yaw angle CLF
        h_yaw = psi
        V_yaw = h_yaw**2
        phi0_yaw = alpha_yaw * V_yaw
        phi1_yaw = [0, 2 * h_yaw * v * l_r]
        Aclf = [[*phi1_y, -1, 0, 0], \
                [*phi1_v, 0, -1, 0], \
                [*phi1_yaw, 0, 0, -1]]
        bclf = [[-phi0_y], [-phi0_v], [-phi0_yaw]]

        # Car_fc relevant CBFs
        if car_fc is None:
            Acbf1 = [0, 0, 0, 0, 0]
            bcbf1 = [0]
            Acbf2 = [0, 0, 0, 0, 0]
            bcbf2 = [0]
            h_CBF1 = []
            h_CBF2 = []
        else:
            x_carfc_rear = car_fc.state[0] - l_rc
            v_carfc = car_fc.state_log[3, -1]
            a_carfc = car_fc.input[0]
            # distance based CBF
            h_CBF1 = x_carfc_rear - x - self.param_sys.l_fc
            h1dot = v_carfc
            Lfh1 = -cos(psi) * v
            Lgh1 = [0, v * sin(psi)]
            Acbf1 = [0, -v * sin(psi), 0, 0, 0] # it was [-Lgh1, 0,0,0]
            bcbf1 = [ Lfh1 + gamma_1 * h_CBF1 + h1dot]
            # force based CBF
            if v > v_carfc:
                h_CBF2 = x_carfc_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v - 0.5 * (v_carfc - v) * (v_carfc - v) / self.goal.lim_acc
                h2dot = v_carfc - (0.5 / self.goal.lim_acc) * 2 * (v_carfc - v) * a_carfc
                Lfh2 = -cos(psi) * v
                Lgh2 = [(-(1 + safety_factor) + (v_carfc - v) / self.goal.lim_acc), v * sin(psi)]
                Acbf2 = [-Lgh2[0], -Lgh2[1], 0, 0, 0]
                bcbf2 = [Lfh2 + gamma_2 * h_CBF2 + h2dot]
            else:
                h_CBF2 = x_carfc_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v
                h2dot = v_carfc
                Lfh2 = -cos(psi) * v
                Lgh2 = [-(1 + safety_factor), v * sin(psi)]
                Acbf2 = [-Lgh2[0], -Lgh2[1], 0, 0, 0]
                bcbf2 = [Lfh2 + gamma_2 * h_CBF2 + h2dot]

        # Car_bt relevant CBFs
        if car_bt is None:
            Acbf3 = [0, 0, 0, 0, 0]
            bcbf3 = [0]
            Acbf4 = [0, 0, 0, 0, 0]
            bcbf4 = [0]
            h_CBF3 = []
            h_CBF4 = []
        else:
            x_carbt_front = car_bt.state[0] + l_fc
            v_carbt = car_bt.state_log[3, -1]
            a_carbt = car_bt.input[0]
            if car_bt.state[0] <= (x - l_fc - l_rc):
                # distance based CBF
                if v_carbt <= v:
                    h_CBF3 = x - l_rc - x_carbt_front
                    h3dot = -v_carbt
                    Lfh3 = cos(psi) * v
                    Lgh3 = [0, -v * sin(psi)]
                    Acbf3 = [0, -Lgh3[1], 0, 0, 0]
                    bcbf3 = [Lfh3 + gamma_3 * h_CBF3 + h3dot]
                else:
                    h_CBF3 = x - l_rc - x_carbt_front - 0.5 * (v_carbt - v) * (v_carbt - v) / self.goal.lim_acc
                    h3dot = -v_carbt - (1 / self.goal.lim_acc) * (v_carbt - v) * a_carbt
                    Lfh3 = cos(psi) * v
                    Lgh3 = [(v - v_carbt) / self.goal.lim_acc, -v * sin(psi)]
                    Acbf3 = [0, -Lgh3[1], 0, 0, 0]
                    bcbf3 = [Lfh3 + gamma_3 * h_CBF3 + h3dot]
            else:
                h_CBF3 = direction_flag * (car_bt.state[1] - y - width - safety_factor)
                Lfh3 = -direction_flag * v * sin(psi)
                Lgh3 = [0, -direction_flag * v * cos(psi)]
                Acbf3 = [0, -Lgh3[1], 0, 0, 0]
                bcbf3 = [Lfh3 + gamma_3 * h_CBF3]
            if v_carbt > v:
                h_CBF4 = -x_carbt_front + x - l_rc - (1 + safety_factor) * v_carbt - 0.5 * (v_carbt - v) * (v_carbt - v) / self.goal.lim_acc
                h4dot = -v_carbt - (1 + safety_factor) * a_carbt - (1 / self.goal.lim_acc) * (v_carbt - v) * a_carbt
                Lfh4 = cos(psi) * v
                Lgh4 = [(v - v_carbt) / self.goal.lim_acc, -v * sin(psi)]
                Acbf4 = [-Lgh4[0], -Lgh4[1], 0, 0, 0]
                bcbf4 = [Lfh4 + gamma_4 * h_CBF4 + h4dot]
            else:
                h_CBF4 = -x_carbt_front + x - l_rc - (1 + safety_factor) * v_carbt
                h4dot = -v_carbt - (1 + safety_factor) * car_bt.input[0]
                Lfh4 = cos(psi) * v
                Lgh4 = [0, -v * sin(psi)]
                Acbf4 = [0, -Lgh4[1], 0, 0, 0]
                bcbf4 = [Lfh4 + gamma_4 * h_CBF4 + h4dot]

        # Car_ft relevant CBFs
        if car_ft is None:
            Acbf5 = [0, 0, 0, 0, 0]
            bcbf5 = [0]
            Acbf6 = [0, 0, 0, 0, 0]
            bcbf6 = [0]
            h_CBF5 = []
            h_CBF6 = []
        else:
            x_carft_rear = car_ft.state[0] - l_rc
            v_carft = car_ft.state_log[3, -1]
            a_carft = car_ft.input_log[0, -1]
            if car_ft.state[0] - x > l_fc + l_rc:
                # distance based CBF
                if v_carft >= v:
                    h_CBF5 = x_carft_rear - x - self.param_sys.l_fc
                    h5dot = v_carft
                    Lfh5 = -cos(psi) * v
                    Lgh5 = [0, v * sin(psi)]
                    Acbf5 = [0, -Lgh5[1], 0, 0, 0]
                    bcbf5 = [Lfh5 + gamma_5 * h_CBF5 + h5dot]
                else:
                    h_CBF5 = x_carft_rear - x - self.param_sys.l_fc - 0.5 * (v_carft - v) * (v_carft - v) / self.goal.lim_acc
                    h5dot = v_carft - (0.5 / self.goal.lim_acc) * 2 * (v_carft - v) * a_carft
                    Lfh5 = -cos(psi) * v
                    Lgh5 = [((v_carft - v) / self.goal.lim_acc), v * sin(psi)]
                    Acbf5 = [-Lgh5[0], -Lgh5[1], 0, 0, 0]
                    bcbf5 = [Lfh5 + gamma_5 * h_CBF5 + h5dot]
            else:
                h_CBF5 = direction_flag * (car_ft.state[1] - y - width - safety_factor)
                Lfh5 = -direction_flag * v * sin(psi)
                Lgh5 = [0, -direction_flag * v * cos(psi)]
                Acbf5 = [0, -Lgh5[1], 0, 0, 0]
                bcbf5 = [Lfh5 + gamma_5 * h_CBF5]
            # force based CBF
            if v_carft < v:
                h_CBF6 = x_carft_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v - 0.5 * (v_carft - v) * (v_carft - v) / self.goal.lim_acc
                h6dot = v_carft - (0.5 / self.goal.lim_acc) * 2 * (v_carft - v) * a_carft
                Lfh6 = -cos(psi) * v
                Lgh6 = [(-(1 + safety_factor) + (v_carft - v) / self.goal.lim_acc), v * sin(psi)]
                Acbf6 = [-Lgh6[0], -Lgh6[1], 0, 0, 0]
                bcbf6 = [Lfh6 + gamma_6 * h_CBF6 + h6dot]
            else:
                h_CBF6 = x_carft_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v
                h6dot = v_carft
                Lfh6 = -cos(psi) * v
                Lgh6 = [-(1 + safety_factor), v * sin(psi)]
                Acbf6 = [-Lgh6[0], -Lgh6[1], 0, 0, 0]
                bcbf6 = [Lfh6 + gamma_6 * h_CBF6 + h6dot]
        
        # three states, 0: keep the current lane, 1: change to the left
        #  adjacent lane, -1: change to the right adjacent lane
        if direction_flag == 0: # ACC state
            # only car_fc relevant CBF is considered
            Acbf3 = [0, 0, 0, 0, 0]
            bcbf3 = [0]
            Acbf4 = [0, 0, 0, 0, 0]
            bcbf4 = [0]
            Acbf5 = [0, 0, 0, 0, 0]
            bcbf5 = [0]
            Acbf6 = [0, 0, 0, 0, 0]
            bcbf6 = [0]
        else: # the ego vehicle is changing its lane, L or R state
            if lane_id == current_lane_id + direction_flag:
                # the ego vehicle is already in its target lane, we
                # dont need car_fc and car_bt relevant CBFs
                Acbf1 = [0, 0, 0, 0, 0]
                bcbf1 = [0]
                Acbf2 = [0, 0, 0, 0, 0]
                bcbf2 = [0]
                Acbf3 = [0, 0, 0, 0, 0]
                bcbf3 = [0]
                Acbf4 = [0, 0, 0, 0, 0]
                bcbf4 = [0]

        # input constraint (accleration limit, slip anlge limit,
        # lateral acceleration limit, slip angle changing rate limit)
        A_u = [[1, 0, 0, 0, 0],\
                [-1, 0, 0, 0, 0],\
                [0, 1, 0, 0, 0],\
                [0, -1, 0, 0, 0],\
                [cos(psi + beta), 0, 0, 0, 0],\
                [-cos(psi + beta), 0, 0, 0, 0]]
        A_u0 = [[0, 1, 0, 0, 0], \
                [0, -1, 0, 0, 0]]
        b_u =  [[self.goal.lim_acc], \
                [self.goal.lim_acc], \
                [self.goal.lim_beta], \
                [self.goal.lim_beta], \
                [0.5 * 0.9 * 9.81], \
                [0.5 * 0.9 * 9.81]]
        b_u0 = [[beta + 1 * self.goal.lim_slip_rate * dt],\
                [-beta + 1 * self.goal.lim_slip_rate * dt]]
        #Constraint_A = [Aclf; Acbf2; Acbf4; Acbf6; A_u; A_u0];
        #Constraint_b = [bclf; bcbf2; bcbf4; bcbf6; b_u; b_u0];

        Constraint_A = Aclf + [Acbf2] + [Acbf4] + [Acbf6] + A_u + A_u0
        Constraint_b = bclf + [bcbf2] + [bcbf4] + [bcbf6] + b_u + b_u0
        
        Constraint_A = np.array(Constraint_A)
        Constraint_b = np.array(Constraint_b)
        '''
        H = [0.01, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0; ...
            0, 0, 15, 0, 0; ...
            0, 0, 0, 0.1, 0; ...
            0, 0, 0, 0, 400; ...
            ];
        F = [0; 0; 0; 0; 0];
        '''
        H = np.array(self.param_opt.H)
        F = np.array(self.param_opt.F)
        #options = optimoptions('quadprog', 'Display', 'off');
        #u = quadprog(H, F, Constraint_A, Constraint_b, [], [], [], [], [], options);
        # @@@ this part should be revised later
        u = quadprog(H, f, L=Constraint_A, k=Constraint_b)

        if u is None: # get optimal input
            # !!! this part must be modified
            acc_n = u[0]
            beta_n = u[1]
            input_n = [[acc_n], [beta_n]]
            e = 1
            if v >= target_speed + 0.5:
                acc_flag = 1

        else: # no optimal input, the ego vehicle enter BL or BR state, drives back to its current lane

            # predictive calculation, to determine if the ego vehicle can change the lane through acceleration
            if car_ft is not None:
                x_carft_rear = car_ft.state[0] - l_rc
                v_carft = car_ft.state[3]
                tmp3 = -x_carft_rear + x + (v * (lim_speed - v) / self.goal.lim_acc + (lim_speed - v)**2 / 2 / self.goal.lim_acc) - l_fc - v_carft * (lim_speed - v) / self.goal.lim_acc - 0.1 * safety_factor * v_carft
            if car_fc is not None:
                x_carfc_rear = car_fc.state[0] - l_rc
                v_carfc = car_fc.state[3]
                tmp1 = x_carfc_rear - x - (v * (lim_speed - v) / self.goal.lim_acc + (lim_speed - v)**2 / 2 / self.goal.lim_acc) - l_fc + v_carfc * (lim_speed - v) / self.goal.lim_acc - 0.1 * safety_factor * v
            if car_bt is not None:
                x_carbt_front = car_bt.state[0] + l_fc
                v_carbt = car_bt.state[3]
                tmp2 = x - x_carbt_front - l_rc + (v * (lim_speed - v) / self.goal.lim_acc + (lim_speed - v)**2 / 2 / self.goal.lim_acc) - v_carbt * (lim_speed - v) / self.goal.lim_acc - 0.1 * safety_factor * v_carbt
            if ((car_ft is not None) and (car_fc is not None) and (tmp3 >= 0 and tmp1 >= 0)) or \
                     ((car_ft is not None) and (car_fc is None) and (tmp3 >= 0)):
                track_speed = lim_speed
                acc_flag = 1
            elif ((car_bt is not None) and (car_ft is not None) and (car_fc is not None) and (tmp2 >= 0 and tmp1 >= 0 and tmp3 >= 0)) or \
                 ((car_bt is not None) and (car_ft is None) and (car_fc is not None) and (tmp2 >= 0 and tmp1 >= 0)) or \
                 ((car_bt is not None) and (car_ft is not None) and (car_fc is None) and (tmp2 >= 0 and tmp3 >= 0)) or \
                 ((car_bt is not None) and (car_ft is None) and (car_fc is None) and (tmp2 >= 0)):
                track_speed = lim_speed
                acc_flag = 1
            else:
                track_speed = target_speed

            # lateral position CLF
            h_y = y - (current_lane_id - 0.5) * self.straightlane.lane_width
            V_y = h_y**2
            phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y
            phi1_y = [0, 2 * h_y * v * cos(psi)]

            # velocity CLF
            h_v = v - track_speed
            V_v = h_v**2
            phi0_v = alpha_v * V_v
            phi1_v = [2 * h_v * 1, 0]

            # yaw angle CLF
            h_yaw = psi
            V_yaw = h_yaw**2
            phi0_yaw = alpha_yaw * V_yaw
            phi1_yaw = [0, 2 * h_yaw * v * l_r]
            Aclf = [[*phi1_y, -1, 0, 0],\
                    [*phi1_v, 0, -1, 0],\
                    [*phi1_yaw, 0, 0, -1]]
            bclf = [[-phi0_y], \
                    [-phi0_v], \
                    [-phi0_yaw]]
            if lane_id == current_lane_id: # the ego vehicle is already in its current lane,
                # FSM enters ACC state, no car_ft and car_bt relevant CBF
                Acbf3 = [0, 0, 0, 0, 0]
                bcbf3 = [0]
                Acbf4 = [0, 0, 0, 0, 0]
                bcbf4 = [0]
                Acbf5 = [0, 0, 0, 0, 0]
                bcbf5 = [0]
                Acbf6 = [0, 0, 0, 0, 0]
                bcbf6 = [0]

            #Constraint_A = [Aclf; Acbf2; Acbf3; Acbf5; A_u; A_u0];
            #Constraint_b = [bclf; bcbf2; bcbf3; bcbf5; b_u; b_u0];

            Constraint_A = Aclf + [Acbf2] + [Acbf3] + [Acbf5] + A_u + A_u0
            Constraint_b = bclf + [bcbf2] + [bcbf3] + [bcbf5] + b_u + b_u0
            Constraint_A = np.array(Constraint_A)
            Constraint_b = np.array(Constraint_b)

            '''
            H = [0.01, 0, 0, 0, 0; ...
                0, 0, 0, 0, 0; ...
                0, 0, 15, 0, 0; ...
                0, 0, 0, 0.1, 0; ...
                0, 0, 0, 0, 400; ...
                ];
            F = [0; 0; 0; 0; 0];
            '''
            H = np.array(self.param_opt.H)
            F = np.array(self.param_opt.F)

            #options = optimoptions('quadprog', 'Display', 'off');
            #u = quadprog(H, F, Constraint_A, Constraint_b, [], [], [], [], [], options);
            # @@@ this part should be revised later
            u = quadprog(H, f, L=Constraint_A, k=Constraint_b)

            if u is not None:
                acc_n = u[0]
                beta_n = u[1]
                input_n = [[acc_n], [beta_n]]
                e = 0
            else: # the ego vehicle can not meet safety-critical requirement
                raise RuntimeError('CLF_CBF Failed')
        
        return acc_flag, input_n, e

    def unpack_input(self, inp)
        acc, beta = inp
        return acc, beta
        
    def unpack_state(self, state)
        x, y, psi, v = state
        return x,y,psi,v
		