from math import atan, tan, cos, sin

class Vehicle:

    def __init__(self, dynamics_flag, veh_param, state_initial, \
            initial_input, controller, initial_lane_id, dt, \
            lanes, direction_flag, acc_flag, scenario):
        self.dynamics_flag = dynamics_flag
        self.param = veh_param
        self.controller = controller
        self.state = state_initial
        # at first time step, the current state of the vehicle is same to initial state
        self.input = initial_input
        # at first time step, the current input is same as initial input
        self.state_log = [state_initial]
        # the first element of states' history
        self.input_log = [initial_input]
        # the first element of inputs' history
        self.initial_lane_id = initial_lane_id
        self.lane_id = initial_lane_id
        self.dt = dt
        self.lanes = lanes
        self.direction_flag = direction_flag
        self.acc_flag = acc_flag
        self.scenario = scenario
        self.other_log = [[initial_lane_id], [1]]

    def update(self):
        # during the simulation, using this method at each time step to update the vehicle
        if self.dynamics_flag == 0: # when the vehile is a normal vehilce
            self.normal_car_update()
        elif self.dynamics_flag == 1: # using the controller to calculate the input and update the state
            self.ego_vehicle_update()
        elif self.dynamics_flag == 2:
            self.lane_change_car_update()

    def plot_vehicle(self):
        if self.dynamics_flag == 0 or self.dynamics_flag == 2:
            self.draw_vehicle(self.state, (self.param.l_fc + self.param.l_rc), \
                    self.param.width, self.dynamics_flag)
        else:
            self.draw_vehicle(self.state, (self.param.l_fc + self.param.l_rc), \
                    self.param.width, self.dynamics_flag)

    def normal_car_update(self):
        self.get_lane_id(self.state)
        speed = self.state[3]
        acceleration = self.input[0]
        self.state[3] = self.state[3] + acceleration * self.dt # new speed
        # speed limit according to different scenarios
        if self.scenario == 1:
            ulim = 33.33
            llim = 23
        elif self.scenario == 2:
            ulim = 16.67
            llim = 12
        if self.state[3] >= ulim:
            self.state[3] = ulim
        elif self.state[3] <= llim:
            self.state[3] = llim
        dx = speed * self.dt + 0.5 * acceleration * self.dt**2 #dx=v*dt+0.5*a*dt^2
        self.state = [self.state[0] + dx] + self.state[1:4] # new state of normal cars
        self.state_log.append(self.state) # update the state hisotory
        self.input_log.append(self.input) # update the input history
        self.other_log = [self.other_log[0]+[self.lane_id], self.other_log[1]+[0]]

    def ego_vehicle_update(self):
        self.get_lane_id(self.state)
        self.acc_flag, u, e = self.controller.get_optimal_input(self.state, self.input, \
                                    self.lane_id, self.input_log, self.initial_lane_id, \
                                    self.direction_flag, self.acc_flag)
        # calculate the optimal input of the vehicle
        dx = self.Bicycle(self.state[:3], [self.state[3] + self.dt * u[0], u[1]]) # calculate dX through nonlinear model
        self.state = self.state + self.dt .* [dx; u(1)] #update the state of the vehicle
        self.input = u # update the input
        self.state_log.append(self.state) # update the state history
        self.input_log.append(self.input) # update the input history
        self.other_log = [self.other_log[0]+[self.lane_id], self.other_log[1]+[e]]

    def lane_change_car_update(self):
        self.get_lane_id(self.state)
        n, u, e = self.controller.get_optimal_input(self.state, self.input, \
                        self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag)
        # calculate the optimal input of the vehicle
        dx = self.Bicycle(self.state[:3], [self.state[3] + self.dt * u[0], u[1]]) # calculate dX through nonlinear model
        self.state = [self.state[0]+self.dt*dx, self.state[1]+self.dt*u[0]]
        self.input = u # update the input
        self.state_log.append(self.state) # update the state history
        self.input_log.append(self.input) # update the input history
        self.other_log = [self.other_log[0]+[self.lane_id], self.other_log[1]+[e]]


    # state = [x, y]
    def get_lane_id(self, state):
        if state[1] <= self.lanes.lane_width - 0.5 * self.param.width:
            self.lane_id = 1
        elif state[1] <= self.lanes.lane_width + 0.5 * self.param.width:
            self.lane_id = 1.5
        elif state[1] <= 2 * self.lanes.lane_width - 0.5 * self.param.width:
            self.lane_id = 2
        elif state[1] <= 2 * self.lanes.lane_width + 0.5 * self.param.width:
            self.lane_id = 2.5
        elif state[1] <= 3 * self.lanes.lane_width - 0.5 * self.param.width:
            self.lane_id = 3
        elif state[1] <= 3 * self.lanes.lane_width + 0.5 * self.param.width:
            self.lane_id = 3.5
        elif state[1] <= 4 * self.lanes.lane_width - 0.5 * self.param.width:
            self.lane_id = 4

    # state: x, y, phi
    # input_v: speed, beta
    def Bicycle(self, state, input_v):
        l_f = self.param.l_f
        l_r = self.param.l_r
        l = l_f + l_r
        x, y, phi = state
        v, beta = input_v
        delta_f = atan(l*tan(beta)/l_r) # calculate front steering angle from slip angle
        xdot = v * cos(phi+beta) # velocity in x direction
        ydot = v * sin(phi+beta) # velocity in y direction
        phidot = v * sin(beta) / l_r # yaw rate
        dX = [xdot, ydot, phidot]
        return dX

    def draw_vehicle(self, state, L, H, dynamics_flag):
        '''
        theta = state(3);
        center1 = state(1);
        center2 = state(2);
        R = ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
        X = ([-L / 2, L / 2, L / 2, -L / 2]);
        Y = ([-H / 2, -H / 2, H / 2, H / 2]);
        for i = 1:4
            T(:, i) = R * [X(i); Y(i)];
        end
        x_lower_left = center1 + T(1, 1);
        x_lower_right = center1 + T(1, 2);
        x_upper_right = center1 + T(1, 3);
        x_upper_left = center1 + T(1, 4);
        y_lower_left = center2 + T(2, 1);
        y_lower_right = center2 + T(2, 2);
        y_upper_right = center2 + T(2, 3);
        y_upper_left = center2 + T(2, 4);
        x_coor = [x_lower_left, x_lower_right, x_upper_right, x_upper_left];
        y_coor = [y_lower_left, y_lower_right, y_upper_right, y_upper_left];
        if dynamics_flag == 0 | dynamics_flag == 2
            color = [0, 0, 1]; % blue
        else
            color = [1, 0, 0]; % red
        end
        patch('Vertices', [x_coor; y_coor]', 'Faces', [1, 2, 3, 4], 'Edgecolor', color, 'Facecolor', color, 'Linewidth', 1.2);
        axis equal;
        '''
        pass
