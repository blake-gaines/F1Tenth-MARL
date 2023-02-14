class EgoControllerGoal:
    def __init__(self, initial_lane_id, direction_flag, \
            desired_speed, lanes, lim_slip_angle, lim_acc, \
            lim_slip_rate, safety_factor, scenario):
        self.target_y = ((initial_lane_id + direction_flag) - 1) * lanes.lane_width + 0.5 * lanes.lane_width
        self.target_speed = desired_speed
        self.lim_beta = lim_slip_angle
        self.lim_acc = lim_acc
        self.lim_slip_rate = lim_slip_rate
        self.safety_factor = safety_factor
        self.scenario = scenario
        self.lim_yaw = None
        if self.scenario == 1:
            self.lim_speed = 33.33 # equal to 120 km/h
        elif self.scenario == 2:
            self.lim_speed = 16.67 # equal to 60 km/h
            