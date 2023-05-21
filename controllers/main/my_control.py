import numpy as np

class MyController:
    def __init__(self):

        # Drone parameters
        self.cruising_height = 0.4
        self.on_platform_height = 0.3
        self.cruising_speed = 0.35
        self.searching_speed = 0.25
        self.parking_speed = 0.15
        self.on_platform = True

        self.v_x = 0.0
        self.v_y = 0.0
        self.w_z = 0.0
        self.z = 0.0

        # States
        self.state, self.future_state = "START", "START"
        self.state_explore, self.future_state_explore = "OFFPLATFORM", "OFFPLATFORM"
        self.state_land_search, self.future_state_land_search = "SEARCH_LEFT", "SEARCH_LEFT"
        self.state_homing, self.future_state_homing = "OFFPLATFORM", "OFFPLATFORM"
        self.state_land, self.future_state_land = "RECENTERING", "RECENTERING"
        self.state_search_left, self.future_state_search_left = "SEARCH_LEFT", "SEARCH_LEFT"
        self.state_search_right, self.future_state_search_right = "SEARCH_RIGHT", "SEARCH_RIGHT"
        self.state_search_forward, self.future_state_search_forward = "SEARCH_FORWARD", "SEARCH_FORWARD"
        self.state_search_backward, self.future_state_search_backward = "SEARCH_BACKWARD", "SEARCH_BACKWARD"

        # Sensor data & parameters
        self.x_drone = None
        self.y_drone = None
        self.yaw_drone = None
        self.range_front = None
        self.range_back = None
        self.range_right = None
        self.range_left = None
        self.range_down = None

        self.old_range_down = None
        self.safe_range_obs = 0.4
        self.range_z_offset = 0.07
        self.x_offset_max = 0.06

        # Map parameters
        self.y_center_map = 1.5
        self.xlim_start_reg = 1.5
        self.xlim_land_reg = 3.5
        self.start_point = None
        self.stop_iter_count = 0
        self.min_stop_iter_count = 1200

        # Path planner parameters
        self.index_current_setpoint = 0
        self.setpoints = [
            [3.75, 2.90],
            [3.75, 0.10],
            [4.0, 0.10],
            [4.0, 2.90],
            [4.25, 2.90],
            [4.25, 0.10],
            [4.50, 0.10],
            [4.50, 2.90],
            [4.75, 2.90],
            [4.75, 0.10],
        ]

        # Search Zone parameters
        self.parking_zone = []
        self.parking_clearance = 0.20
        self.parking_min_vel = 0.05
        self.stop_height = 0.04



    def step_control(self, sensor_data):

        self.old_range_down = self.range_down 

        # Update sensor data
        self.x_drone = sensor_data["x_global"]
        self.y_drone = sensor_data["y_global"]
        self.yaw_drone = sensor_data["yaw"]
        self.range_front = sensor_data["range_front"]
        self.range_back = sensor_data["range_back"]
        self.range_right = sensor_data["range_right"]
        self.range_left = sensor_data["range_left"]
        self.range_down = sensor_data["range_down"]

        # Update states
        self.state = self.future_state
        self.state_explore = self.future_state_explore
        self.state_land_search = self.future_state_land_search
        self.state_homing = self.future_state_homing
        self.state_land = self.future_state_land
        self.state_search_left = self.future_state_search_left
        self.state_search_right = self.future_state_search_right
        self.state_search_forward = self.future_state_search_forward
        self.state_search_backward = self.future_state_search_backward

        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        print("x: ", self.x_drone, "y: ", self.y_drone)
        print("x_goal: ", x_goal, "y_goal: ", y_goal)
        print("state: ", self.state)
        print("state_land", self.state_land)

        # Update control commands
        match self.state:
            case "START":
                self.start()

            case "EXPLORE":
                self.explore()

            case "LAND_SEARCH":
                self.land_search()

            case "LAND":
                self.land()

            case "HOMING":
                self.homing()

        # Correct yaw
        if abs(self.yaw_drone) > 0.1:
            self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, (-8 * self.yaw_drone), self.z

        # Return control commands
        print("--------------------------------------------------")
        print("v_x: ", self.v_x, "v_y: ", self.v_y, "w_z: ", self.w_z, "z: ", self.z)
        print("range down: ", self.range_down)

        control_command = [self.v_x, self.v_y, self.w_z, self.z]
        return control_command

    ###########################################################################
    ############################# STATE FUNCTIONS #############################

    def start(self):
        # Condition for next state
        if self.range_down >= self.on_platform_height:
            # Save start point
            if self.start_point is None:
                self.start_point = [self.x_drone, self.y_drone]

            # Check if drone is in land region
            if self.x_drone >= self.xlim_land_reg:
                self.future_state = "HOMING"
                print("STATUS - homing")
            else:
                self.future_state = "EXPLORE"
                print("STATUS - exploration")
        else:
            # Take off
            self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, 0.0, 1.15 * self.on_platform_height
        return



    def explore(self):
        # Condition for next state
        if self.x_drone >= self.xlim_land_reg:
            self.future_state = "LAND_SEARCH"
            print("STATUS - land search")

        match self.state_explore:

            case "OFFPLATFORM":
                self.v_x, self.v_y, self.w_z, self.z = self.cruising_speed, 0.0, 0.0, self.on_platform_height

                if self.range_down - self.old_range_down > self.range_z_offset:
                    self.future_state_explore = "FORWARD"

            case "FORWARD":
                self.v_x, self.v_y, self.w_z, self.z = self.cruising_speed, 0.0, 0.0, self.cruising_height

                # Check if obstacle in front
                if self.range_front <= self.safe_range_obs:
                    if self.y_drone < self.y_center_map:
                        self.future_state_explore = "LEFT"
                    else:
                        self.future_state_explore = "RIGHT"

            case "LEFT":
                # Avoiding obstacle with left
                self.v_x, self.v_y, self.w_z, self.z = 0.0, self.cruising_speed, 0.0, self.cruising_height

                if self.range_front > self.safe_range_obs:
                    self.future_state_explore = "FORWARD"

            case "RIGHT":
                # Avoiding obstacle with right
                self.v_x, self.v_y, self.w_z, self.z = 0.0, -self.cruising_speed, 0.0, self.cruising_height

                if self.range_front > self.safe_range_obs:
                    self.future_state_explore = "FORWARD"
        return



    def land_search(self):
        match self.state_land_search:
            case "SEARCH_RIGHT":
                self.search_right()

            case "SEARCH_LEFT":
                self.search_left()

            case "SEARCH_FORWARD":
                self.search_forward()

            case "SEARCH_BACKWARD":
                self.search_backward()

        # Condition for next state
        if (self.old_range_down - self.range_down > self.range_z_offset):
            self.v_x, self.v_y, self.w_z, self.z = self.v_x, self.v_y, self.w_z, self.on_platform_height
            self.future_state = "LAND"
            self.parking_zone.append([self.x_drone, self.y_drone])
            print("STATUS - landing")
        return



    def land(self):
        match self.state_land:
            case "RECENTERING":
                if self.state_land_search == "SEARCH_LEFT":
                    self.v_x = self.parking_zone[0][0] - self.x_drone
                    self.v_y = (self.parking_zone[0][1] + self.parking_clearance) - self.y_drone
                elif self.state_land_search == "SEARCH_RIGHT":
                    self.v_x = self.parking_zone[0][0] - self.x_drone
                    self.v_y = (self.parking_zone[0][1] - self.parking_clearance) - self.y_drone
                elif self.state_land_search == "SEARCH_FORWARD":
                    self.v_x = (self.parking_zone[0][0] - self.parking_clearance) - self.x_drone
                    self.v_y = self.parking_zone[0][1] - self.y_drone
                else:
                    self.v_x = (self.parking_zone[0][0] + self.parking_clearance) - self.x_drone
                    self.v_y = self.parking_zone[0][1] - self.y_drone

                if abs(self.v_x) < self.parking_min_vel and abs(self.v_y) < self.parking_min_vel:
                    self.future_state_land = "P3_SEARCH"

            case "P3_SEARCH":
                # Drone takes predefined direction until it finds P2, the last point needed to find the
                # landing zone center
                if self.state_land_search == "SEARCH_LEFT" or self.state_land_search == "SEARCH_RIGHT":
                    self.v_x, self.v_y, self.w_z, self.z = self.parking_speed, 0.0, 0.0, self.on_platform_height
                elif self.state_land_search == "SEARCH_FORWARD" or self.state_land_search == "SEARCH_BACKWARD":
                    self.v_x, self.v_y, self.w_z, self.z = 0.0, self.parking_speed, 0.0, self.on_platform_height

                if (self.range_down - self.old_range_down > self.range_z_offset):
                    self.v_x, self.v_y, self.w_z, self.z = -self.v_x, -self.v_y, 0.0, self.cruising_height
                    self.parking_zone.append([self.x_drone, self.y_drone])
                    self.future_state_land = "PARKING"

            case "PARKING":
                # Drone is recentered between P1 and P2 in one direction and P3 in the other direction,
                # depending on its previous state_land_search
                if self.state_land_search == "SEARCH_LEFT":
                    self.v_x = (self.parking_zone[1][0] - self.parking_clearance) - self.x_drone
                    self.v_y = (self.parking_zone[0][1] + self.parking_clearance) - self.y_drone
                elif self.state_land_search == "SEARCH_RIGHT":
                    self.v_x = (self.parking_zone[1][0] - self.parking_clearance) - self.x_drone
                    self.v_y = (self.parking_zone[0][1] - self.parking_clearance) - self.y_drone
                elif self.state_land_search == "SEARCH_FORWARD":
                    self.v_x = (self.parking_zone[0][0] - self.parking_clearance) - self.x_drone
                    self.v_y = (self.parking_zone[0][1] + self.parking_clearance) - self.y_drone
                else:
                    self.v_x = (self.parking_zone[0][0] + self.parking_clearance) - self.x_drone
                    self.v_y = (self.parking_zone[0][1] + self.parking_clearance) - self.y_drone
                
                if (self.old_range_down - self.range_down > self.range_z_offset):
                    self.z = self.on_platform_height

                if abs(self.v_x) < self.parking_min_vel and abs(self.v_y) < self.parking_min_vel:
                    self.future_state_land = "LANDING"

            case "LANDING":
                # Drone is landing on the platform
                self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, 0.0, 0.98 * self.z
                if self.z < self.stop_height:
                    self.future_state_land = "STOP"

            case "STOP":
                # Drone is stopped on the platform
                self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, 0.0, 0.0

                # Waits for a number of iterations before going back to the start state
                if self.x_drone > self.xlim_start_reg:
                    self.stop_iter_count += 1

                    if self.stop_iter_count > self.min_stop_iter_count:
                        self.stop_iter_count = 0
                        self.future_state = "START"
                        self.future_state_land = "RECENTERING"
                        self.parking_zone = []
                        print("STATUS - start")
        return



    def homing(self):
        # Condition for next state
        if self.x_drone <= self.xlim_start_reg:
            self.index_current_setpoint = 0
            self.setpoints = [self.start_point]
            self.future_state = "LAND_SEARCH"
            print("STATUS - land search")

        match self.state_homing:
            
            case "OFFPLATFORM":
                self.v_x, self.v_y, self.w_z, self.z = -self.cruising_speed, 0.0, 0.0, self.on_platform_height

                if self.range_down - self.old_range_down > self.range_z_offset:
                    self.future_state_homing = "BACKWARD"

            case "BACKWARD":
                self.v_x, self.v_y, self.w_z, self.z = -self.cruising_speed, 0.0, 0.0, self.cruising_height

                # Check if obstacle in front
                if self.range_back <= self.safe_range_obs:
                    if self.y_drone < self.start_point[1]:
                        self.future_state_homing = "LEFT"
                    else:
                        self.future_state_homing = "RIGHT"

            case "LEFT":
                # Avoiding obstacle with left
                self.v_x, self.v_y, self.w_z, self.z = 0.0, self.cruising_speed, 0.0, self.cruising_height

                if self.range_back > self.safe_range_obs:
                    self.future_state_homing = "BACKWARD"

            case "RIGHT":
                # Avoiding obstacle with right
                self.v_x, self.v_y, self.w_z, self.z = 0.0, -self.cruising_speed, 0.0, self.cruising_height

                if self.range_back > self.safe_range_obs:
                    self.future_state_homing = "BACKWARD"
        return

    ###########################################################################
    ############################# OTHER FUNCTIONS #############################

    def path_planning(self):
        if self.index_current_setpoint == len(self.setpoints):
            self.index_current_setpoint = 0
            self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, 0.0, self.cruising_height

        # Get the goal position and drone position
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        distance_drone_to_goal = np.linalg.norm([x_goal - self.x_drone, y_goal - self.y_drone])

        # When the drone reaches the goal setpoint, e.g., distance < 0.1m
        if distance_drone_to_goal < 0.05 or (self.x_drone > self.xlim_start_reg and distance_drone_to_goal < 0.1):
            # Select the next setpoint as the goal position
            self.index_current_setpoint += 1
            # Hover at the final setpoint
            if self.index_current_setpoint == len(self.setpoints):
                self.index_current_setpoint = 0
                self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, 0.0, self.cruising_height
                return

        # Calculate next state based on current goal setpoint
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]

        if abs(y_goal - self.y_drone) > abs(x_goal - self.x_drone):
            if y_goal > self.y_drone:
                self.future_state_land_search = "SEARCH_LEFT"
            else:
                self.future_state_land_search = "SEARCH_RIGHT"
        else:
            if x_goal > self.x_drone:
                self.future_state_land_search = "SEARCH_FORWARD"
            else:
                self.future_state_land_search = "SEARCH_BACKWARD"

        self.v_x, self.v_y, self.w_z, self.z = 0.0, 0.0, 0.0, self.cruising_height
        return



    def search_forward(self):
        x_goal, _ = self.setpoints[self.index_current_setpoint]

        match self.state_search_forward:
            case "SEARCH_FORWARD":
                self.v_x, self.v_y, self.w_z, self.z = self.searching_speed, 0.0, 0.0, self.cruising_height
                if self.range_front <= self.safe_range_obs:
                    self.future_state_search_forward = "HANDLE_OBSTACLE"

            case "HANDLE_OBSTACLE":
                if (self.y_drone < self.y_center_map and self.x_drone > self.xlim_start_reg) or (
                    self.y_drone < self.start_point[1] and self.x_drone < self.xlim_start_reg
                ):
                    self.v_x, self.v_y, self.w_z, self.z = 0.0, self.searching_speed, 0.0, self.cruising_height
                else:
                    self.v_x, self.v_y, self.w_z, self.z = 0.0, -self.searching_speed, 0.0, self.cruising_height
                if self.range_front >= self.safe_range_obs:
                    self.future_state_search_forward = "SEARCH_FORWARD"

        if x_goal <= self.x_drone:
            self.future_state_search_forward = "SEARCH_FORWARD"
            self.path_planning()
        return



    def search_backward(self):
        x_goal, _ = self.setpoints[self.index_current_setpoint]

        match self.state_search_backward:
            case "SEARCH_BACKWARD":
                self.v_x, self.v_y, self.w_z, self.z = -self.searching_speed, 0.0, 0.0, self.cruising_height
                if self.range_back <= self.safe_range_obs:
                    self.future_state_search_backward = "HANDLE_OBSTACLE"

            case "HANDLE_OBSTACLE":
                if (self.y_drone < self.y_center_map and self.x_drone > self.xlim_land_reg) or (
                    self.y_drone < self.start_point[1] and self.x_drone < self.xlim_start_reg
                ):
                    self.v_x, self.v_y, self.w_z, self.z = 0.0, self.searching_speed, 0.0, self.cruising_height
                else:
                    self.v_x, self.v_y, self.w_z, self.z = 0.0, -self.searching_speed, 0.0, self.cruising_height
                if self.range_back > self.safe_range_obs:
                    self.future_state_search_backward = "SEARCH_BACKWARD"

        if x_goal >= self.x_drone:
            self.future_state_search_backward = "SEARCH_BACKWARD"
            self.path_planning()
        return



    def search_left(self):
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]

        match self.state_search_left:
            case "SEARCH_LEFT":
                self.v_x, self.v_y, self.w_z, self.z = 0.0, self.searching_speed, 0.0, self.cruising_height
                if self.range_left <= self.safe_range_obs:
                    self.future_state_search_left = "HANDLE_OBSTACLE"
                    if self.x_drone > self.xlim_start_reg:
                        self.v_x, self.v_y, self.w_z, self.z = self.searching_speed, 0.0, 0.0, self.cruising_height
                    else:
                        self.v_x, self.v_y, self.w_z, self.z = -self.searching_speed, 0.0, 0.0, self.cruising_height
                elif (
                    abs(self.x_drone - x_goal) > self.x_offset_max
                    and self.range_back > self.safe_range_obs
                    and self.range_front > self.safe_range_obs
                ):
                    self.future_state_search_left = "X_RESET"

            case "HANDLE_OBSTACLE":
                if self.x_drone > self.xlim_start_reg:
                    self.v_x, self.v_y, self.w_z, self.z = self.searching_speed, 0.0, 0.0, self.cruising_height
                else:
                    self.v_x, self.v_y, self.w_z, self.z = -self.searching_speed, 0.0, 0.0, self.cruising_height
                if self.range_left > self.safe_range_obs:
                    self.future_state_search_left = "AVOID_OBSTACLE"
                    self.waiting_obstacle = True

            case "AVOID_OBSTACLE":
                self.v_x, self.v_y, self.w_z, self.z = 0.0, self.searching_speed, 0.0, self.cruising_height
                if self.range_back >= self.safe_range_obs and not self.waiting_obstacle:
                    self.future_state_search_left = "X_RESET"
                elif self.range_back <= self.safe_range_obs and self.waiting_obstacle:
                    self.waiting_obstacle = False

            case "X_RESET":
                self.reset_x()
                if abs(self.x_drone - x_goal) < 0.01:
                    self.future_state_search_left = "SEARCH_LEFT"

        if y_goal <= self.y_drone:
            self.waiting_obstacle = False
            self.future_state_search_left = "SEARCH_LEFT"
            self.path_planning()
        return



    def search_right(self):
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]

        match self.state_search_right:
            case "SEARCH_RIGHT":
                self.v_x, self.v_y, self.w_z, self.z = 0.0, -self.searching_speed, 0.0, self.cruising_height
                if self.range_right <= self.safe_range_obs:
                    self.future_state_search_right = "HANDLE_OBSTACLE"
                    if self.x_drone > self.xlim_start_reg:
                        self.v_x, self.v_y, self.w_z, self.z = self.searching_speed, 0.0, 0.0, self.cruising_height
                    else:
                        self.v_x, self.v_y, self.w_z, self.z = -self.searching_speed, 0.0, 0.0, self.cruising_height
                elif (
                    abs(self.x_drone - x_goal) > self.x_offset_max
                    and self.range_back > self.safe_range_obs
                    and self.range_front > self.safe_range_obs
                ):
                    self.future_state_search_right = "X_RESET"

            case "HANDLE_OBSTACLE":
                if self.x_drone > self.xlim_start_reg:
                    self.v_x, self.v_y, self.w_z, self.z = self.searching_speed, 0.0, 0.0, self.cruising_height
                else:
                    self.v_x, self.v_y, self.w_z, self.z = -self.searching_speed, 0.0, 0.0, self.cruising_height
                if self.range_right > self.safe_range_obs:
                    self.future_state_search_right = "AVOID_OBSTACLE"
                    self.waiting_obstacle = True

            case "AVOID_OBSTACLE":
                self.v_x, self.v_y, self.w_z, self.z = 0.0, -self.searching_speed, 0.0, self.cruising_height
                if self.range_back > self.safe_range_obs and not self.waiting_obstacle:
                    self.future_state_search_right = "X_RESET"
                elif self.range_back <= self.safe_range_obs and self.waiting_obstacle:
                    self.waiting_obstacle = False

            case "X_RESET":
                self.reset_x()
                if abs(self.x_drone - x_goal) < 0.01:
                    self.future_state_search_right = "SEARCH_RIGHT"

        if y_goal >= self.y_drone:
            self.future_state_search_right = "SEARCH_RIGHT"
            self.waiting_obstacle = False
            self.path_planning()
        return



    def reset_x(self):
        x_goal, _ = self.setpoints[self.index_current_setpoint]

        if self.x_drone > x_goal:
            self.v_x, self.v_y, self.w_z, self.z = -(abs(self.x_drone - x_goal) + 0.05), 0.0, 0.0, self.cruising_height
        else:
            self.v_x, self.v_y, self.w_z, self.z = (abs(self.x_drone - x_goal) + 0.05), 0.0, 0.0, self.cruising_height
        return