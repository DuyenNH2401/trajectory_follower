import numpy as np
from controller import Supervisor

import config


class EPuckController:
    def __init__(self):
        # Initialize Robot
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        # State variables
        self.last_seen = None
        self.stop_counter = 0
        self.step_count = 0
        self.is_finished = False

        # Odometry state
        self.xw = config.INITIAL_X
        self.yw = config.INITIAL_Y
        self.theta = config.INITIAL_THETA

        # Mapping
        self.map = np.zeros((config.MAP_RESOLUTION, config.MAP_RESOLUTION))

        # Waypoints
        self.waypoints = config.WP

        # Initialize devices
        self._init_devices()

    def _init_devices(self):
        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Ground sensors
        self.gs = [self.robot.getDevice(f"gs{i}") for i in range(3)]
        for device in self.gs:
            device.enable(self.timestep)

        # Lidar
        self.lidar = self.robot.getDevice("LDS-01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        # Display
        self.display = self.robot.getDevice("display")

        # GPS
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        # Compass
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)

        # Marker
        self.marker = self.robot.getFromDef("marker").getField("translation")

    def step_simulation(self) -> bool:
        """
        Returns:
            bool: True if the simulation should continue, False if it should stop
        """
        if self.robot.step(self.timestep) == -1:
            return False

        self.step_count += 1
        return not self.is_finished

    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """This function should set the speeds of the left and right motors of the robot. It should take into account the maximum speed limits and ensure that the speeds are within the allowed range.

        Args:
            left_speed (float): Desired speed for the left motor
            right_speed (float): Desired speed for the right motor
        """
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def follow_line(self):
        """This function should implement the main control loop for line following. It should read the ground sensors, calculate the motor speeds using the line_following function, and set the motor speeds accordingly. It should also update the odometry state for use in mapping and error calculation."""
        # Read ground sensors
        L = self.gs[0].getValue() < config.THRESHOLD
        C = self.gs[1].getValue() < config.THRESHOLD
        R = self.gs[2].getValue() < config.THRESHOLD

        # Calculate wheel velocities
        phil, phir = self._line_following(L, C, R)

        # Set velocities
        self.set_motor_speeds(phil, phir)

        if self.is_finished:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            error = np.sqrt(self.xw**2 + self.yw**2)
            print(f"Final Error (Distance to 0,0): {error:.4f} meters")

    def _line_following(self, L: bool, C: bool, R: bool) -> (float, float):
        """This function should implement the line following logic based on the readings of the ground sensors (L, C, R).

        rgs:
            L: Left ground sensor reading (True if on line, False if on ground)
            C: Center ground sensor reading (True if on line, False if on ground)
            R: Right ground sensor reading (True if on line, False if on ground)

        Returns:
            phil, phir: Left and right motor speeds
        """
        phil, phir = config.BASE_SPEED, config.BASE_SPEED

        if L and C and R:
            if self.step_count > 50:
                self.stop_counter += 1
                if self.stop_counter >= 10:
                    phil, phir = 0.0, 0.0
                    self.is_finished = True
                    print("!STOP!")
        else:
            self.stop_counter = 0

            # Forward
            if not L and C and not R:
                phil, phir = config.BASE_SPEED, config.BASE_SPEED
                self.last_seen = None

            # Hard turn left
            elif L and not C and not R:
                phil, phir = -0.1 * config.BASE_SPEED, config.BASE_SPEED
                self.last_seen = "left"

            # Hard turn right
            elif not L and not C and R:
                phil, phir = config.BASE_SPEED, -0.1 * config.BASE_SPEED
                self.last_seen = "right"

            # Soft turn left
            elif L and C and not R:
                phil, phir = 0.1 * config.BASE_SPEED, config.BASE_SPEED
                self.last_seen = "left"

            # Soft turn right
            elif not L and C and R:
                phil, phir = config.BASE_SPEED, 0.1 * config.BASE_SPEED
                self.last_seen = "right"

            # Lost line
            elif not L and not C and not R:
                if self.last_seen == "left":
                    phil, phir = -0.8 * config.BASE_SPEED, 0.8 * config.BASE_SPEED
                elif self.last_seen == "right":
                    phil, phir = 0.8 * config.BASE_SPEED, -0.8 * config.BASE_SPEED

        return phil, phir

    def update_odometry(self):
        """This function should update the robot's odometry state (xw, yw, omegaz) based on the actual wheel velocities (actual_phil, actual_phir) and the time step. This will be used for mapping and error calculation."""

        # delta_t = self.timestep / 1000.0
        # delta_x = (
        #     config.WHEEL_RADIUS * (self.actual_phil + self.actual_phir) / 2.0 * delta_t
        # )
        # delta_omega = (
        #     config.WHEEL_RADIUS
        #     * (self.actual_phir - self.actual_phil)
        #     / config.WHEEL_AXLE_LENGTH
        #     * delta_t
        # )

        # self.xw += delta_x * np.cos(self.theta)
        # self.yw += delta_x * np.sin(self.theta)
        # self.theta += delta_omega

        self.xw = self.gps.getValues()[0]
        self.yw = self.gps.getValues()[1]
        self.theta = np.arctan2(
            self.compass.getValues()[0], self.compass.getValues()[1]
        )

    def trajectory_following(self, index: int):
        """This function should implement the main control loop for trajectory following. It should compute the error between the robot's current position and the target waypoint, and adjust the motor speeds accordingly to minimize this error. It should also update the odometry state for use in mapping and error calculation."""

        self._place_marker(index)
        rho, alpha = self._computing_error(index)

        if rho < 0.1:
            index += 1
            if index >= len(self.waypoints):
                print("Trajectory completed!")
                self.is_finished = True

        phil = -alpha * config.P1 + rho * config.P2
        phir = alpha * config.P1 + rho * config.P2

        return rho

    def _computing_error(self, index: int) -> (float, float):
        """This function should compute the error between the robot's current position (xw, yw) and the target waypoint specified by the index. The error can be calculated as the Euclidean distance between the robot's position and the waypoint. This error will be used to determine how well the robot is following the desired trajectory.

        Args:
            index (int): The index of the target waypoint

        Returns:
            tuple[float, float]: The error in terms of distance and angle
        """
        self._place_marker(index)

        rho = np.sqrt(
            (self.xw - self.waypoints[index][0]) ** 2
            + (self.yw - self.waypoints[index][1]) ** 2
        )

        alpha = (
            np.arctan2(
                self.waypoints[index][1] - self.yw,
                self.waypoints[index][0] - self.xw,
            )
            - self.theta
        )

        return rho, alpha

    def _place_marker(self, index):
        self.marker.setSFVec3f([*self.waypoints[index], 0.0])

    def lidar2world_coordinate(self) -> (np.ndarray, np.ndarray):
        """This function should convert the LIDAR readings from the robot's local coordinate frame to the world coordinate frame using the current odometry state (xw, yw, omegaz). It should return the LIDAR points in both the robot's local frame and the world frame for use in mapping and error calculation.

        Returns:
            X_r: LIDAR points in the robot's local coordinate frame (shape: 3 x N)
            D: LIDAR points in the world coordinate frame (shape: 3 x N)
        """

        ranges = np.array(self.lidar.getRangeImage())

        ranges[ranges == np.inf] = 100

        num_points = ranges.shape[0]
        angles = np.linspace(np.pi / 4, -np.pi / 4, num_points)

        w_T_r = np.array(
            [
                [np.cos(self.theta), -np.sin(self.theta), self.xw],
                [np.sin(self.theta), np.cos(self.theta), self.yw],
                [0, 0, 1],
            ]
        )

        X_r = np.array(
            [ranges * np.cos(angles), ranges * np.sin(angles), np.ones(len(angles))]
        )

        D = w_T_r @ X_r

        return X_r, D

    def world2map(self, xw: np.ndarray, yw: np.ndarray) -> (np.ndarray, np.ndarray):
        """
        This function should convert world coordinates (xw, yw) to map pixel coordinates (x_pixel, y_pixel) based on the defined map resolution and the world coordinate limits. This will be used for mapping the LIDAR points onto the occupancy grid.

        Args:
            xw (np.ndarray): World x coordinates
            yw (np.ndarray): World y coordinates
        Returns:
            x_pixel (np.ndarray): Map pixel x coordinates
            y_pixel (np.ndarray): Map pixel y coordinates
        """

        X_max, X_min = 0.305 + 0.5, 0.305 - 0.5
        Y_max, Y_min = 0.25 + 0.5, 0.25 - 0.5

        x_val = (xw - X_min) * config.MAP_RESOLUTION / (X_max - X_min)
        y_val = (yw - Y_min) * config.MAP_RESOLUTION / (Y_max - Y_min)

        if isinstance(xw, np.ndarray):
            x_pixel = np.clip(x_val.astype(int), 0, config.MAP_RESOLUTION - 1)
            y_pixel = np.clip(y_val.astype(int), 0, config.MAP_RESOLUTION - 1)

        else:
            x_pixel = max(0, min(config.MAP_RESOLUTION - 1, int(x_val)))
            y_pixel = max(0, min(config.MAP_RESOLUTION - 1, int(y_val)))

        return x_pixel, y_pixel

    def probabilistic_mapping(self, world_point: np.ndarray) -> None:
        """
        This function should update the occupancy grid map based on the LIDAR points in the world coordinate frame. It should use a probabilistic approach to increase the probability of occupancy for cells corresponding to detected obstacles and decrease the probability for cells corresponding to free space. The map should be updated incrementally as new LIDAR data is received.

        Args:
            world_point (np.ndarray): LIDAR points in the world coordinate frame (shape: 3 x N)
        """
        map_point = self.world2map(world_point[0], world_point[1])
        np.add.at(self.map, (map_point[0], map_point[1]), 0.01)
        self.map = np.clip(self.map, 0.0, 1.0)
