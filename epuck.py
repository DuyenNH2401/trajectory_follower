import config
import numpy as np
from controller import Supervisor


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
        self.omegaz = config.INITIAL_OMEGA
        self.actual_phil = 0.0
        self.actual_phir = 0.0

        # Mapping
        self.map = np.zeros((config.MAP_RESOLUTION, config.MAP_RESOLUTION))

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

    def line_following(self, L, C, R):
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

    def step_simulation(self) -> bool:
        if self.robot.step(self.timestep) == -1:
            return False

        self.step_count += 1
        return not self.is_finished

    def follow_line(self):
        # Read ground sensors
        L = self.gs[0].getValue() < config.THRESHOLD
        C = self.gs[1].getValue() < config.THRESHOLD
        R = self.gs[2].getValue() < config.THRESHOLD

        # Calculate wheel velocities
        phil, phir = self.line_following(L, C, R)

        # Set velocities
        self.left_motor.setVelocity(phil)
        self.right_motor.setVelocity(phir)

        # Update velocities for odometry calculation in the next step
        self.actual_phil = phil
        self.actual_phir = phir

        if self.is_finished:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            error = np.sqrt(self.xw**2 + self.yw**2)
            print(f"Final Error (Distance to 0,0): {error:.4f} meters")

    def update_odometry(self):
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

        # self.xw += delta_x * np.cos(self.omegaz)
        # self.yw += delta_x * np.sin(self.omegaz)
        # self.omegaz += delta_omega

        self.xw = self.gps.getValues()[0]
        self.yw = self.gps.getValues()[1]
        self.omegaz = np.arctan2(
            self.compass.getValues()[0], self.compass.getValues()[1]
        )

    def lidar2world_coordinate(self):
        ranges = np.array(self.lidar.getRangeImage())

        ranges[ranges == np.inf] = 100

        num_points = ranges.shape[0]
        angles = np.linspace(np.pi / 4, -np.pi / 4, num_points)

        w_T_r = np.array(
            [
                [np.cos(self.omegaz), -np.sin(self.omegaz), self.xw],
                [np.sin(self.omegaz), np.cos(self.omegaz), self.yw],
                [0, 0, 1],
            ]
        )

        X_r = np.array(
            [ranges * np.cos(angles), ranges * np.sin(angles), np.ones(len(angles))]
        )

        D = w_T_r @ X_r

        return X_r, D

    def world2map(self, xw, yw):
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

    def probabilistic_mapping(self, world_point):
        map_point = self.world2map(world_point[0], world_point[1])
        np.add.at(self.map, (map_point[0], map_point[1]), 0.01)
        self.map = np.clip(self.map, 0.0, 1.0)
