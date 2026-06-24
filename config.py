# Robot Speed Settings
MAX_SPEED = 6.28  # Maximum speed of e-puck motors (rad/s)
BASE_SPEED = MAX_SPEED * 0.5  # Base speed for line following

# Sensor Settings
THRESHOLD = 350  # Light threshold for ground sensors (black line vs white ground)

# Odometry Settings
INITIAL_X = 0.0
INITIAL_Y = 0.028
INITIAL_THETA = 1.5708  # 90 degrees in radians

# WHEEL_RADIUS = 0.0201  # r (meters)
# WHEEL_AXLE_LENGTH = 0.053  # d (meters)

# Map Settings
MAP_RESOLUTION = 300
WP = [
    (0, 0.25),
    (0, 0.68),
    (0.25, 0.68),
    (0.43, 0.68),
    (0.66, 0.52),
    (0.35, 0.25),
    (0.63, 0.01),
    (0.63, -0.16),
    (0.13, -0.16),
    (0, -0.16),
    (0, 0),
    (0, 0.25),
]

P1 = 3
P2 = 6.28 * 2
