# Configuration settings for Motor Control UI

# Serial Communication
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 1.0  # seconds
TELEMETRY_INTERVAL = 100  # milliseconds (as per protocol)

# Data Buffer
BUFFER_SIZE = 500  # Number of samples to keep in memory
PLOT_UPDATE_INTERVAL = 200  # milliseconds

# UI Settings
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 900
REFRESH_RATE = 50  # milliseconds

# Motor Control Limits
FREQUENCY_MIN = 0.0  # Hz
FREQUENCY_MAX = 100.0  # Hz
FREQUENCY_DEFAULT = 50.0  # Hz

AMPLITUDE_MIN = 0.0
AMPLITUDE_MAX = 1.0
AMPLITUDE_DEFAULT = 0.5

RPM_MIN = 0
RPM_MAX = 3000
RPM_DEFAULT = 1500

# Torque Estimation (placeholder constants)
# These would need to be calibrated based on motor specifications
MOTOR_POLE_PAIRS = 2
MOTOR_MOMENT_OF_INERTIA = 0.001  # kg*m^2
FRICTION_COEFFICIENT = 0.01  # N*m*s

# Plotting
PLOT_STYLE = 'seaborn-v0_8-darkgrid'
PLOT_LINE_WIDTH = 1.5
PLOT_UPDATE_FPS = 10  # frames per second
