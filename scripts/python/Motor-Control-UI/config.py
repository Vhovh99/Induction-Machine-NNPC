# Configuration settings for Motor Control UI

# Serial Communication (matches firmware LPUART1 settings)
SERIAL_BAUDRATE = 230400
SERIAL_TIMEOUT = 1.0  # seconds

# Data Buffer
BUFFER_SIZE = 500  # Number of samples to keep in memory

# UI Settings
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 900
REFRESH_RATE = 50  # milliseconds

# Speed Reference Limits (RPM) — firmware converts RPM to rad/s internally
SPEED_REF_MIN = -2000.0
SPEED_REF_MAX = 2000.0
SPEED_REF_DEFAULT = 500.0

# Id Reference Limits (A)
ID_REF_MIN = -10.0
ID_REF_MAX = 10.0
ID_REF_DEFAULT = 1.13  # firmware default

# Telemetry Divider
TELEMETRY_DIV_MIN = 0
TELEMETRY_DIV_MAX = 10000
TELEMETRY_DIV_DEFAULT = 200  # firmware default

# Motor Parameters (for display conversions)
MOTOR_POLE_PAIRS = 2
PWM_FREQUENCY = 20000  # Hz — firmware PWM/ISR frequency

# Plotting
PLOT_LINE_WIDTH = 1.5
