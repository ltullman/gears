# A demo program written for Spike Prime to accomplish FLL Cargo Connect missions.
# Click Run button to run the program in simulator mode.
# To change settings like robot start position etc, go to menu MCU -> Settings.

# Import modules
from spike import ColorSensor, DistanceSensor, MotorPair, PrimeHub
from spike.control import wait_for_seconds, wait_until
from spike.operator import equal_to, less_than_or_equal_to

# Parameters
BLACK = 36
WHITE = 97
THRESHOLD = (BLACK + WHITE) / 2

# Robot parts
# Note that the ports are hard-coded and
# cannot be changed for now.
motion_sensor = PrimeHub().motion_sensor
motion_sensor.reset_yaw_angle()
motor_pair = MotorPair('A', 'B')
# Port C and D are reserved for front and back motors.
color_sensor = ColorSensor('E')
# We use a (hypothetical) port G here.
# Spike Prime only supports 6 ports, but the simulator
# can have more than that.
distance_sensor = DistanceSensor('F')

# Follow line forever
def follow_line(speed=60):
    PROPORTIONAL_GAIN = 2
    while True:
        deviation = color_sensor.get_reflected_light() - THRESHOLD
        turn_rate = int(PROPORTIONAL_GAIN * deviation)
        motor_pair.start_at_power(steering=turn_rate, power=speed)

# Does this work for helicopter air drop mission?
motor_pair.move(15, speed=50)
follow_line()

# Distance sensor works as well.
# motor_pair.start(speed=100)
# wait_until(distance_sensor.get_distance_cm, less_than_or_equal_to, 20)
# motor_pair.stop()