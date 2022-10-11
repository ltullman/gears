# A demo program written for Spike Prime to accomplish FLL Cargo Connect missions.
# Click Run button to run the program in simulator mode.
# To change settings like robot start position etc, go to menu MCU -> Settings.

# Import modules
from spike import ColorSensor, DistanceSensor, MotorPair, PrimeHub
from spike.control import wait_for_seconds, wait_until
from spike.operator import equal_to, less_than_or_equal_to

# Robot parts
# Note that the ports are hard-coded and
# cannot be changed for now.
motor_pair = MotorPair('A', 'B')
motor_pair.move_tank(100, 'degrees', left_speed=35, right_speed=35)
