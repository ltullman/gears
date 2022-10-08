#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from spike import *
from spike.control import *
from spike.operator import *

# Create the sensors and motors objects
OUTPUT_A = 'outA'
OUTPUT_B = 'outB'
OUTPUT_C = 'outC'

INPUT_1 = 'in1'
INPUT_2 = 'in2'
INPUT_3 = 'in3'
INPUT_4 = 'in4'
INPUT_5 = 'in5'

motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in3 = GyroSensor(INPUT_3)
gps_sensor_in4 = GPSSensor(INPUT_4)
pen_in5 = Pen(INPUT_5)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts

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