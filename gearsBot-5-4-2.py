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

hub = PrimeHub() 
motors = MotorPair('A','B') # create new pair of motors
motor_enc = Motor('A')

# spkr = Sound()
# btn = Button()
# radio = Radio()

# color_sensor_in1 = ColorSensor(INPUT_1)
# ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in1 = GyroSensor(INPUT_1)
# gps_sensor_in4 = GPSSensor(INPUT_4)
# pen_in5 = Pen(INPUT_5)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where code starts

# distance to drive in rotations
def drive_straight(rotations):
    # convert rotations to degrees
    distance = rotations * 360
    # get original angle for calculating difference
    original_angle = hub.motion_sensor.get_yaw_angle()
    # get original degrees counted for calculating difference
    original_pos = motor_enc.get_degrees_counted()

    while(original_pos - motor_enc.get_degrees_counted() < distance):
        # calculate change in angle
        new_angle = hub.motion_sensor.get_yaw_angle()
        delta = new_angle - original_angle

        # scale difference, for every 1 degree off
        # adjust turning by 2
        delta *= 2

        # move the motors by step_dist, speed is proportional to delta
        motors.start_tank(50 - delta, 50 + delta)

        print("target: {} | delta: {} | angle: {}".format(distance, original_pos - motor_enc.get_degrees_counted(), delta))

    # stop motors since moving is done
    motors.stop()
    
# test the program
drive_straight(5)