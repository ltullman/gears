# Import modules.
import simPython
from math import pi
from spike import ColorSensor, Motor, MotorPair, PrimeHub
from spike.control import wait_for_seconds
from time import ticks_ms

# Constants
# If you see a large variance among runs, set
# the speed values lower.
DEFAULT_DRIVE_SPEED = 80
# Recommend to keep the attachment motor speed low.
# You can overwrite the speed in individual
# run_for_degrees() calls.
DEFAULT_MOTOR_SPEED = 12
DEFAULT_ROTATE_SPEED = 30
DEFAULT_FOLLOW_LINE_SPEED = 40

# Don't change these values unless you know how they works.
WHEEL_DIAMETER = 5.6
LIGHT_DROP_THRESHOLD = -50
RAMP_DEGREES = 180
OVER_ROTATE_DEGREES = 0
BLACK = 36
WHITE = 97
THRESHOLD = (BLACK + WHITE) / 2

# Robot parts
motion_sensor = PrimeHub().motion_sensor
motion_sensor.reset_yaw_angle()
motor_pair = MotorPair('A', 'B')
motor_pair.set_default_speed(DEFAULT_DRIVE_SPEED)
motor_pair.set_stop_action('brake')
left_motor = Motor('A')
right_motor = Motor('B')
front_motor = Motor('C')
front_motor.set_default_speed(DEFAULT_MOTOR_SPEED)
back_motor = Motor('D')
back_motor.set_default_speed(DEFAULT_MOTOR_SPEED)
left_color_sensor = ColorSensor('E')
right_color_sensor = ColorSensor('F')

# Global state
pre_stop_angle = None

# Calculate diff of two angles and handle overflows.
def angle_diff(a, b):
    diff = (a - b + 360) % 360
    return diff if diff <= 180 else diff - 360

# Go straight for given distance, and/or until hit a black line.
# Set cm to a negative value to go backward.
# Gyro yaw angle is used to calibrate direction.
def go_straight(cm=0, black_sensor=None, speed=DEFAULT_DRIVE_SPEED):
    global pre_stop_angle
    PROPORTIONAL_GAIN = 12
    STUCK_DETECTION_DEGREES = 10
    STUCK_DETECTION_MS = 500
    power = speed
    if cm < 0:
        power = -power
        PROPORTIONAL_GAIN = -PROPORTIONAL_GAIN
    base = motion_sensor.get_yaw_angle() if pre_stop_angle is None else pre_stop_angle
    target_degrees = abs(cm) / (pi * WHEEL_DIAMETER) * 360
    left_motor.set_degrees_counted(0)
    right_motor.set_degrees_counted(0)
    last_time = ticks_ms()
    last_degrees = 0
    last_light = BLACK
    light_diff = []
    while True:
        # Break out if robot has hit a black line.
        if black_sensor is not None:
            # Detect white -> black transition.
            # Use reflected light diffs instead of absolute values
            # to adapt to different lighting environments.
            light = black_sensor.get_reflected_light()
            light_diff.append(light - last_light)
            light_diff = light_diff[-5:]
            if sum(light_diff) < LIGHT_DROP_THRESHOLD:
                break
            last_light = light
        degrees = abs(left_motor.get_degrees_counted())
        degrees += abs(right_motor.get_degrees_counted())
        degrees = degrees / 2
        # Break out if robot has gone for desired distance.
        if target_degrees != 0 and degrees >= target_degrees:
            break
        # Break out if robot has been stuck.
        time = ticks_ms()
        if degrees - last_degrees > STUCK_DETECTION_DEGREES:
            last_degrees = degrees
            last_time = time
        elif time - last_time > STUCK_DETECTION_MS:
            break
        actual_power = power
        if degrees < RAMP_DEGREES or target_degrees - degrees < RAMP_DEGREES:
            actual_power = actual_power * 2 // 3
        if target_degrees > 0 and target_degrees - degrees < RAMP_DEGREES / 3:
            actual_power = actual_power // 2
        deviation = angle_diff(motion_sensor.get_yaw_angle(), base)
        turn_rate = int(PROPORTIONAL_GAIN * deviation)
        motor_pair.start_at_power(steering=-turn_rate, power=actual_power)
    pre_stop_angle = motion_sensor.get_yaw_angle()
    motor_pair.stop()

# Rotate clockwise for given degrees.
# To rotate counter-clockwise, set degrees to a negative value.
def rotate(degrees, speed=DEFAULT_ROTATE_SPEED):
    global pre_stop_angle
    degrees = degrees - OVER_ROTATE_DEGREES if degrees > 0 else degrees + OVER_ROTATE_DEGREES
    base = motion_sensor.get_yaw_angle() if pre_stop_angle is None else pre_stop_angle
    if degrees > 0:
        left_speed = speed
        right_speed = -speed
    else:
        left_speed = -speed
        right_speed = speed
    motor_pair.start_tank(left_speed=left_speed, right_speed=right_speed)
    while True:
        diff = angle_diff(motion_sensor.get_yaw_angle(), base)
        gap = (degrees - diff) * (degrees / abs(degrees))
        if gap <= 0:
            break
        # Sometimes robot rotates forever without this line.
        wait_for_seconds(0.005)
    pre_stop_angle = motion_sensor.get_yaw_angle()
    motor_pair.stop()

# Follow line on right color sensor until left color sensor
# hits black N times.
# Right color sensor should be placed between white line (on the left)
# and black line (on the right).
def follow_line(hit_black_count=1, speed=DEFAULT_FOLLOW_LINE_SPEED):
    global pre_stop_angle
    PROPORTIONAL_GAIN = 2
    last_light = BLACK
    light_diff = []
    while True:
        # Detect white -> black transition.
        # Use reflected light diffs instead of absolute values
        # to adapt to different lighting environments.
        light = left_color_sensor.get_reflected_light()
        light_diff.append(light - last_light)
        light_diff = light_diff[-5:]
        if sum(light_diff) < LIGHT_DROP_THRESHOLD:
            hit_black_count -= 1
            light_diff = []
        if hit_black_count <= 0:
            break
        last_light = light
        deviation = right_color_sensor.get_reflected_light() - THRESHOLD
        turn_rate = int(PROPORTIONAL_GAIN * deviation)
        motor_pair.start_at_power(steering=turn_rate, power=speed)
    pre_stop_angle = motion_sensor.get_yaw_angle()
    motor_pair.stop()

# Calibrate the robot starting position,
# by following line with right color sensor
# until left color sensor hitting a black spot.
def calibrate(hit_black_count=1, speed=DEFAULT_FOLLOW_LINE_SPEED):
    global pre_stop_angle
    PROPORTIONAL_GAIN = 2
    last_light = BLACK
    light_diff = []
    motion_sensor.reset_yaw_angle()
    angles = []
    while True:
        # Detect white -> black transition.
        # Use reflected light diffs instead of absolute values
        # to adapt to different lighting environments.
        light = left_color_sensor.get_reflected_light()
        light_diff.append(light - last_light)
        light_diff = light_diff[-5:]
        if sum(light_diff) < LIGHT_DROP_THRESHOLD:
            hit_black_count -= 1
            light_diff = []
        if hit_black_count <= 0:
            break
        last_light = light
        deviation = right_color_sensor.get_reflected_light() - THRESHOLD
        turn_rate = int(PROPORTIONAL_GAIN * deviation)
        motor_pair.start_at_power(steering=turn_rate, power=speed)
        angles.append(motion_sensor.get_yaw_angle())
        angles = angles[-100:]
    pre_stop_angle = sum(angles) // len(angles)
    motor_pair.stop()
