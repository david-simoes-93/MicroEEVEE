import math
from enum import Enum
from typing import List

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from PathFinder.FakeGpio import GPIO

import Utils

# Motor parameters
# "Gear box ratio" times "Encoder, pulses per revolution"
GEAR_RATIO_times_ENCODER_PULSES = 236  # 34 * 11 but with 1.58 ratio?

# Robot dimensions
WHEEL2WHEEL_DIST = 15.9  # 14.7  # cm
WHEEL_DIAM = 6.7  # cm
WHEEL_PER = math.pi * WHEEL_DIAM

RIGID_COMPASS_THRESHOLD = Utils.to_radian(10)
ODOMETRY_THETA_THRESHOLD = Utils.to_radian(10)
ODOMETRY_GPS_THRESHOLD = 2
ODOMETRY_GRANULARITY_ARR = [0, 1, -1, 2, -2, 3, -3, 4, -4]
ODOMETRY_GRANULARITY = 4
CM_PER_CELL = 12.5
HALF_LINE_WIDTH = 1.25


class MotorState(Enum):
    STOPPED = 0
    FORWARD = 1
    TURNING_LEFT = 2
    TURNING_RIGHT = 3
    BACK = 4
    STOPPING = 5


# Motor actuator
class MotorActuator:
    # In Board mode, the EN0, EN1, and PWM pin numbers
    def __init__(self, en_a, en_b, pwm):
        GPIO.setup(pwm, GPIO.OUT)
        GPIO.setup(en_a, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(en_b, GPIO.OUT, initial=GPIO.LOW)

        # Configure the pwm and enable pins
        self.pwm = GPIO.PWM(pwm, 100)  # 100 Hz
        self.en_a = en_a
        self.en_b = en_b

        self.pwm.start(0)

    def set(self, pwm):
        pwm = max(min(100, pwm), -100)

        if pwm > 0:
            GPIO.output(self.en_a, GPIO.HIGH)
            GPIO.output(self.en_b, GPIO.LOW)
        elif pwm < 0:
            GPIO.output(self.en_a, GPIO.LOW)
            GPIO.output(self.en_b, GPIO.HIGH)
        else:
            GPIO.output(self.en_a, GPIO.LOW)
            GPIO.output(self.en_b, GPIO.LOW)

        # https://www.bananarobotics.com/shop/How-to-use-the-L298N-Dual-H-Bridge-Motor-Driver
        # Don't use above 90% PWM
        self.pwm.ChangeDutyCycle(min(abs(pwm), 90))


class MovementHandler:
    def __init__(self, motor_left, motor_right, simulator):
        self.motor_left = motor_left
        self.motor_right = motor_right

        self.prev_left_speed = 0
        self.prev_right_speed = 0

        self.gradual_speed_increment = 5

        self.state = MotorState.STOPPED
        self.stopping_counter = 0

        self.simulator = simulator

    def update_sim(self, l_speed, r_speed):
        if not self.simulator:
            return
        self.simulator.set_motor_pwm(l_speed, r_speed)

    def slow_adapt_speed(self, l_speed, r_speed):
        if l_speed - self.prev_left_speed > self.gradual_speed_increment:
            self.prev_left_speed += self.gradual_speed_increment
        elif l_speed - self.prev_left_speed < -self.gradual_speed_increment:
            self.prev_left_speed -= self.gradual_speed_increment
        else:
            self.prev_left_speed = l_speed

        if r_speed - self.prev_right_speed > self.gradual_speed_increment:
            self.prev_right_speed += self.gradual_speed_increment
        elif r_speed - self.prev_right_speed < -self.gradual_speed_increment:
            self.prev_right_speed -= self.gradual_speed_increment
        else:
            self.prev_right_speed = r_speed

        # self.prev_left_speed = (self.prev_left_speed + l_speed) / 2
        # self.prev_right_speed = (self.prev_right_speed + r_speed) / 2

        self.motor_left.set(l_speed)
        self.motor_right.set(r_speed)
        # print("Moving: %4.2f %4.2f" % (l_speed, r_speed))
        self.update_sim(l_speed,r_speed)

    def rotate_right(self, speed=45):
        l_speed = speed
        r_speed = -speed
        self.slow_adapt_speed(l_speed, r_speed)

        self.state = MotorState.TURNING_RIGHT

    def rotate_left(self, speed=45):
        l_speed = -speed
        r_speed = speed
        self.slow_adapt_speed(l_speed, r_speed)

        self.state = MotorState.TURNING_LEFT

    def forward(self, speed=40):
        l_speed = speed
        r_speed = speed
        self.slow_adapt_speed(l_speed, r_speed)

        # not possible to have positive left and negative right, i think
        if self.prev_left_speed > 0 or self.prev_right_speed > 0:
            self.state = MotorState.FORWARD
        else:
            self.state = MotorState.BACK

    def follow_direction(self, theta_target, my_theta, speed=40):
        l_speed = speed
        r_speed = speed

        # cap angle difference at 45º
        theta_diff = min(max(Utils.normalize_radian_angle(theta_target - my_theta), -math.pi / 2), math.pi / 2)
        # print("theta_diff", to_degree(theta_diff))
        if (theta_diff > math.pi / 4 and speed > 0) or (theta_diff < -math.pi / 4 and speed < 0):
            l_speed *= 1.2
            r_speed *= 0.8
        elif (theta_diff > 0 and speed > 0) or (theta_diff < 0 and speed < 0):
            l_speed *= 1.1
            r_speed *= 0.9
        elif (theta_diff < -math.pi / 4 and speed > 0) or (theta_diff > math.pi / 4 and speed < 0):
            l_speed *= 0.8
            r_speed *= 1.2
        elif (theta_diff < 0 and speed > 0) or (theta_diff > 0 and speed < 0):
            l_speed *= 0.9
            r_speed *= 1.1

        self.slow_adapt_speed(l_speed, r_speed)

        # not possible to have positive left and negative right, i think
        if self.prev_left_speed > 0 or self.prev_right_speed > 0:
            self.state = MotorState.FORWARD
        else:
            self.state = MotorState.BACK

    def stop(self):
        self.motor_left.set(0)
        self.motor_right.set(0)

        if self.prev_left_speed != 0 or self.prev_right_speed != 0:
            self.stopping_counter = 0
            self.state = MotorState.STOPPING
        elif self.stopping_counter > 5:
            self.state = MotorState.STOPPED
        self.stopping_counter += 1

        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.update_sim(0,0)

    def emergency_stop(self):
        # abrupt stop
        self.motor_left.set(-self.prev_left_speed)
        self.motor_right.set(-self.prev_right_speed)

        # decrease speed
        if self.prev_left_speed > 10:
            self.prev_left_speed -= 10
        elif self.prev_left_speed < -10:
            self.prev_left_speed += 10
        else:
            self.prev_left_speed = 0

        if self.prev_right_speed > 10:
            self.prev_right_speed -= 10
        elif self.prev_right_speed < -10:
            self.prev_right_speed += 10
        else:
            self.prev_right_speed = 0
        self.update_sim(0,0)

    @classmethod
    def odometry(cls, encLeft: int, encRight: int, gps_x: float, gps_y: float, theta: float, ground_sensors: List[bool]):
        dLeft = (encLeft * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES
        dRight = (encRight * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES

        dCenter = (dLeft + dRight) / 2.0
        phi = (dRight - dLeft) / WHEEL2WHEEL_DIST

        gps_y = gps_y + dCenter * math.sin(theta)
        gps_x = gps_x + dCenter * math.cos(theta)
        theta = Utils.normalize_radian_angle(theta + phi)

        """
        if (ground_sensors[0] or ground_sensors[4]) and abs(Utils.get_rigid_compass(theta)-theta) < RIGID_COMPASS_THRESHOLD:
            # TODO: can adjust X/Y because we are at an intersection
            pass

        # ideally, we would find a near position in known map which matches our sensors perfectly
        # then if != from our estimate, we approximate our estimate
        # how about we try some stuff, maybe a rotation of 10º and then an adjustment of 2cm
        for gran_x in ODOMETRY_GRANULARITY_ARR:
            for gran_y in ODOMETRY_GRANULARITY_ARR:
                for gran_theta in ODOMETRY_GRANULARITY_ARR:
                    delta_x = gran_x * ODOMETRY_GPS_THRESHOLD / ODOMETRY_GRANULARITY
                    delta_y = gran_y * ODOMETRY_GPS_THRESHOLD / ODOMETRY_GRANULARITY
                    delta_theta = gran_theta * ODOMETRY_GPS_THRESHOLD / ODOMETRY_GRANULARITY
                    #score = check_score(gps_x, gps_y, theta, ground_sensors)
        """

        
        # how about instead we draw lines across our positive sensors
        # line across far ones must be rigid_angle
        # line across mids must be rigid angle
        # other lines don't say much
        if (ground_sensors[0] and ground_sensors[4]) or (ground_sensors[1] and ground_sensors[3]):
            rigid_compass = Utils.get_rigid_compass(theta)
            # if we are rotated more than we should be, but not TOO much (to avoid fucking rotations)
            if ODOMETRY_THETA_THRESHOLD < abs(rigid_compass-theta) < 2 * ODOMETRY_THETA_THRESHOLD:
                if theta > rigid_compass:
                    new_theta = rigid_compass + ODOMETRY_THETA_THRESHOLD
                else:
                    new_theta = rigid_compass - ODOMETRY_THETA_THRESHOLD
                print(f"adjusting theta from {Utils.to_degree(theta):.2f} to {Utils.to_degree(new_theta):.2f}")
                theta = new_theta

        # and then after fixing theta, we just match
        # either of the far ones
        # either of the mids

        """
        if ground_sensors[0]:
            # get sensor pos
            sensor_coords = Utils.far_left_sensor_gps(gps_x, gps_y, theta)
            sensor_delta_x = sensor_coords[0] - round(sensor_coords[0] / CM_PER_CELL)*CM_PER_CELL
            sensor_delta_y = sensor_coords[1] - round(sensor_coords[1] / CM_PER_CELL)*CM_PER_CELL
            # either sensor x or y must be a multiple of 12.5+-1.25, but not off by too far
            if HALF_LINE_WIDTH < min(abs(sensor_delta_x), abs(sensor_delta_y)) < 2 * HALF_LINE_WIDTH:
                if abs(sensor_delta_x) < abs(sensor_delta_y):
                    # x is closer, so adjust x
                    if sensor_delta_x > HALF_LINE_WIDTH:
                        new_gps_x = gps_x - (sensor_delta_x-HALF_LINE_WIDTH)
                    else:
                        new_gps_x = gps_x - (sensor_delta_x+HALF_LINE_WIDTH)
                    print(f"adjusting x from {gps_x} to {new_gps_x}: {sensor_delta_x}")
                    gps_x = new_gps_x
                else:
                    # y is closer, so adjust y
                    if sensor_delta_y > HALF_LINE_WIDTH:
                        new_gps_y = gps_y - (sensor_delta_y-HALF_LINE_WIDTH)
                    else:
                        new_gps_y = gps_y - (sensor_delta_y+HALF_LINE_WIDTH)
                    print(f"adjusting y from {gps_y} to {new_gps_y}: {sensor_delta_y}")
                    gps_y = new_gps_y
        else:
            # neither sensor x or y must be a multiple of 12.5+-1.25
            pass
        """
        
        gps_deltas = [MovementHandler.get_sensor_adjustment(ground_sensors[0], Utils.far_left_sensor_gps(gps_x, gps_y, theta)),
                      MovementHandler.get_sensor_adjustment(ground_sensors[1], Utils.left_sensor_gps(gps_x, gps_y, theta)),
                      MovementHandler.get_sensor_adjustment(ground_sensors[2], Utils.front_sensor_gps(gps_x, gps_y, theta)),
                      MovementHandler.get_sensor_adjustment(ground_sensors[3], Utils.right_sensor_gps(gps_x, gps_y, theta)),
                      MovementHandler.get_sensor_adjustment(ground_sensors[4], Utils.far_right_sensor_gps(gps_x, gps_y, theta))]
        avg_gps_delta_x, avg_gps_delta_y = 0, 0
        for gps_delta in gps_deltas:
            avg_gps_delta_x += gps_delta[0]
            avg_gps_delta_y += gps_delta[1]
        if avg_gps_delta_x != 0 or avg_gps_delta_y != 0:
            avg_gps_delta_x /= len(gps_deltas)
            avg_gps_delta_y /= len(gps_deltas)
            print(f"adjusting [x,y] from [{gps_x:.2f},{gps_y:.2f}] by [{avg_gps_delta_x:.2f},{avg_gps_delta_y:.2f}]")
            gps_x = gps_x + avg_gps_delta_x
            gps_y = gps_y + avg_gps_delta_y
        
        return gps_x, gps_y, theta
    
    @classmethod
    def get_sensor_adjustment(cls, sensor_val, sensor_coords):
        # get sensor pos
        sensor_delta_x = sensor_coords[0] - round(sensor_coords[0] / CM_PER_CELL)*CM_PER_CELL
        sensor_delta_y = sensor_coords[1] - round(sensor_coords[1] / CM_PER_CELL)*CM_PER_CELL

        if sensor_val:
            # either sensor x or y must be a multiple of 12.5+-1.25, but not off by too far
            if HALF_LINE_WIDTH < min(abs(sensor_delta_x), abs(sensor_delta_y)) < 2 * HALF_LINE_WIDTH:
                if abs(sensor_delta_x) < abs(sensor_delta_y):
                    # x is closer, so adjust x
                    if sensor_delta_x > HALF_LINE_WIDTH:
                        return [-sensor_delta_x+HALF_LINE_WIDTH, 0]
                    else:
                        return [-sensor_delta_x-HALF_LINE_WIDTH, 0]
                else:
                    # y is closer, so adjust y
                    if sensor_delta_y > HALF_LINE_WIDTH:
                        return [0, -sensor_delta_y+HALF_LINE_WIDTH]
                    else:
                        return [0, -sensor_delta_y-HALF_LINE_WIDTH]
        else:
            pass
        return [0,0]
