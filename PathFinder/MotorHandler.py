import math
from enum import Enum

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from PathFinder.FakeGpio import GPIO

from Utils import normalize_radian_angle

# Motor parameters
# "Gear box ratio" times "Encoder, pulses per revolution"
GEAR_RATIO_times_ENCODER_PULSES = 236  # 34 * 11 but with 1.58 ratio?

# Robot dimensions
WHEEL2WHEEL_DIST = 15.9  # 14.7  # cm
WHEEL_DIAM = 6.7  # cm
WHEEL_PER = math.pi * WHEEL_DIAM


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
    def __init__(self, motor_left, motor_right):
        self.motor_left = motor_left
        self.motor_right = motor_right

        self.prev_left_speed = 0
        self.prev_right_speed = 0

        self.gradual_speed_increment = 5

        self.state = MotorState.STOPPED
        self.stopping_counter = 0

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
        theta_diff = min(max(normalize_radian_angle(
            theta_target - my_theta), -math.pi / 2), math.pi / 2)
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

    @classmethod
    def odometry(cls, encLeft, encRight, xpos, ypos, theta):
        dLeft = (encLeft * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES
        dRight = (encRight * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES
        print(f"{dLeft} {dRight}")

        dCenter = (dLeft + dRight) / 2.0
        phi = (dRight - dLeft) / WHEEL2WHEEL_DIST

        ypos = ypos + dCenter * math.sin(theta)
        xpos = xpos + dCenter * math.cos(theta)
        theta = normalize_radian_angle(theta + phi)

        return xpos, ypos, theta
