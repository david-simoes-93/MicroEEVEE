import math
from enum import Enum
from typing import List

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from FakeGpio import GPIO

import Utils

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


GRADUAL_SPEED_INCREMENT = 5
SPEED_MAX_MODIFIER = 0.6
SPEED_MAX_ANGLE = 45

class MovementHandler:
    def __init__(self, motor_left, motor_right, simulator):
        self.motor_left = motor_left
        self.motor_right = motor_right

        self.prev_left_speed = 0
        self.prev_right_speed = 0

        self.state = MotorState.STOPPED
        self.stopping_counter = 0

        self.simulator = simulator

    def update_sim(self, l_speed, r_speed):
        if not self.simulator:
            return
        self.simulator.set_motor_pwm(l_speed, r_speed)

    def slow_adapt_speed(self, l_speed, r_speed):
        if l_speed - self.prev_left_speed > GRADUAL_SPEED_INCREMENT:
            l_speed = self.prev_left_speed + GRADUAL_SPEED_INCREMENT
        elif l_speed - self.prev_left_speed < -GRADUAL_SPEED_INCREMENT:
            l_speed = self.prev_left_speed - GRADUAL_SPEED_INCREMENT

        if r_speed - self.prev_right_speed > GRADUAL_SPEED_INCREMENT:
            r_speed = self.prev_right_speed + GRADUAL_SPEED_INCREMENT
        elif r_speed - self.prev_right_speed < -GRADUAL_SPEED_INCREMENT:
            r_speed = self.prev_right_speed - GRADUAL_SPEED_INCREMENT


        self.motor_left.set(l_speed)
        self.motor_right.set(r_speed)
        # print("Moving: %4.2f %4.2f" % (l_speed, r_speed))
        self.prev_left_speed = l_speed
        self.prev_right_speed = r_speed

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

    def follow_direction(self, theta_target, my_theta, speed):
        l_speed = speed
        r_speed = speed

        # cap angle difference at 90º
        theta_diff = Utils.to_degree(theta_target - my_theta)
        # print("theta_diff", to_degree(theta_diff))

        if theta_diff > SPEED_MAX_ANGLE:
            l_speed *= 1+SPEED_MAX_MODIFIER
            r_speed *= 1-SPEED_MAX_MODIFIER
        elif theta_diff < -SPEED_MAX_ANGLE:
            l_speed *= 1+SPEED_MAX_MODIFIER
            r_speed *= 1-SPEED_MAX_MODIFIER
        else:
            modifier = theta_diff * SPEED_MAX_MODIFIER / SPEED_MAX_ANGLE # [-0.5, 0.5]
            l_speed *= 1+modifier
            r_speed *= 1-modifier
        #print(f"speed of {l_speed:.1f},{r_speed:.1f} to angle {theta_diff}")
        self.slow_adapt_speed(l_speed, r_speed)

        self.state = MotorState.FORWARD

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

