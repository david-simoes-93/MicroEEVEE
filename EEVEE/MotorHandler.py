import RPi.GPIO as GPIO
import math

from EEVEE.Utils import normalize_radian_angle

PI = 3.14159265359

# Motor parameters
RPM_OUT = 210  # RPM in the output shaft @ 9.6V
GEAR_RATIO = 34  # Gear box ratio
ENCODER_PULSES = 11  # Encoder, pulses per revolution

# Robot dimensions
WHEEL2WHEEL_DIST = 19  # cm
WHEEL_DIAM = 6.5  # cm
WHEEL_PER = PI * WHEEL_DIAM


# Motor actuator
class MotorActuator:
    # In Board mode, the EN0, EN1, and PWM pin numbers
    def __init__(self, en_a, en_b, pwm):
        GPIO.setup(pwm, GPIO.OUT)
        GPIO.setup(en_a, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(en_b, GPIO.OUT, initial=GPIO.LOW)

        # Configure the pwm and enable pins
        self.pwm = GPIO.PWM(pwm, 100)   # 100 Hz
        self.en_a = en_a
        self.en_b = en_b

        self.pwm.start(0)

    def set(self, pwm):
        if pwm > 0:
            GPIO.output(self.en_a, GPIO.HIGH)
            GPIO.output(self.en_b, GPIO.LOW)
        else:
            GPIO.output(self.en_a, GPIO.LOW)
            GPIO.output(self.en_b, GPIO.HIGH)

        # https://www.bananarobotics.com/shop/How-to-use-the-L298N-Dual-H-Bridge-Motor-Driver
        # Don't use above 90% PWM
        self.pwm.ChangeDutyCycle(min(abs(pwm),90))


def odometry(encLeft, encRight, xpos, ypos, theta):
    dLeft = (encLeft * WHEEL_PER) / (ENCODER_PULSES * GEAR_RATIO)
    dRight = (encRight * WHEEL_PER) / (ENCODER_PULSES * GEAR_RATIO)

    dCenter = (dLeft + dRight) / 2.0
    phi = (dRight - dLeft) / WHEEL2WHEEL_DIST

    ypos = ypos + dCenter * math.sin(theta)
    xpos = xpos + dCenter * math.cos(theta)
    theta = normalize_radian_angle(theta + phi)

    return xpos, ypos, theta

