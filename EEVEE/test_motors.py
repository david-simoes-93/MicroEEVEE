# -*- coding: utf-8 -*-
# Define Libraries
import RPi.GPIO as GPIO
import time

# inicia a roda parada, vai aumentado a velocidade.
# Quando chega aos 100% de duty cycle, inverte a rotação e assim sucessivamente em loop


EN0_left_pin = 16
EN1_left_pin = 18
PWM_left_pin = 12
rotation_value_left = 0

EN0_right_pin = 29
EN1_right_pin = 31
PWM_right_pin = 33
rotation_right = 0


# int pin0, int pin1, bool forward
def invert_rotation(pin0, pin1, forward):
    if forward:
        GPIO.output(pin0, initial=GPIO.HIGH)
        GPIO.output(pin1, initial=GPIO.LOW)
    else:
        GPIO.output(pin0, initial=GPIO.LOW)
        GPIO.output(pin1, initial=GPIO.HIGH)


def invert_rotation(motor_num):
    global rotation_value_left, rotation_value_right

    if motor_num == 'A':
        if rotation_value_left:
            GPIO.output(EN0_left_pin, initial=GPIO.HIGH)
            GPIO.output(EN1_left_pin, initial=GPIO.LOW)
            rotation_value_left = 0
        else:
            GPIO.output(EN1_left_pin, initial=GPIO.HIGH)
            GPIO.output(EN0_left_pin, initial=GPIO.LOW)
            rotation_value_left = 1
    elif motor_num == 'B':
        if rotation_value_right:
            GPIO.output(EN0_right_pin, initial=GPIO.HIGH)
            GPIO.output(EN1_right_pin, initial=GPIO.LOW)
            rotation_value_right = 0
        else:
            GPIO.output(EN1_right_pin, initial=GPIO.HIGH)
            GPIO.output(EN0_right_pin, initial=GPIO.LOW)
            rotation_value_right = 1
    else:
        print("ERROR no motor with that name [A or B]\n")

# Configuring don’t show warnings
#GPIO.setwarnings(False)

# Configuring GPIO
GPIO.setmode(GPIO.BOARD)
print("RPI info", GPIO.RPI_INFO)

GPIO.setup(PWM_left_pin, GPIO.OUT)
GPIO.setup(PWM_right_pin, GPIO.OUT)

# Enables
GPIO.setup(EN0_left_pin, GPIO.OUT, initial=GPIO.HIGH)  # enableA
GPIO.setup(EN1_left_pin, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(EN0_right_pin, GPIO.OUT, initial=GPIO.HIGH)  # enableB
GPIO.setup(EN1_right_pin, GPIO.OUT, initial=GPIO.LOW)

# Configure the pwm objects and initialize its value
pwm_left = GPIO.PWM(PWM_left_pin, 100)
pwm_left.start(0)

pwm_right = GPIO.PWM(PWM_right_pin, 100)
pwm_right.start(100)

# Create the dutycycle variables
duty_cycle_left = 0
duty_cycle_right = 100

# Loop infinite
while True:

    # increment gradually the speed
    pwm_left.ChangeDutyCycle(duty_cycle_left)
    time.sleep(0.05)
    duty_cycle_left += 1
    if duty_cycle_left == 100:
        invert_rotation('A')
        duty_cycle_left = 0

    # decrement gradually the speed
    pwm_right.ChangeDutyCycle(duty_cycle_right)
    time.sleep(0.05)
    duty_cycle_right -= 1
    if duty_cycle_right == 0:
        invert_rotation('B')
        duty_cycle_right = 100

# End code
pwm_left.ChangeDutyCycle(0)
pwm_right.ChangeDutyCycle(0)
GPIO.cleanup()
exit()
