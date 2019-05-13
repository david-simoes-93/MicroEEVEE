# -*- coding: utf-8 -*-
# Define Libraries
import RPi.GPIO as GPIO
import time

# inicia a roda parada, vai aumentado a velocidade. Quando chega aos 100% de duty cycle, inverte a rotação e assim sucessivamente em loop


ENA_0 = 16
ENA_1 = 18
PWM_A = 12
rotation_value_A = 0

ENB_0 = 29
ENB_1 = 31
PWM_B = 33
rotation_value_B = 0


def invertRotation(motor_num):
    if (motor_num == 'A'):
        if (rotation_value_A):
            GPIO.output(ENA_0, initial=GPIO.HIGH)
            GPIO.output(ENA_1, initial=GPIO.LOW)
            rotation_value_A = 0
        else:
            GPIO.output(ENA_1, initial=GPIO.HIGH)
            GPIO.output(ENA_0, initial=GPIO.LOW)
            rotation_value_A = 1
    elif (motor_num == 'B'):
        if (rotation_value_B):
            GPIO.output(ENB_0, initial=GPIO.HIGH)
            GPIO.output(ENB_1, initial=GPIO.LOW)
            rotation_value_B = 0
        else:
            GPIO.output(ENB_1, initial=GPIO.HIGH)
            GPIO.output(ENB_0, initial=GPIO.LOW)
            rotation_value_B = 1
    else:
        print("ERROR no motor with that name [A or B]\n")


# Configuring don’t show warnings
GPIO.setwarnings(False)

# Configuring GPIO
GPIO.setmode(GPIO.BOARD)
print("RPI info", GPIO.RPI_INFO)

GPIO.setup(PWM_A, GPIO.OUT)
GPIO.setup(PWM_B, GPIO.OUT)

# Enables
GPIO.setup(ENA_0, GPIO.OUT, initial=GPIO.HIGH)  # enableA
GPIO.setup(ENA_1, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(ENB_0, GPIO.OUT, initial=GPIO.HIGH)  # enableB
GPIO.setup(ENB_1, GPIO.OUT, initial=GPIO.LOW)

# Configure the pwm objects and initialize its value
pwmBlue = GPIO.PWM(PWM_A, 100)
pwmBlue.start(0)

pwmRed = GPIO.PWM(PWM_B, 100)
pwmRed.start(100)

# Create the dutycycle variables
dcBlue = 0
dcRed = 100

# Loop infinite
while True:

    # increment gradually the luminosity
    pwmBlue.ChangeDutyCycle(dcBlue)
    time.sleep(0.05)
    dcBlue = dcBlue + 1
    if dcBlue == 100:
        invertRotation('A')
        dcBlue = 0

    # decrement gradually the luminosity
    pwmRed.ChangeDutyCycle(dcRed)
    time.sleep(0.05)
    dcRed = dcRed - 1
    if dcRed == 0:
        invertRotation('B')
        dcRed = 100

# End code
pwmBlue.ChangeDutyCycle(0)
pwmRed.ChangeDutyCycle(0)
GPIO.cleanup()
exit()
