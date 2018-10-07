import time
from multiprocessing import Process, Value
from EEVEE.USHandler import us_async
from EEVEE.MotorHandler import MotorActuator
from EEVEE.LedHandler import LedActuator
import RPi.GPIO as GPIO
from EEVEE.ArduinoHandler import ArduinoHandler, EmptyArduino
import pygame
import math
from serial import SerialException


def render(screen, ir_left, ir_right, us_left, us_front, us_right, us_back,
           ground_far_left, ground_left, ground_mid, ground_right, ground_far_right,
           motor_left, motor_right):
    white, black = (255, 255, 255), (0, 0, 0)
    screen.fill(white)

    pygame.draw.circle(screen, black, [150, 150], 10)

    pygame.draw.line(screen, (0, 0, 255), [148, 150], [148 - int(ir_left * 10), 150], 2)
    pygame.draw.line(screen, (0, 0, 255), [152, 150], [152 + int(ir_right * 10), 150], 2)

    us_left_xy, us_right_xy = int(math.sqrt(us_left) / 2 * 10), int(math.sqrt(us_right) / 2 * 10)
    pygame.draw.line(screen, (0, 255, 0), [144, 144], [144 - us_left_xy, 144 - us_left_xy], 2)
    pygame.draw.line(screen, (0, 255, 0), [156, 144], [156 + us_right_xy, 144 - us_right_xy], 2)
    pygame.draw.line(screen, (0, 255, 0), [150, 141], [150, 141 - int(us_front * 10)], 2)
    pygame.draw.line(screen, (0, 255, 0), [150, 159], [150, 159 + int(us_back * 10)], 2)

    pygame.draw.circle(screen, black, [100, 10], 6)
    pygame.draw.circle(screen, black, [125, 10], 6)
    pygame.draw.circle(screen, black, [150, 10], 6)
    pygame.draw.circle(screen, black, [175, 10], 6)
    pygame.draw.circle(screen, black, [200, 10], 6)
    pygame.draw.circle(screen, black if ground_far_left else white, [100, 10], 5)
    pygame.draw.circle(screen, black if ground_left else white, [125, 10], 5)
    pygame.draw.circle(screen, black if ground_mid else white, [150, 10], 5)
    pygame.draw.circle(screen, black if ground_right else white, [175, 10], 5)
    pygame.draw.circle(screen, black if ground_far_right else white, [200, 10], 5)

    pygame.draw.line(screen, (255, 0, 0), [10, 150], [10, 150 - int(motor_left)], 5)
    pygame.draw.line(screen, (255, 0, 0), [290, 150], [290, 150 - int(motor_right)], 5)

    pygame.display.flip()


def main():
    gui = True
    remote_control = True

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # asynchronously update US sensors
    us0, us1, us2, us3 = Value('f', 0), Value('f', 0), Value('f', 0), Value('f', 0)
    keep_running_us = Value('b', True)
    Process(target=us_async, args=(keep_running_us, 13, 16, us0, 11, 18, us1, 7, 22, us2, 12, 24, us3)).start()

    # motors
    m1 = MotorActuator(35, 37, 33)  # IN1 IN2 ENA - Right Motor
    m2 = MotorActuator(38, 40, 32)  # IN3 IN4 ENB - Left Motor

    # LED
    led0 = LedActuator(26)

    # Arduino
    try:
        arduino = ArduinoHandler()
    except SerialException:
        print("Serial connection not found")
        arduino = EmptyArduino()


    if gui:
        pygame.init()
        screen = pygame.display.set_mode([300, 300])
        pygame.display.set_caption("EEVEE")

    left_motor_speed, right_motor_speed = 0,0

    while True:
        arduino.get()
        #print(us0.value, us1.value, us2.value, us3.value)
        #print(arduino.ir0, arduino.ir1)
        print(arduino.button0, arduino.button1)
        #print(arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4)
        print(arduino.m1_encoder, arduino.m2_encoder)

        led0.set(us1.value > 0.20)

        if remote_control:
            if left_motor_speed > 0:
                left_motor_speed -= 5
            elif left_motor_speed < 0:
                left_motor_speed += 5
            if right_motor_speed > 0:
                right_motor_speed -= 5
            elif right_motor_speed < 0:
                right_motor_speed += 5

            """pressed = pygame.key.get_pressed()
            print(pressed)
            if pressed[pygame.K_LEFT]:
                print("LEFT")
                left_motor_speed -= 15
                right_motor_speed += 15
            elif pressed[pygame.K_RIGHT]:
                left_motor_speed += 15
                right_motor_speed -= 15

            if pressed[pygame.K_UP]:
                print("UP")
                left_motor_speed += 10
                right_motor_speed += 10
            elif pressed[pygame.K_DOWN]:
                left_motor_speed -= 10
                right_motor_speed -= 10

            if pressed[pygame.K_SPACE]:
                left_motor_speed = 0
                right_motor_speed = 0
            if pressed[pygame.K_ESCAPE]:
                m1.set(0)
                m2.set(0)
                pygame.quit()
                exit()"""

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        print("left")
                        left_motor_speed -= 10
                        right_motor_speed += 10
                    elif event.key == pygame.K_RIGHT:
                        print("right")
                        left_motor_speed += 10
                        right_motor_speed -= 10
                    if event.key == pygame.K_UP:
                        print("up")
                        left_motor_speed += 10
                        right_motor_speed += 10
                    elif event.key == pygame.K_DOWN:
                        print("down")
                        left_motor_speed -= 10
                        right_motor_speed -= 10
                    
                    if event.key == pygame.K_SPACE:
                        print("break")
                        left_motor_speed = 0
                        right_motor_speed = 0
                    if event.key == pygame.K_ESCAPE:
                        m1.set(0)
                        m2.set(0)
                        pygame.quit()
                        keep_running_us.value = False
                        exit()

        if gui:
            render(screen, arduino.ir0, arduino.ir1, us0.value, us1.value, us2.value, us3.value,
                   arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4,
                   left_motor_speed, right_motor_speed)

        m1.set(left_motor_speed)
        m2.set(right_motor_speed)


if __name__ == "__main__":
    main()
