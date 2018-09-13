import time
from multiprocessing import Process, Value
from EEVEE.USHandler import us_async
from EEVEE.MotorHandler import MotorActuator
from EEVEE.LedHandler import LedActuator
import RPi.GPIO as GPIO
from EEVEE.ArduinoHandler import ArduinoHandler


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # asynchronously update US sensors
    us0, us1, us2, us3 = Value('f', 0), Value('f', 0), Value('f', 0), Value('f', 0)
    Process(target=us_async, args=(13, 16, us0, 11, 18, us1, 7, 22, us2, 12, 24, us3)).start()

    # motors
    m1 = MotorActuator(35, 37, 33)
    m2 = MotorActuator(40, 38, 32)

    # LED
    led0 = LedActuator(26)

    # Arduino
    arduino = ArduinoHandler()

    while True:
        arduino.get()
        print(us0.value, us1.value, us2.value, us3.value)
        print(arduino.ir0, arduino.ir1)
        print(arduino.button0, arduino.button1)
        print(arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4)
        print(arduino.m1_encoder, arduino.m2_encoder)

        m1.set(10)
        m2.set(-10)

        led0.set(us1.value > 0.20)

if __name__ == "__main__":
    main()
